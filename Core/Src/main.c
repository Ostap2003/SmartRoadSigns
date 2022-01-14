/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        opensource.org/licenses/BSD-3-Clause
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "WS_matrix.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim4;
DMA_HandleTypeDef hdma_tim4_ch1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM4_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


/* Speed radar code START*/
volatile uint32_t tim1_overflows = 0;

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef * htim) {
    if (htim -> Instance == TIM1) {
        ++tim1_overflows;
    }
}

extern void initialise_monitor_handles(void);  // for semihosting

const unsigned long analysis_every = 500;  // ms
const unsigned int blink_every = 30;

#define VelBufferSize 40 // max LEDs that we have in a cascade

uint32_t RiseVal = 0;
uint32_t FallVal = 0;
uint32_t Difference = 0;
int Rise_Captured = 0;
float velocities[VelBufferSize];
int currVelocityIndex = 0;

int NumOfVelocities=0;

int Rises[VelBufferSize];
int Falls[VelBufferSize];
float Frequencies[VelBufferSize];
volatile int Differences[VelBufferSize];
uint32_t timOverflows[VelBufferSize];

void pushVelocity(float currVelocity, int currRise, int currFall, int currDifference, float currFrequnecy, uint32_t currOverflows) {
		velocities[currVelocityIndex] = currVelocity;
		Rises[currVelocityIndex] = currRise;
		Falls[currVelocityIndex] = currFall;
		Differences[currVelocityIndex] = currDifference;
		Frequencies[currVelocityIndex] = currFrequnecy;
		timOverflows[currVelocityIndex] = currOverflows;
		currVelocityIndex++;
		if (currVelocityIndex >VelBufferSize-1){
			currVelocityIndex = 0;
		}
}
void reloadVelBuffer() {
	currVelocityIndex=0;
	for(int i=0;i<VelBufferSize;i++){
		velocities[i] = 0;
				Rises[i] = 0;
				Falls[i] = 0;
				Differences[i] = 0;
				Frequencies[i] = 0;
				timOverflows[i] = 0;
	}
}


int compare (const void * a, const void * b) {
	  int data1 = *(int *)a, data2 = *(int *)b;
	  if(data1 < data2) // a < b
		return -1;
	  else if(data1 == data2) // a == b
		return 0;
	  else
		return 1;  // a > b
}

float findAvg(float arr[40], int arr_len) {
    // find 25, 75 percentiles
    int lower_percentile = arr_len * 0.25;
    int high_percentile = arr_len * 0.75;
    float sum = 0;
    for (int i = lower_percentile + 1; i < high_percentile; i++) {
        sum += arr[i];
    }
    return sum / (arr_len / 2);
}


float frequency;
float velocity;

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef * htim) // is called whenever rising or falling edge is captured

{
    if (htim -> Channel == HAL_TIM_ACTIVE_CHANNEL_1) // if the interrupt is triggered in channel 1
    {
        if (!Rise_Captured) // if the rise time(RiseVal) is not captured, then it is a rising edge
        {
            RiseVal = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
            RiseVal = HAL_GetTick(); // in miliseconds
            Rise_Captured = 1;
        } else {
            FallVal = 65535 * tim1_overflows + HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
            Difference = FallVal - RiseVal; // duration of high

            // 8Mhz(Clck frequency) / 8(prescaler) = 1Mhz
            // to get frequency of signal divide 1Mhz by its duration
            frequency = 1028500.0 / Difference;

            // velocity = frequency / period_to_frequency;
            velocity = 51308.0/Difference;

            pushVelocity(velocity, RiseVal, FallVal,Difference, frequency,tim1_overflows);
            RiseVal = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
            tim1_overflows = 0;
            Rise_Captured=1;
        }
    }
}

/* Speed radar code END*/

/* Matrix code START*/

void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim) {
	// here we set second part of pwm_data
	uint8_t indx = 24;
	if (already_sent < MAX_LED + 2) {
		uint32_t color;
		color = ((LED_Data[curr_data_id][1]<<16) | (LED_Data[curr_data_id][2]<<8) | (LED_Data[curr_data_id][3]));
		for (int j = 23; j >= 0; j--) {
			if (color&(1<<j)) {
				pwm_data[indx] = 57; // if the bit is 1, the duty cycle is 64%
			} else {
				pwm_data[indx] = 29;  // if the bit is 0, the duty cycle is 32%
			}
			indx++;
		}
		curr_data_id++;
		already_sent++;
	} else if (already_sent < MAX_LED + 2) {
		// HERE MAX LED IN LOOP CAN BE WRONG
		// USE 24 * 2
		for (int i = indx; i < 48; i++) {
			pwm_data[i] = 0;
		}
		already_sent++;
	} else {
		// transfer ended, reset all variables, stop dma
		already_sent = 0;
		curr_data_id = 0;
		HAL_TIM_PWM_Stop_DMA(&htim4, TIM_CHANNEL_1);
	}
}


void HAL_TIM_PWM_PulseFinishedHalfCpltCallback(TIM_HandleTypeDef *htim) {
	uint8_t indx = 0;
	if (already_sent < MAX_LED) {
		uint32_t color;
		color = ((LED_Data[curr_data_id][1]<<16) | (LED_Data[curr_data_id][2]<<8) | (LED_Data[curr_data_id][3]));

		for (int j=23; j>=0; j--) {
			if (color&(1<<j)) {
				pwm_data[indx] = 57; // if the bit is 1, the duty cycle is 64%
			} else {
				pwm_data[indx] = 29;  // if the bit is 0, the duty cycle is 32%
			}
			indx++;
		}
		curr_data_id++;
		already_sent++;
	} else if (already_sent < MAX_LED + 2) {
		for (int i = indx; i < 24; i++) {
			pwm_data[i] = 0;
		}
		already_sent++;
	}
}


void WS2812_Send (void)
{
	uint32_t indx=0;
	uint32_t color;  //32 bit variable to store 24 bits of color

	for (uint16_t i = 0; i < 2; i++) {
		color = ((LED_Data[curr_data_id][1]<<16) | (LED_Data[curr_data_id][2]<<8) | (LED_Data[curr_data_id][3])); // green red blue

		for (int j=23; j>=0; j--) {
			if (color&(1<<j)) {
				pwm_data[indx] = 57; // if the bit is 1, the duty cycle is 64%
			} else {
				pwm_data[indx] = 29;  // if the bit is 0, the duty cycle is 32%
			}
			indx++;
		}
		curr_data_id++;
	}

	HAL_TIM_PWM_Start_DMA(&htim4, TIM_CHANNEL_1, (uint32_t *)pwm_data, indx);
	already_sent += 2;
}
/* Matrix code END*/

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_TIM1_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */

    initialise_monitor_handles();  // for semihosting
    HAL_TIM_Base_Start_IT( & htim1);


    /* Speed radar code START */
    HAL_TIM_IC_Start_IT( & htim1, TIM_CHANNEL_1); // start input capture in interrupt mode for timer 1
    __HAL_TIM_ENABLE_IT( & htim1, TIM_IT_CC1);
    uint32_t analysis_next = HAL_GetTick();

    /* Speed radar code END */

//    WS_set_sign(12);
//    WS_img_set(sign_img);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
    while (1) {

    	        if (HAL_GetTick() >= analysis_next) {
    	            __HAL_TIM_DISABLE_IT( & htim1, TIM_IT_CC1);
    	            Rise_Captured =0;

    	            qsort (velocities, VelBufferSize, sizeof(float), compare);
    	            float avgVel = findAvg(&velocities, VelBufferSize);
    	            printf("AvgVel: %.2f \n", avgVel);


    	            printf("Velocities: \n [");
    	            for (int i = 0; i < VelBufferSize; i ++) {
    	              printf("%.2f, ", velocities[i]);
    	            }
    	            printf("]\n\n");

    	            printf("Freq: \n [");


					for (int i = 0; i < VelBufferSize; i ++) {
					  printf("%.2f, ", Frequencies[i]);
					}
					printf("]\n\n");

    	            reloadVelBuffer();

    	            // showVelocity(avgVel);
    	            // WS_set_sign(avgVel);

    	            analysis_next = HAL_GetTick() + analysis_every;

    	            __HAL_TIM_ENABLE_IT( & htim1, TIM_IT_CC1);
    	        }

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 72-1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim1, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 90-1;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
    /* User can add his own implementation to report the HAL error return state */
    __disable_irq();
    while (1) {}
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
    /* User can add his own implementation to report the file name and line number,
       ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
