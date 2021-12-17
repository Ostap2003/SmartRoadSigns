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

volatile uint32_t tim1_overflows = 0;

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef * htim) {
    if (htim -> Instance == TIM1) {
        ++tim1_overflows;
    }
}
void TIM1_reinit() {
    HAL_TIM_Base_Stop( & htim1);
    __HAL_TIM_SET_COUNTER( & htim1, 0);
    tim1_overflows = 0;
    HAL_TIM_Base_Start_IT( & htim1);
}

extern void initialise_monitor_handles(void);


//const unsigned long print_every = 300; // ms

const unsigned long analysis_every = 100; // ms
const float period_to_frequency = 70.235; // Hz/m/s

uint32_t RiseVal = 0;
uint32_t FallVal = 0;
uint32_t Difference = 0;
int Rise_Captured = 0;

float velocities[4];
int currVelocityIndex = 0;

void pushVelocity(float currVelocity) {
	velocities[currVelocityIndex] = currVelocity;
	currVelocityIndex++;
	if (currVelocityIndex > 3){
		currVelocityIndex = 0;
	}
}

float averageVelocity(){
	float sum = 0;
	for (int i=0;i<4;i++){
		sum+= velocities[i];
	}
	return sum/4;
}

int check_delta(float curr_velocity, float velocity_to_check) {
	// set some epsilon, as all velocities can vary +- 0.5 km/h
	float delta = 0.5;
	float left_bound = curr_velocity - delta;
	float right_bound = curr_velocity + delta;
	if (velocity_to_check + delta < left_bound || velocity_to_check - delta > right_bound) {
		return 0;
	}
	return 1;
}

float modeVelocity() {
	// here we store number of similar velocities that are delta-close to velocity stored in velocities[curr_id]
	float freq_list[10] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
	for (int i = 0; i < 10; i++) {
		for (int j = 0; j < 10; j++) {
			if (check_delta(velocities[i], velocities[j])) {
				freq_list[i]++;
			}
		}
	}

	float max_freq = -1;
	int max_freq_id;
	for (int i = 0; i < 10; i++) {
		if (freq_list[i] > max_freq) {
			max_freq = freq_list[i];
			max_freq_id = i;
		}
	}
	return velocities[max_freq_id];
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

            Rise_Captured = 1;
            // Change the polarity to the opposite - falling edge.
            __HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_FALLING);
        } else // if the rise time(RiseVal) is captured, then it is a falling edge
        {
            FallVal = 10000 * tim1_overflows + HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);

            if (FallVal > RiseVal) {
                Difference = FallVal - RiseVal; // duration of high
            } else if (RiseVal > FallVal) {
                // 65535 - current Auto Reload Register
                Difference = (10000 - RiseVal) + FallVal; // duration of high

            }

            Rise_Captured = 0; // change it back to false

            frequency = 1000000.0 / Difference;
            velocity = frequency / period_to_frequency;

            pushVelocity(velocity);
            // set polarity to capture the next rising edge
            __HAL_TIM_SET_COUNTER(htim, 0); // reset the counter
            TIM1_reinit();
            __HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_RISING);
        }
    }
}


#define MAX_LED 300 // max LEDs that we have in a cascade

uint8_t LED_Data[MAX_LED][3];  // matrix of 4 columns, number of rows = number of LEDs we have

int datasentflag=0;  // to make sure that the dma does not send another data while the first data is still transmitted


void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)  // this callback is called when data transmission is finished
{
	HAL_TIM_PWM_Stop_DMA(&htim4, TIM_CHANNEL_1);  // stop dma, when the transmission is finished
	datasentflag = 1;
}

void Set_LED (int LEDnum, int Red, int Green, int Blue)
{
	LED_Data[LEDnum][0] = LEDnum;
	LED_Data[LEDnum][1] = Green;  // store green first as ws2821b requires this order (g,r,b)
	LED_Data[LEDnum][2] = Red;
	LED_Data[LEDnum][3] = Blue;
}



// uint16_t pwmData[(24*MAX_LED)+50]; // store 24 bits for each led + 50 values for reset code

uint16_t pwmData[(24*MAX_LED) + 50];

void WS2812_Send (void)
{
	uint32_t indx=0;
	uint32_t color;  //32 bit variable to store 24 bits of color

	for (uint16_t i = 0; i<MAX_LED; i++)  // iterate through all of the LEDs
		{

			color = ((LED_Data[i][1]<<16) | (LED_Data[i][2]<<8) | (LED_Data[i][3])); // green red blue


			for (int i=23; i>=0; i--) // iterate through the 24 bits which specify the color
			{
				if (color&(1<<i))
				{
					pwmData[indx] = 6; // if the bit is 1, the duty cycle is 64%
				}

				else pwmData[indx] = 3;  // if the bit is 0, the duty cycle is 32%

				indx++;
			}

		}
	for (uint16_t i=0; i<50; i++)  // store values to keep the pulse low for 50+ us, reset code
	{
		pwmData[indx] = 0;
		indx++;
	}
	HAL_TIM_PWM_Start_DMA(&htim4, TIM_CHANNEL_1, (uint32_t *)pwmData, indx);  // send the data to the dma
	while (!datasentflag){};  // this flag will be set when the data transmission is finished, dma is stopped and now we can send another data
	datasentflag = 0;
}



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
    initialise_monitor_handles();
    HAL_TIM_IC_Start_IT( & htim1, TIM_CHANNEL_1); // start input capture in interrupt mode for timer 1
    __HAL_TIM_ENABLE_IT( & htim1, TIM_IT_CC1);

    uint32_t analysis_next = HAL_GetTick();


    void WS_Reset(void){
    	for (int i=0; i < MAX_LED; i++){
        Set_LED(i, 0, 0, 0);
      }
    	WS2812_Send();
    }


    void WS_FullSet(){
        for (uint16_t i=0; i < MAX_LED; i++){
            Set_LED(i, 254, 0, 0);
        }
        WS2812_Send();

    }

//    int zero[8][3] = {
//    		{1, 1, 1},
//			{1, 0, 1},
//			{1, 0, 1},
//			{1, 0, 1},
//			{1, 0, 1},
//			{1, 0, 1},
//			{1, 0, 1},
//			{1, 1, 1},
//    };
//
//    int one[8][3] = {
//		{0, 1, 0},
//		{0, 1, 0},
//		{0, 1, 0},
//		{0, 1, 0},
//		{0, 1, 0},
//		{0, 1, 0},
//		{0, 1, 0},
//		{0, 1, 0}
//    };
//
//    int two[8][3] = {
//    		{1, 1, 1},
//    		{1, 0, 1},
//    		{0, 0, 1},
//    		{1, 1, 1},
//    		{1, 0, 0},
//    		{1, 0, 0},
//    		{1, 0, 0},
//    		{1, 1, 1}
//    };
//
//    int digits[3] = {zero, one, two};
//    int final_matrix[64];
//
//    void build_num(int num1, int num2) {
//    	int mtr_num1 = digits[num1];
//    	int mtr_num2 = digits[num2];
//
//    	int curr_id = 0;
//    	int frst_num_id = 0;
//    	int sec_num_id = 0;
//
//    	while (curr_id != 64) {
//			for (int j = 0; j < 3; j++) {
//				final_matrix[curr_id] = mtr_num1[frst_num_id][j];
//				curr_id++;
//			}
//
//			frst_num_id++;
//			final_matrix[curr_id] = 0;
//			curr_id++;
//
//			for (int i = 0; i < 2; i++) {
//				final
//			}
//
//			for (int j = 0; j < 3; j++) {
//				final_matrix[curr_id] = mtr_num2[sec_num_id][j];
//				curr_id++;
//			}
//			sec_num_id++;
//    	}
//    }


    WS_FullSet();
    //printf("SIZE\n");
    //printf("%lu\n", sizeof(pwmData));

    //WS_Reset();
    //printf("AFTER SIZE\n");
    //printf("%lu\n", sizeof(pwmData));

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
    while (1) {

        if (HAL_GetTick() >= analysis_next) {
            __HAL_TIM_DISABLE_IT( & htim1, TIM_IT_CC1);
            float avgVel = averageVelocity();
//            float mode_vel = modeVelocity();
            // printf("%f %f %f %f \n",velocities[0],velocities[1],velocities[2],velocities[3]);
            //printf("2123Velocity: %.2f, freq: %.2f Riseval %d Fall %d  Diff %d \n", avgVel, frequency, RiseVal, FallVal, Difference);

            printf("Velocity: %.2f, freq: %.2f Riseval %d Fall %d  Diff %d \n", avgVel, frequency, RiseVal, FallVal, Difference);


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
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
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

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 8-1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 10000;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
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
