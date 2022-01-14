/*
 * WS_matrix.c
 *
 *  Created on: Jan 14, 2022
 *      Author: ostap
 */

/* Matrix code START*/

#include "WS_matrix.h"
#include "digits.h"
#include "sign_part.h"
#include "additional_signs.h"

uint16_t already_sent = 0;  // track how much leds received data
uint32_t curr_data_id = 0;


void Set_LED (int LEDnum, int Red, int Green, int Blue) {
	// store green first as ws2821b requires this order (g,r,b)
	LED_Data[LEDnum][0] = LEDnum;
	LED_Data[LEDnum][1] = Green;
	LED_Data[LEDnum][2] = Red;
	LED_Data[LEDnum][3] = Blue;
}


uint8_t final_matrix[128] ={0};

void build_num(uint8_t num1, uint8_t num2) {
    uint8_t curr_id = 0;
    uint8_t frst_num_id = 0;
    uint8_t sec_num_id = 7;

    while (curr_id != 64) {
        curr_id+=4;

        for (uint8_t j = 0; j < 3; j++) {
            final_matrix[curr_id] = digits[num1][frst_num_id][j];
            curr_id++;
        }

        curr_id++;
        frst_num_id++;
    }

    while (curr_id != 128) {
        curr_id+=4;

        for (uint8_t j = 0; j < 3; j++) {
            final_matrix[curr_id] = digits[num2][sec_num_id][2 - j];
            curr_id++;
        }

        curr_id++;
        sec_num_id--;

    }
}

void WS_Reset(void) {
    for (int i = 0; i < MAX_LED; i++) {
        Set_LED(i, 0, 0, 0);
    }
    WS2812_Send();
}

void WS_Set(int matrix[]) {
    for (int i = 0; i < 64; i++) {
        if (matrix[i] == 1) {
            Set_LED(i, 254, 0, 0);
        } else {
            Set_LED(i, 0, 0, 0);
        }
    }
    for (int j = 64; j < MAX_LED; j++) {
    	Set_LED(j, 0, 0, 0);
    }
    WS2812_Send();
}

void WS_FullSet() {
    for (int i = 0; i < MAX_LED; i++) {
        Set_LED(i, 0, 254, 0);
    }

    WS2812_Send();
}

void buildExclamationIntoNumbers() {
	 uint8_t curr_id = 0;
	 uint8_t excl_id = 0;

	    while (curr_id != 64) {
	        for (uint8_t j = 0; j < 2; j++) {
	            final_matrix[curr_id] = exclamationPoint[excl_id][j];
	            curr_id++;
	        }
	        curr_id+=6;
	        excl_id++;
	    }

	    excl_id=7;
	    while (curr_id != 128) {
	        for (uint8_t j = 0; j < 2; j++) {
	            final_matrix[curr_id] = exclamationPoint[excl_id][1 - j];
	            curr_id++;
	        }
	        curr_id+=6;
	        excl_id--;
	    }
}

int speedLimit = 3;
volatile int limitSurpassed = 0;

void showVelocity(float currVelocity) {
    int firstDigit = fmod((currVelocity / 10), 10);
    int secondDigit = fmod(currVelocity, 10);
    build_num(firstDigit, secondDigit);
    if (!limitSurpassed) {
		for (int i = 0; i < 128; i++) {
			if (final_matrix[i] == 1) {
				Set_LED(i+128, 0, 254, 0);
			} else {
				Set_LED(i+128, 0, 0, 0);
			}
		}
    } else {
		for (int i = 0; i < 128; i++) {
			if (final_matrix[i] == 1) {
				Set_LED(i+128, 254, 0, 0);
			} else if (final_matrix[i]==2){
				Set_LED(i+128, 254, 254, 0);
			} else {
				Set_LED(i+128, 0, 0, 0);
			}
		}
    }
}


void WS_img_set(int matrix[256][3]) {
	for (int i = 0; i < MAX_LED; i++) {
		if(i < 128){
			Set_LED(i, matrix[i][0],matrix[i][1], matrix[i][2]);
		} else if (i > 256){
			Set_LED(i, matrix[i-128][0],matrix[i-128][1], matrix[i-128][2]);
		} else {
			Set_LED(i, 0, 0, 0);
		}
	}
	WS2812_Send();
}

void WS_add_number_to_sign_left(int number) {
	// iterate over array with numebr, start from array[1]
	// for first part of the sign -> start from = 44
	// for second part of the sign -> start from = ?
	int start_from = 44;
	int pixels_to_set = 21;
	while (pixels_to_set != 0) {
		for (int i = 1; i <= 7; i++) {
			for (int j = 0; j < 3; j++) {
				if (digits[number][i][j] == 1) {
					Set_LED(start_from, 100, 100, 0);
				}
				pixels_to_set--;
				start_from++;
			}
			start_from += 5; // skip 5 pixels
		}
	}
}

void WS_add_number_to_sign_right(int number) {
	int start_from = 292;
	int pixels_to_set = 21;
	while (pixels_to_set != 0) {
		for (int i = 7; i >= 1; i--) {
			for (int j = 2; j >= 0; j--) {
				if (digits[number][i][j] == 1) {
					Set_LED(start_from, 100, 100, 0);
				}
				pixels_to_set--;
				start_from++;
			}
			start_from += 5; // skip 5 pixels
		}
	}
}

void WS_set_sign_partly(int from) {
	// sets first and last two matricies
	int  curr_id = from;
	for (int i = 0; i < 16; i++) {
		for (int j = 0; j < 8; j++) {
			if (sign[i][j] == 1) {
				Set_LED(curr_id, 100, 0, 0);
			} else {
				Set_LED(curr_id, 0, 0, 0);
			}
			curr_id++;
		}
	}
}

void WS_set_sign(int curr_velocity) {
	WS_set_sign_partly(0);
	WS_add_number_to_sign_left(9);
	// two matricies are ready
	showVelocity(curr_velocity);
	// four matricies are ready
	WS_set_sign_partly(256);
	WS_add_number_to_sign_right(0);

	WS2812_Send();
}

/* Matrix code END */
