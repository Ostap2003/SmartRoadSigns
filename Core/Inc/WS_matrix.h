/*
 * WS_matrix.h
 *
 *  Created on: Jan 14, 2022
 *      Author: ostap
 */

#include "main.h"


#ifndef INC_WS_MATRIX_H_
#define INC_WS_MATRIX_H_

#define MAX_LED 384 			// max LEDs that we have in a cascade
uint8_t LED_Data[MAX_LED][4];  	// matrix of 4 columns, number of rows = number of LEDs we have
uint16_t already_sent;  	// track how much leds received data
uint32_t curr_data_id;
uint16_t pwm_data[24*2];


// set RGB values in the LED_Data array, LED_num - id of the led,
// Red, Green, Blue <- color values. take values in range [0-255]
void Set_LED (int LEDnum, int Red, int Green, int Blue);

// sets number on the matrices. Sets values in LED_Data,
// uses digits representation from digits.h
void build_num(uint8_t num1, uint8_t num2);

// sets all values in LED_Data to 0.
// after, sends data to matrix
void WS_Reset(void);

void WS_Set(int matrix[]);

// tester function, sets data for all LEDs and sends it to matrices
void WS_FullSet();

// builds exclamation point on the matrices
void buildExclamationIntoNumbers();

// builds numbers, that represent velocity.
// sets LED
void showVelocity(float currVelocity);

// Sets LEDs that will represent an image of the sign.
// Reads 2D RGB array and sets values
// After sends data
void WS_img_set(int matrix[256][3]);

// Sets LED_Data for digit representation on the left side of the sign
//works with LED_Data
void WS_add_number_to_sign_left(int number);

// Sets LED_Data for digit representation on the right side of the sign
//works with LED_Data
void WS_add_number_to_sign_right(int number);

// sets boundaries of the sign, works with LED_Data
void WS_set_sign_partly(int from);

// Main function that sets matrices
// calls ShowVelocity(), WS_add_number_to_sign_left(),
// WS_add_number_to_sign_right(), WS_set_sign_partly()
// Sends LED_Data to matrices
void WS_set_sign(int curr_velocity);


#endif /* INC_WS_MATRIX_H_ */
