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
uint8_t LedData[MAX_LED][4];  	// matrix of 4 columns, number of rows = number of LEDs we have
uint16_t alreadySent;  	// track how much leds received data
uint32_t currDataId;
uint16_t pwmData[24*2];


// set RGB values in the LedData array, LEDnum - id of the led,
// Red, Green, Blue <- color values. take values in range [0-255]
void SetLED (int LEDnum, int Red, int Green, int Blue);

// sets number on the matrices. Sets values in LedData,
// uses digits representation from digits.h
void buildNum(uint8_t num1, uint8_t num2);

// sets all values in LedData to 0.
// after, sends data to matrix
void WsReset(void);


// tester function, sets data for all LEDs and sends it to matrices
void WsFullSet();

// builds exclamation point on the matrices
void buildExclamationIntoNumbers();

// sets speed limit for the velocity, speeds over this
// builds numbers, that represent velocity.
// sets LED
void showVelocity(float currVelocity);

// Sets LEDs that will represent an image of the sign.
// Reads 2D RGB array and sets values
// After sends data
void WsImgSet(int matrix[256][3]);

// Sets LedData for digit representation on the left side of the sign
//works with LedData
void WsAddNumberToSignLeft(int number);

// Sets LedData for digit representation on the right side of the sign
//works with LedData
void WsAddNumberToSignRight(int number);

// sets boundaries of the sign, works with LedData
void WsSetSignPartly(int from);

// Main function that sets matrices
// calls showVelocity(), WsAddNumberToSignLeft(),
// WsAddNumberTosignRight(), WsSetSignPartly()
// Sends LedData to matrices
void WsSetSign(int curr_velocity);


#endif /* INC_WS_MATRIX_H_ */
