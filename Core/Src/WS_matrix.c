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

uint16_t alreadySent = 0;  // track how much leds received data
uint32_t currDataId = 0;


void SetLED (int LEDnum, int Red, int Green, int Blue) {
	// store green first as ws2821b requires this order (g,r,b)
	LedData[LEDnum][0] = LEDnum;
	LedData[LEDnum][1] = Green;
	LedData[LEDnum][2] = Red;
	LedData[LEDnum][3] = Blue;
}


uint8_t finalMatrix[128] ={0};

void buildNum(uint8_t num1, uint8_t num2) {


    uint8_t currId = 0;
    uint8_t frstNumId = 0;
    uint8_t secNumId = 7;

    while (currId != 64) {
        currId+=4;

        if (!num1){ // don't show zero as the first digit of numbers smaller than 10.
        		for (uint8_t j = 0; j < 3; j++) {
        		            finalMatrix[currId] = 0;
        		            currId++;
        		 }
        }else{
        	for (uint8_t j = 0; j < 3; j++) {
        	            finalMatrix[currId] = digits[num1][frstNumId][j];
        	            currId++;
        	        }
        }
        currId++;
        frstNumId++;
    }

    while (currId != 128) {
        currId+=4;
        if ((!num1) && (!num2)){ // don't show zero if it is the velocity
               for (uint8_t j = 0; j < 3; j++) {
                finalMatrix[currId] = 0;
                currId++;
               }
        }
         else{
				for (uint8_t j = 0; j < 3; j++) {
					finalMatrix[currId] = digits[num2][secNumId][2 - j];
					currId++;
				}
               }
        currId++;
        secNumId--;

    }
}

void WsReset(void) {
    for (int i = 0; i < MAX_LED; i++) {
        SetLED(i, 0, 0, 0);
    }
    WS2812_Send();
}



void WsFullSet() {
    for (int i = 0; i < MAX_LED; i++) {
        SetLED(i, 0, 254, 0);
    }

    WS2812_Send();
}

void buildExclamationIntoNumbers() {
	 uint8_t currId = 0;
	 uint8_t exclId = 0;

	    while (currId != 64) {
	        for (uint8_t j = 0; j < 2; j++) {
	            finalMatrix[currId] = exclamationPoint[exclId][j];
	            currId++;
	        }
	        currId+=6;
	        exclId++;
	    }

	    exclId=7;
	    while (currId != 128) {
	        for (uint8_t j = 0; j < 2; j++) {
	            finalMatrix[currId] = exclamationPoint[exclId][1 - j];
	            currId++;
	        }
	        currId+=6;
	        exclId--;
	    }
}

int speedLimit = 10;
volatile int limitSurpassed = 0;

void setSpeedLimit(int limit) {speedLimit = limit;}

void showVelocity(float currVelocity) {
    if (currVelocity>speedLimit){
       limitSurpassed=1;
       }else{
       limitSurpassed=0;
    }
    int firstDigit = fmod((currVelocity / 10), 10);
    int secondDigit = fmod(currVelocity, 10);
    buildNum(firstDigit, secondDigit);

    if (!limitSurpassed) {
		for (int i = 0; i < 128; i++) {
			if (finalMatrix[i] == 1) {
				SetLED(i+128, 0, 254, 0);
			} else {
				SetLED(i+128, 0, 0, 0);
			}
		}
    } else {
		for (int i = 0; i < 128; i++) {
			if (finalMatrix[i] == 1) {
				SetLED(i+128, 254, 0, 0);
			} else if (finalMatrix[i]==2){
				SetLED(i+128, 254, 254, 0);
			} else {
				SetLED(i+128, 0, 0, 0);
			}
		}
    }
}


void WsImgSet(int matrix[256][3]) {
	for (int i = 0; i < MAX_LED; i++) {
		if(i < 128){
			SetLED(i, matrix[i][0],matrix[i][1], matrix[i][2]);
		} else if (i > 256){
			SetLED(i, matrix[i-128][0],matrix[i-128][1], matrix[i-128][2]);
		} else {
			SetLED(i, 0, 0, 0);
		}
	}
	WS2812_Send();
}

void WsAddNumberToSignLeft(int number) {
	// iterate over array with number, start from array[1]
	// for first part of the sign -> start from = 44
	// for second part of the sign -> start from = ?
	int startFrom = 44;
	int pixelsToSet = 21;
	while (pixelsToSet != 0) {
		for (int i = 1; i <= 7; i++) {
			for (int j = 0; j < 3; j++) {
				if (digits[number][i][j] == 1) {
					SetLED(startFrom, 100, 100, 0);
				}
				pixelsToSet--;
				startFrom++;
			}
			startFrom += 5; // skip 5 pixels
		}
	}
}

void WsAddNumberToSignRight(int number) {
	int startFrom = 292;
	int pixelsToSet = 21;
	while (pixelsToSet != 0) {
		for (int i = 7; i >= 1; i--) {
			for (int j = 2; j >= 0; j--) {
				if (digits[number][i][j] == 1) {
					SetLED(startFrom, 100, 100, 0);
				}
				pixelsToSet--;
				startFrom++;
			}
			startFrom += 5; // skip 5 pixels
		}
	}
}

void WsSetSignPartly(int from) {
	// sets first and last two matricies
	int  currId = from;
	for (int i = 0; i < 16; i++) {
		for (int j = 0; j < 8; j++) {
			if (sign[i][j] == 1) {
				SetLED(currId, 100, 0, 0);
			} else {
				SetLED(currId, 0, 0, 0);
			}
			currId++;
		}
	}
}

void WsSetSign(int currVelocity) {
	WsSetSignPartly(0);
    int firstDigitOfLimit = fmod((speedLimit / 10), 10);
    	WsAddNumberToSignLeft(1);
	// two matrices are ready
	showVelocity(currVelocity);
	// four matrices are ready
	WsSetSignPartly(256);
	int secondDigitOfLimit = fmod(speedLimit, 10);
	WsAddNumberToSignRight(0);

	WS2812_Send();
}

/* Matrix code END */
