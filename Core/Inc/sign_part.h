/*
 * sign_part.h
 *
 *  Created on: Jan 13, 2022
 *      Author: ostap
 */

#ifndef INC_SIGN_PART_H_
#define INC_SIGN_PART_H_

int sign[16][8] = {
	 {0, 0, 0, 0, 0, 1, 1, 1},
	 {0, 0, 0, 1, 1, 1, 0, 0},
	 {0, 0, 0, 1, 0, 0, 0, 0},
	 {0, 0, 1, 1, 0, 0, 0, 0},
	 {0, 1, 1, 0, 0, 0, 0, 0},
	 {0, 1, 0, 0, 0, 0, 0, 0},
	 {1, 1, 0, 0, 0, 0, 0, 0},
	 {1, 0, 0, 0, 0, 0, 0, 0},
	 {1, 0, 0, 0, 0, 0, 0, 0},
	 {1, 1, 0, 0, 0, 0, 0, 0},
	 {0, 1, 0, 0, 0, 0, 0, 0},
	 {0, 1, 1, 0, 0, 0, 0, 0},
	 {0, 0, 1, 1, 0, 0, 0, 0},
	 {0, 0, 0, 1, 0, 0, 0, 0},
	 {0, 0, 0, 1, 1, 1, 0, 0},
	 {0, 0, 0, 0, 0, 1, 1, 1}
};

uint8_t exclamationPoint[8][2]={
	{2,2},
	{2,2},
	{2,2},
	{2,2},
	{2,2},
	{2,2},
	{0,0},
	{2,2}
 };

#endif /* INC_SIGN_PART_H_ */
