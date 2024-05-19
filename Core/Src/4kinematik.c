/*
 * 4kinematik.c
 *
 *  Created on: Dec 22, 2023
 *      Author: asus
 */
#include "4kinematik.h"

double out[4];

void kinematikM(int x, int y, int th, double nos) {
	out[0] = nos*2*(-sin(d2r(135))*x + cos(d2r(135))*y + 0.25*th);
	out[1] = nos*2*(-sin(d2r(225))*x + cos(d2r(225))*y + 0.25*th);
	out[2] = nos*2*(-sin(d2r(315))*x + cos(d2r(315))*y + 0.25*th);
	out[3] = nos*2*(-sin(d2r(45))*x + cos(d2r(45))*y + 0.25*th);
}
