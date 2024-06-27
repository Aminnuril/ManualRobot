/*
 * kinematik.h
 *
 *  Created on: Dec 22, 2023
 *      Author: asus
 */

#ifndef INC_4KINEMATIK_H_
#define INC_4KINEMATIK_H_

extern double out[4];

#ifdef __cplusplus
extern "C" {
#endif

#include <math.h>

//#define lambda 20
#define d2r(x) x*(M_PI/180)

void kinematikM1(int x, int y, int th, double nos);
void kinematikM2(int x, int y, int th, double nos);

#ifdef __cplusplus
}
#endif

#endif /* INC_4KINEMATIK_H_ */
