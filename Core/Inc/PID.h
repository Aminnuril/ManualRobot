/*
 * PID.h
 *
 *  Created on: Feb 17, 2024
 *      Author: asus
 */

#ifndef INC_PID_H_
#define INC_PID_H_

#include "stm32f4xx.h"
#include "stm32f4xx_hal.h"
#include "main.h"
#include "stdint.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct __attribute__((__packed__)) PID_struct {
	double kp, ki, kd;
	double sum_max, sum_min;
	double error, out_sum, d_input;
	double setpoint, feedback, last_feedback;
	double max_Out, min_Out;
	uint32_t d_time, t_prev, t_now;
	int sample_time, mode, output;
} PID;

//typedef struct __attribute__((__packed__)) HIGH_PASS_struct {
//	double weight, last_out, out;
//	uint32_t d_time, t_prev, t_now, sample_t;
//} Filter;

//jangan lupa, habis dideklarasi, didefinisikan
extern PID lf, rf, lb, rb;

void PID_Compute(PID *motor, double set_point, double feed_back);
//void Filter_func(Filter *obj, const double *input);

#ifdef __cplusplus
}
#endif

#endif /* INC_PID_H_ */
