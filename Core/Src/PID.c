/*
 * PID.c
 *
 *  Created on: Feb 17, 2024
 *      Author: asus
 */

#include "PID.h"
#include "math.h"
#include "stdlib.h"
#include "stdbool.h"

PID lf = {
		.sample_time = 100, .kp = 0, .ki = 0, .kd = 0,
		.min_Out = 0, .max_Out = 1000
};

PID rf = {
		.sample_time = 100, .kp = 0, .ki = 0, .kd = 0,
		.min_Out = 0, .max_Out = 1000
};

PID lb = {
		.sample_time = 100, .kp = 0, .ki = 0, .kd = 0,
		.min_Out = 0, .max_Out = 1000
};

PID rb = {
		.sample_time = 100, .kp = 0, .ki = 0, .kd = 0,
		.min_Out = 0, .max_Out = 1000
};


void PID_Compute(PID *motor, double set_point, double feed_back){
	motor->t_now = HAL_GetTick();
	motor->d_time = motor->t_now - motor->t_prev;
	if(motor->d_time >= motor->sample_time){
		motor->setpoint = set_point; //setpoint user
		motor->feedback = feed_back; //sensor

		motor->error = motor->setpoint - motor->feedback;
		motor->d_input = motor->feedback - motor->last_feedback;
		motor->out_sum += (motor->ki * motor->error);

		if(motor->out_sum > motor->max_Out) motor->out_sum = motor->max_Out;
		else if(motor->out_sum < motor->min_Out) motor->out_sum = motor->min_Out;
		//PID output calculation
		motor->output = motor->kp * motor->error;
		motor->output += (motor->out_sum - (motor->kd * motor->d_input));
		//max & min output PID value
		if(motor->output > motor->max_Out) motor->output = (int)motor->max_Out;
		else if(motor->output < motor->min_Out) motor->output = (int)motor->min_Out;

		motor->last_feedback = motor->feedback;
		motor->t_prev = motor->t_now;
	}
}

//void Filter_func(Filter *obj, const double *input){
//	obj->t_now = HAL_GetTick();
//	obj->d_time = obj->t_now - obj->t_prev;
//	if(obj->d_time >= obj->sample_t){
//		obj->out = (obj->weight * (*input)) + ((1 - obj->weight) * obj->last_out);
//		obj->last_out = obj->out;
//		obj->t_prev = obj->t_now;
//	}
//}
