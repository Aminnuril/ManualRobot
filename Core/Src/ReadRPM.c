/*
 * ReadRPM.c
 *
 *  Created on: Feb 16, 2024
 *      Author: asus
 */
#include "ReadRPM.h"
#include "math.h"

RPM_motor L_front = {
		.sample_time = 100, .pulse_enc = 0,
		.port_enc_1 = ENC1_A_GPIO_Port, .port_enc_2 = ENC1_B_GPIO_Port,
		.pin_enc_1 = ENC1_A_Pin, .pin_enc_2 = ENC1_B_Pin,
		.d_wheel = 0.1, .pulse_dist = 0, .ppr = 14
};
RPM_motor R_front = {
		.sample_time = 100, .pulse_enc = 0,
		.port_enc_1 = ENC2_A_GPIO_Port, .port_enc_2 = ENC2_B_GPIO_Port,
		.pin_enc_1 = ENC2_A_Pin, .pin_enc_2 = ENC2_B_Pin,
		.d_wheel = 0.1, .pulse_dist = 0, .ppr = 14
};
RPM_motor R_back = {
		.sample_time = 100, .pulse_enc = 0,
		.port_enc_1 = ENC1_A_GPIO_Port, .port_enc_2 = ENC1_B_GPIO_Port,
		.pin_enc_1 = ENC1_A_Pin, .pin_enc_2 = ENC1_B_Pin,
		.d_wheel = 0.1, .pulse_dist = 0, .ppr = 14
};
RPM_motor L_back = {
		.sample_time = 100, .pulse_enc = 0,
		.port_enc_1 = ENC4_B_GPIO_Port, .port_enc_2 = ENC4_A_GPIO_Port,
		.pin_enc_1 = ENC4_B_Pin, .pin_enc_2 = ENC4_A_Pin,
		.d_wheel = 0.1, .pulse_dist = 0, .ppr = 14
};

void calculate_interrupt_RPM(RPM_motor *motor){
	motor->start_time = HAL_GetTick();
	if((motor->start_time - motor->prev_time) >= (1000 / motor->sample_time)){
		motor->prev_time = motor->start_time;
		motor->RPM = (double)(motor->pulse_enc * motor->sample_time * 60) / motor->ppr * 2;
		motor->pulse_enc = 0;
	}
}

void read_pulse_A(RPM_motor *motor){
	if((HAL_GPIO_ReadPin(motor->port_enc_2, motor->pin_enc_2)) == GPIO_PIN_RESET){
		motor->pulse_enc++;
	}
	else{
		motor->pulse_enc--;
	}
}

void read_pulse_B(RPM_motor *motor){
	if((HAL_GPIO_ReadPin(motor->port_enc_1, motor->pin_enc_1)) == GPIO_PIN_RESET){
		motor->pulse_enc--;
	}
	else{
		motor->pulse_enc++;
	}
}
