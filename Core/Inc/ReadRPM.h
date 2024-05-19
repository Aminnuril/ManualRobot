/*
 * ReadRPM.h
 *
 *  Created on: Feb 16, 2024
 *      Author: asus
 */

#ifndef INC_READRPM_H_
#define INC_READRPM_H_

#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_gpio.h"
#include "stm32f4xx_hal_tim.h"
#include "main.h"
#include "stdint.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct __attribute__((__packed__)) RPM_motor_struct {
	uint32_t start_time, prev_time, sample_time;
	int pulse_enc, pulse_dist, pulse_tim;
	double RPM, d_wheel, ppr;
	GPIO_TypeDef *port_enc_1, *port_enc_2;
	uint16_t pin_enc_1, pin_enc_2;
} RPM_motor;

extern RPM_motor L_front, R_front, L_back, R_back;

void calculate_interrupt_RPM(RPM_motor *motor);
double calculate_timer_RPM(RPM_motor *motor, TIM_HandleTypeDef *encoder_cnt);
void read_pulse_A(RPM_motor *motor);
void read_pulse_B(RPM_motor *motor);
int map(int value, int fromLow, int fromHigh, int toLow, int toHigh);

#ifdef __cplusplus
}
#endif

#endif /* INC_READRPM_H_ */
