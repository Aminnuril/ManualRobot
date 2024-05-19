/*
 * motor_control.h
 *
 *  Created on: Dec 30, 2023
 *      Author: asus
 */

#ifndef INC_MOTOR_CTRL_H_
#define INC_MOTOR_CTRL_H_

#include "main.h"
#include "stm32f4xx_hal_gpio.h"
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_tim.h"
#include "stdint.h"
#include "stdlib.h"
#include "stdbool.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct __attribute__((__packed__)) motor_channel_struct {
	TIM_HandleTypeDef* in1_;
	TIM_HandleTypeDef* in2_;
	uint32_t ch1_;
	uint32_t ch2_;
	GPIO_TypeDef *en_port;
	volatile uint16_t en_pin;
} motor_channel;

typedef struct __attribute__((__packed__)) escStruct{
	TIM_HandleTypeDef *htim;
	uint32_t channel;
} esc;

extern motor_channel LeftFront, LeftBack, RightFront, RightBack, MBola, MPelontar, MExtendLeft, MExtendRight, MPadi;
extern esc roller1, roller2;
extern TIM_HandleTypeDef htim1, htim3, htim4, htim5, htim8, htim9, htim12;

void bldc_init(esc *servo, uint32_t sp);
void bldc_drive(esc *servo, uint32_t duty);
void motor_init(motor_channel *wheel_n);
void motor_drive(motor_channel *wheel_n, int16_t dir, int16_t rpm);
void disable_motor(motor_channel *wheel_n);
void enable_motor(motor_channel *wheel_n);

#ifdef __cplusplus
}
#endif

#endif /* INC_MOTOR_CTRL_H_ */
