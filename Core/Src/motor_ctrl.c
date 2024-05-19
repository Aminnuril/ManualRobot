/*
 * motor_ctrl.c
 *
 *  Created on: Dec 30, 2023
 *      Author: asus
 */

#include "motor_ctrl.h"

/* bldc */
esc roller1 = {.htim = &htim12, .channel = TIM_CHANNEL_1};
esc roller2 = {.htim = &htim12, .channel = TIM_CHANNEL_2};

//omni pg
motor_channel LeftFront = {
		.in1_ = &htim1, .in2_ = &htim1,
		.ch1_ = TIM_CHANNEL_1, .ch2_ = TIM_CHANNEL_2,
		.en_port = EN_LF_GPIO_Port,
		.en_pin = EN_LF_Pin
};
motor_channel LeftBack = {
		.in1_ = &htim1, .in2_ = &htim1,
		.ch1_ = TIM_CHANNEL_4, .ch2_ = TIM_CHANNEL_3,
		.en_port = EN_LB_GPIO_Port,
		.en_pin = EN_LB_Pin
};
motor_channel RightFront = {
		.in1_ = &htim3, .in2_ = &htim3,
		.ch1_ = TIM_CHANNEL_1, .ch2_ = TIM_CHANNEL_2,
		.en_port = EN_RF_GPIO_Port,
		.en_pin = EN_RF_Pin
};
motor_channel RightBack = {
		.in1_ = &htim3, .in2_ = &htim3,
		.ch1_ = TIM_CHANNEL_3, .ch2_ = TIM_CHANNEL_4,
		.en_port = EN_RB_GPIO_Port,
		.en_pin = EN_RB_Pin
};

//mechanism dc motor brushed
motor_channel MPelontar = {
		.in1_ = &htim4, .in2_ = &htim4,
		.ch1_ = TIM_CHANNEL_1, .ch2_ = TIM_CHANNEL_2,
		.en_port = EN_PELONTAR_GPIO_Port,
		.en_pin = EN_PELONTAR_Pin
};
motor_channel MBola = {
		.in1_ = &htim4, .in2_ = &htim4,
		.ch1_ = TIM_CHANNEL_3, .ch2_ = TIM_CHANNEL_4,
		.en_port = EN_BOLA_GPIO_Port,
		.en_pin = EN_BOLA_Pin
};
motor_channel MExtendLeft = {
		.in1_ = &htim5, .in2_ = &htim5,
		.ch1_ = TIM_CHANNEL_1, .ch2_ = TIM_CHANNEL_2,
		.en_port = EN_EXDL_GPIO_Port,
		.en_pin = EN_EXDL_Pin
};
motor_channel MExtendRight = {
		.in1_ = &htim8, .in2_ = &htim8,
		.ch1_ = TIM_CHANNEL_1, .ch2_ = TIM_CHANNEL_2,
		.en_port = EN_EXDR_GPIO_Port,
		.en_pin = EN_EXDR_Pin
};
motor_channel MPadi = {
		.in1_ = &htim9, .in2_ = &htim9,
		.ch1_ = TIM_CHANNEL_2, .ch2_ = TIM_CHANNEL_1,
		.en_port = EN_PADI_GPIO_Port,
		.en_pin = EN_PADI_Pin
};

void motor_init(motor_channel *wheel_n){
	HAL_GPIO_WritePin(wheel_n->en_port, wheel_n->en_pin, GPIO_PIN_SET);
	HAL_TIM_PWM_Start(wheel_n->in1_, wheel_n->ch1_);
	HAL_TIM_PWM_Start(wheel_n->in2_, wheel_n->ch2_);

	__HAL_TIM_SET_COMPARE(wheel_n->in1_, wheel_n->ch1_, 0);
	__HAL_TIM_SET_COMPARE(wheel_n->in2_, wheel_n->ch2_, 0);
}

void motor_drive(motor_channel *wheel_n, int16_t dir, int16_t rpm){
	if(dir > 0){
		__HAL_TIM_SET_COMPARE(wheel_n->in1_, wheel_n->ch1_, rpm);
		__HAL_TIM_SET_COMPARE(wheel_n->in2_, wheel_n->ch2_, 0);
	}
	else if(dir < 0){
		__HAL_TIM_SET_COMPARE(wheel_n->in1_, wheel_n->ch1_, 0);
		__HAL_TIM_SET_COMPARE(wheel_n->in2_, wheel_n->ch2_, abs(rpm));
	}
	else{
		__HAL_TIM_SET_COMPARE(wheel_n->in1_, wheel_n->ch1_, 0);
		__HAL_TIM_SET_COMPARE(wheel_n->in2_, wheel_n->ch2_, 0);
	}
}

void disable_motor(motor_channel *wheel_n){
	HAL_GPIO_WritePin(wheel_n->en_port, wheel_n->en_pin, GPIO_PIN_RESET);
}

void enable_motor(motor_channel *wheel_n){
	HAL_GPIO_WritePin(wheel_n->en_port, wheel_n->en_pin, GPIO_PIN_SET);
}

void bldc_init(esc *servo, uint32_t sp) {
	HAL_TIM_PWM_Start(servo->htim, servo->channel);
	__HAL_TIM_SET_COMPARE(servo->htim, servo->channel, sp);
}

void bldc_drive(esc *servo, uint32_t duty) {
	__HAL_TIM_SET_COMPARE(servo->htim, servo->channel, duty);
}
