/*
 * stepper_pwm.h
 *
 *  Created on: Mar 30, 2025
 *      Author: Andre
 */

#ifndef INC_STEPPER_PWM_H_
#define INC_STEPPER_PWM_H_

// pwm_utils.h
#include "stm32l4xx_hal.h"

extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern uint8_t stepper_active;

void PWM_Init(void);
void Start_PWM_For_N_Periods(uint32_t N);

#endif /* INC_STEPPER_PWM_H_ */
