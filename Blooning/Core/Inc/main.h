/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32l4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */
#define TIM4_ADDR 0x40000800//timer 4 base register
#define TIM_CCR2_OFFSET 0x38//capture/compare register 2
#define CCR_MASK 0xFFFF
#define SERVO_MIN 50
#define SERVO_MAX 260
#define SERVO_MID (SERVO_MIN + SERVO_MAX)/2
//TODO: verify that CW and CCW directions are correct
#define STEP_CW 1
#define STEP_CCW 0
#define DPAD_UP_MASK (1<<4)
#define DPAD_RIGHT_MASK (1<<5)
#define DPAD_DOWN_MASK (1<<6)
#define DPAD_LEFT_MASK (1<<7)
#define BUTTON_X_MASK (1<<6)
#define EXTI_PR_OFFSET 0x14
#define EXTI_ADDR 0x40010400UL
#define COLOR_ORANGE 'O'
#define COLOR_GREEN 'G'
#define COLOR_BLUE 'B'
#define SERVO_PWM_PERIOD_US 20000
#define ON_TARGET 12
#define CAMERA_FOV 65
#define CAMERA_MID 128

#define MOTOR_STEP_SIZE 16 //16th steps
#define MOTOR_FULL_ROTATION_STEPS 200*MOTOR_STEP_SIZE

#define DPAD_STEPS 50

#define RS_Pin GPIO_PIN_13
#define RS_GPIO_Port GPIOE
#define EN_Pin GPIO_PIN_15
#define EN_GPIO_Port GPIOE
#define D4_Pin GPIO_PIN_14
#define D4_GPIO_Port GPIOE
#define D5_Pin GPIO_PIN_12
#define D5_GPIO_Port GPIOE
#define D6_Pin GPIO_PIN_10
#define D6_GPIO_Port GPIOE
#define D7_Pin GPIO_PIN_7
#define D7_GPIO_Port GPIOE
/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define PS2_SCK_Pin GPIO_PIN_5
#define PS2_SCK_GPIO_Port GPIOA
#define PS2_MISO_Pin GPIO_PIN_6
#define PS2_MISO_GPIO_Port GPIOA
#define PS2_MOSI_Pin GPIO_PIN_7
#define PS2_MOSI_GPIO_Port GPIOA
#define Stepper_Dir_Pin GPIO_PIN_12
#define Stepper_Dir_GPIO_Port GPIOF
#define Stepper_Step_Pin GPIO_PIN_13
#define Stepper_Step_GPIO_Port GPIOF
#define PS2_CS_Pin GPIO_PIN_14
#define PS2_CS_GPIO_Port GPIOD
#define servo_pwm_Pin GPIO_PIN_7
#define servo_pwm_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */
void set_tim4_ccr2(uint16_t val);

// arg should be from 0 to 90
void set_pitch(int degrees_from_level);

// use the STEP_LEFT or STEP_RIGHT macros
void motor_take_step(int dir);

void ps2_transaction(void);

void LCD_SendCommand(uint8_t command);
void LCD_SendData(uint8_t data);
void LCD_Clear(void);
void LCD_WriteString(char* str);
void LCD_Init(void);

int calculate_rotation(int x, int* dir);
int calculate_pitch_change(int y);
void aim_at_coords(int x, int y);
void start_pwm_N_steps(uint32_t N);

extern SPI_HandleTypeDef hspi1;
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
