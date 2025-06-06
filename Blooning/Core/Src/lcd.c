/*
 * lcd.c
 *
 *  Created on: Apr 4, 2025
 *      Author: Andre
 */

#include "main.h"
#include "stdio.h"
#include "string.h"
#include "stepper_pwm.h"

// NOTE: Anode Pin must be connected to Power rail, Sweeper of the Potentiometer has to be connected to Vo.

void LCD_SendCommand4(uint8_t command) {
 HAL_GPIO_WritePin(RS_GPIO_Port, RS_Pin, GPIO_PIN_RESET);
 HAL_Delay(2);
 HAL_GPIO_WritePin(D4_GPIO_Port, D4_Pin, command & 1);
 HAL_GPIO_WritePin(D5_GPIO_Port, D5_Pin, (command >> 1) & 1);
 HAL_GPIO_WritePin(D6_GPIO_Port, D6_Pin, (command >> 2) & 1);
 HAL_GPIO_WritePin(D7_GPIO_Port, D7_Pin, (command >> 3) & 1);
 HAL_GPIO_WritePin(EN_GPIO_Port, EN_Pin, GPIO_PIN_SET);
 HAL_Delay(2);
 HAL_GPIO_WritePin(EN_GPIO_Port, EN_Pin, GPIO_PIN_RESET);
 HAL_Delay(10);
}
void LCD_Init(void) {
 HAL_Delay(100);
 LCD_SendCommand4(0x3);
 LCD_SendCommand4(0x3);
 LCD_SendCommand4(0x3);
 LCD_SendCommand4(0x2);
 LCD_SendCommand(0x2C);
 LCD_SendCommand(0x0F);
 LCD_SendCommand(0x01);
 LCD_SendCommand(0x06);
 LCD_SendCommand(0x0C);
}
void LCD_SendCommand(uint8_t command) {
 HAL_GPIO_WritePin(RS_GPIO_Port, RS_Pin, GPIO_PIN_RESET);
 HAL_Delay(2);
 HAL_GPIO_WritePin(D4_GPIO_Port, D4_Pin, (command >> 4) & 1);
 HAL_GPIO_WritePin(D5_GPIO_Port, D5_Pin, (command >> 5) & 1);
 HAL_GPIO_WritePin(D6_GPIO_Port, D6_Pin, (command >> 6) & 1);
 HAL_GPIO_WritePin(D7_GPIO_Port, D7_Pin, (command >> 7) & 1);
 HAL_GPIO_WritePin(EN_GPIO_Port, EN_Pin, GPIO_PIN_SET);
 HAL_Delay(2);
 HAL_GPIO_WritePin(EN_GPIO_Port, EN_Pin, GPIO_PIN_RESET);
 HAL_Delay(2);
 HAL_GPIO_WritePin(D4_GPIO_Port, D4_Pin, command & 1);
 HAL_GPIO_WritePin(D5_GPIO_Port, D5_Pin, (command >> 1) & 1);
 HAL_GPIO_WritePin(D6_GPIO_Port, D6_Pin, (command >> 2) & 1);
 HAL_GPIO_WritePin(D7_GPIO_Port, D7_Pin, (command >> 3) & 1);
 HAL_GPIO_WritePin(EN_GPIO_Port, EN_Pin, GPIO_PIN_SET);
 HAL_Delay(2);
 HAL_GPIO_WritePin(EN_GPIO_Port, EN_Pin, GPIO_PIN_RESET);
 HAL_Delay(30);
}
void LCD_SendData(uint8_t data) {
 HAL_GPIO_WritePin(RS_GPIO_Port, RS_Pin, GPIO_PIN_SET);
 HAL_Delay(2);
 HAL_GPIO_WritePin(D4_GPIO_Port, D4_Pin, (data >> 4) & 1);
 HAL_GPIO_WritePin(D5_GPIO_Port, D5_Pin, (data >> 5) & 1);
 HAL_GPIO_WritePin(D6_GPIO_Port, D6_Pin, (data >> 6) & 1);
 HAL_GPIO_WritePin(D7_GPIO_Port, D7_Pin, (data >> 7) & 1);
 HAL_GPIO_WritePin(EN_GPIO_Port, EN_Pin, GPIO_PIN_SET);
 HAL_Delay(2);
 HAL_GPIO_WritePin(EN_GPIO_Port, EN_Pin, GPIO_PIN_RESET);
 HAL_Delay(2);
 HAL_GPIO_WritePin(D4_GPIO_Port, D4_Pin, data & 1);
 HAL_GPIO_WritePin(D5_GPIO_Port, D5_Pin, (data >> 1) & 1);
 HAL_GPIO_WritePin(D6_GPIO_Port, D6_Pin, (data >> 2) & 1);
 HAL_GPIO_WritePin(D7_GPIO_Port, D7_Pin, (data >> 3) & 1);
 HAL_GPIO_WritePin(EN_GPIO_Port, EN_Pin, GPIO_PIN_SET);
 HAL_Delay(2);
 HAL_GPIO_WritePin(EN_GPIO_Port, EN_Pin, GPIO_PIN_RESET);
 HAL_Delay(2);
}
void LCD_Clear(void) {
LCD_SendCommand(0x01);
HAL_Delay(2);
}

void LCD_WriteLines(char* line1, char* line2){
	//LCD_Init();
	LCD_Clear();
	//LCD_SendCommand(0b10000000);
	LCD_WriteString(line1);
	LCD_SendCommand(0b11000000);
	LCD_WriteString(line2);
}
void LCD_WriteString(char* str) {
	while(*str) {
		LCD_SendData(*str++);
		HAL_Delay(2);
	}
}
