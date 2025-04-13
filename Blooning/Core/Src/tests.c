/*
 * tests.c
 *
 *  Created on: Apr 4, 2025
 *      Author: Andre
 */

#include "main.h"
#include "stdio.h"
#include "string.h"
#include "stepper_pwm.h"

//use the STEP_CW or STEP_CCW macros
//this function is deprecated as we no longer use gpio for the stepper, instead we use pwm
void motor_take_step(int dir){
	HAL_GPIO_WritePin(Stepper_Dir_GPIO_Port,Stepper_Dir_Pin,dir); //set the direction of the step
	HAL_GPIO_WritePin(Stepper_Step_GPIO_Port,Stepper_Step_Pin,GPIO_PIN_SET);
	HAL_GPIO_WritePin(Stepper_Step_GPIO_Port,Stepper_Step_Pin,GPIO_PIN_RESET);
}

//if the stepper motor seems shaky just put some weight on it
void spin_motor_test(void){
	HAL_GPIO_WritePin(Stepper_Dir_GPIO_Port,Stepper_Dir_Pin,STEP_CW); //set the direction of the step
	start_pwm_N_steps(200);
	HAL_Delay(500);
	HAL_GPIO_WritePin(Stepper_Dir_GPIO_Port,Stepper_Dir_Pin,STEP_CCW); //set the direction of the step
	start_pwm_N_steps(200);
	HAL_Delay(1500);
}

//ps2 transaction from class presentation
//for reference see https://docs.google.com/presentation/d/12RinNV7N_wY5BfJECBLrAtkQDGO9stQqThNh4kqjwPg/edit#slide=id.g3420c6d2660_0_211
void ps2_transaction(void){
	static uint8_t PSX_RX[21];
	static uint8_t PSX_TX[21] = {
	     0x01, 0x42, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	     0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	     0x00, 0x00, 0x00, 0x00, 0x00
	 };
   // Get state of all buttons and store it in PSX_RX (Transmission length 21)
   HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET); // CS LOW
   HAL_SPI_TransmitReceive(&hspi1, PSX_TX, PSX_RX, 21, 10);
   HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET); // CS HIGH

   // get state of d-pad and X button
   int dpad_status = ~PSX_RX[3];
   int dpad_up = dpad_status & DPAD_UP_MASK;
   int dpad_down = dpad_status & DPAD_DOWN_MASK;
   int dpad_left = dpad_status & DPAD_LEFT_MASK;
   int dpad_right = dpad_status & DPAD_RIGHT_MASK;

   int button_status = ~PSX_RX[4];
   int button_x = button_status & BUTTON_X_MASK;
   //note that they will not be 1 or 0, they will be 0 or positive int
   //TODO: do something with this data
   if(dpad_up){
	   printf("UP IS PRESSED \r\n");
   }
   if(dpad_down){
   	   printf("DOWN IS PRESSED \r\n");
   }
   if(dpad_left){
   	   printf("LEFT IS PRESSED \r\n");
   }
   if(dpad_right){
   	   printf("RIGHT IS PRESSED \r\n");
   }
   if(button_x){
   	   printf("X IS PRESSED \r\n");
   }
}


void calculate_coords_test(void){
	printf("Testing calculate_rotation(x,dir_addr) \n\r");
	for(int i = 0; i < 256; i++){
	  int dir;
	  int n_steps = calculate_rotation(i,&dir);
	  if(dir == STEP_CW){
		  printf("x = %d, dir = STEP_CW, n_steps = %d \n\r",i,n_steps);
	  }
	  else if(dir == STEP_CCW){
		  printf("x = %d, dir = STEP_CCW, n_steps = %d \n\r",i,n_steps);
	  }
	  else{
		  printf("!!DIR EXPLODED!! x = %d, dir = %d, n_steps = %d \n\r",i,dir,n_steps);
	  }
	}
	HAL_Delay(10000);
}
