/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "string.h"
#include "stepper_pwm.h"
#include "stdlib.h"
#include <stdlib.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef hlpuart1;
UART_HandleTypeDef huart1;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim15;
TIM_HandleTypeDef htim16;

NOR_HandleTypeDef hnor1;

/* USER CODE BEGIN PV */
#pragma pack(1) //do not pad inside struct
struct coord{
	char color;
	uint8_t x;
	uint8_t y;
};
#pragma pack() //restore default packing

int coord_cnt;

int change_mode = 1; //Track if the mode is changed

struct coord coord_list[255];
enum TurretMode {
	AUTO_RBG, //slowest to fastest (IN GAME)
	AUTO_GBR, // fastest to slowest (IN_GAME)
	MANUAL, // use ps2 controller
	RELOAD
};

enum color {
	RED=3,GREEN=2,BLUE=1
};

int cartridge_state = 0;


enum TurretMode current_mode = MANUAL;
enum TurretMode mode_before_reload;

uint8_t stepper_active = 0; //0 = stepper pwm is off, 1 = stepper pwm is on
int current_pitch = PITCH_REST;
int enter_reload_from_auto = 0;

char RBG_Prios [3] = {'R','B','G'};
char GBR_Prios [3] = {'G','B','R'};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_LPUART1_UART_Init(void);
static void MX_TIM4_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM15_Init(void);
static void MX_TIM16_Init(void);
static void MX_FMC_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

//This is a helper function, if you want to mess with the servo, use the set_pitch function
void set_tim4_ccr2(uint16_t val){
	uint32_t * tim4_ccr2 = (uint32_t *)(TIM4_ADDR + TIM_CCR2_OFFSET);
	*tim4_ccr2 &= ~CCR_MASK;
	*tim4_ccr2 |= (uint16_t) val;
}

// arg should be from 0 to 90
void set_pitch(int degrees_from_level){
        // servo is on unsidedown
        // writing it like this makes my sanity exist
        degrees_from_level = 180 - degrees_from_level;
	set_tim4_ccr2(degrees_from_level);
}

// this function is to control the angle on the servo that controls the trigger
//degrees should be 0 to 180
void set_trigger_angle(int degrees){
	int val = degrees*(SERVO_MAX-SERVO_MIN)/180 + SERVO_MIN;
	TIM15->CCR1 = val;
}

// this function is to control the angle on the servo that controls the cartridge
//degrees should be 0 to 180
void set_cartridge_angle(int degrees){
	int val = degrees*(SERVO_MAX-SERVO_MIN)/180 + SERVO_MIN;
	TIM16->CCR1 = val;
}

void read_coords(){
	unsigned char *tx_buf = "request\n";
	while(stepper_active){
		continue;
	}
	// after we are done moving give the RPI time to recompute
	HAL_UART_Transmit(&huart1, tx_buf, 8, 1000);
	uint8_t cnt_rx;
	HAL_UART_Receive(&huart1, &cnt_rx, 1, 1000);
	coord_cnt = cnt_rx;
	if(coord_cnt > 20)
		coord_cnt = 20;
	HAL_UART_Receive(&huart1, coord_list, coord_cnt*3, 500); // color, x, y

}

//INTERRUPT HANDLER
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
    if(GPIO_Pin == GPIO_PIN_13){
    	reset_prios();
        //USER BUTTON WAS PRESSED
        int speed = 50000; //button is debouncing :)
        for(int i = 0; i < speed; i++){
            continue; //can't use HAL_DELAY since SYS_Tick interrupt priority is low
        }
        if(current_mode == RELOAD){
            current_mode = mode_before_reload;
            change_mode = 1;
        }
        else{
            current_mode = (current_mode + 1) % 3; //cycle modes
            change_mode = 1; //Mode was changed so LCD needs to be updated
        }
        uint32_t *p = EXTI_ADDR 	+ EXTI_PR_OFFSET;
        *p |= (1<<13); //clear interrupt pending
    }
}

/*
 * takes x and y between 0 and 255
 */
void draw_circle(uint8_t x, uint8_t y, enum color c){
	uint16_t scaled_x = ((uint64_t)x*5)>>1;
	uint16_t scaled_y = ((uint64_t)y*15)>>3;

	*((uint8_t *)VGA_COLOR_ADDR) = c;
	*((uint8_t *)VGA_X_UPPER_ADDR) = (uint8_t)(scaled_x & 0xFF);
	*((uint8_t *)VGA_X_LOWER_ADDR) = (uint8_t)(scaled_x >> 8);
	*((uint8_t *)VGA_Y_UPPER_ADDR) = (uint8_t)(scaled_y & 0xFF);
	*((uint8_t *)VGA_Y_LOWER_ADDR) = (uint8_t)(scaled_y >> 8);
}

void manual_control(void){
	//TODO: make this a function lcd stuff
	//static int displayed = 1;
	if (change_mode){
		lcd_display();
	}

	printf("MODE: MANUAL \n\r");
	static uint8_t PSX_RX[21];
	static uint8_t PSX_TX[21] = {
		 0x01, 0x42, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		 0x00, 0x00, 0x00, 0x00, 0x00
	 };
   // Get state of all buttons and store it in PSX_RX (Transmission length 21)
   HAL_GPIO_WritePin(PS2_CS_GPIO_Port, PS2_CS_Pin, GPIO_PIN_RESET); // CS LOW
   HAL_SPI_TransmitReceive(&hspi1, PSX_TX, PSX_RX, 21, 10);
   HAL_GPIO_WritePin(PS2_CS_GPIO_Port, PS2_CS_Pin, GPIO_PIN_SET); // CS HIGH

   // get state of d-pad and X button
   int dpad_status = ~PSX_RX[3];
   int dpad_up = dpad_status & DPAD_UP_MASK;
   int dpad_down = dpad_status & DPAD_DOWN_MASK;
   int dpad_left = dpad_status & DPAD_LEFT_MASK;
   int dpad_right = dpad_status & DPAD_RIGHT_MASK;

   int button_status = ~PSX_RX[4];
   int button_x = button_status & BUTTON_X_MASK;
   int button_y = button_status & BUTTON_Y_MASK;

   //note that they will not be 1 or 0, they will be 0 or positive int
   if(dpad_up){
	   if(current_pitch < PITCH_MAX){
		   current_pitch+=1;
		   set_pitch(current_pitch);
		   HAL_Delay(10);
	   }

   }
   if(dpad_down){
	   if(current_pitch > PITCH_MIN){
		   current_pitch-=1;
		   set_pitch(current_pitch);
		   HAL_Delay(10);
	   }

   }
   if(dpad_left){
	   HAL_GPIO_WritePin(Stepper_Dir_GPIO_Port,Stepper_Dir_Pin,STEP_CCW); //set the direction of the step
	   start_pwm_N_steps(DPAD_STEPS);
   }
   if(dpad_right){
	   HAL_GPIO_WritePin(Stepper_Dir_GPIO_Port,Stepper_Dir_Pin,STEP_CW); //set the direction of the step
	   start_pwm_N_steps(DPAD_STEPS);
   }

   if(button_x){
	   shoot();
	   HAL_Delay(400);
   }
    if (button_y) {
        reload();
        HAL_Delay(400);
    }

}

// Function to start pwm of stepper motor for N steps
// see TIM2_IRQHandler for interrupt handler related to this functionality
void start_pwm_N_steps(uint32_t N){
	if(N == 0){
		return; //this would explode
	}
	//stop pwm
	HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_2);
	HAL_TIM_Base_Stop(&htim2);
	__HAL_TIM_SetCounter(&htim2, 0);


    // clear any pending interrupts
    __HAL_TIM_DISABLE_IT(&htim2, TIM_IT_UPDATE);
    __HAL_TIM_CLEAR_FLAG(&htim2, TIM_SR_UIF);

	//configure tim2 to count N periods
	htim2.Instance = TIM2;
	htim2.Init.Prescaler = 0;
	htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim2.Init.Period = N;  // Interrupt after N+1 periods
	htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
	if(HAL_TIM_Base_Init(&htim2) != HAL_OK){
		Error_Handler(); //debug
	}


	// enable tim2 interrupt that will turn pwm off
	HAL_NVIC_SetPriority(TIM2_IRQn, 0, 0);
	__HAL_TIM_ENABLE_IT(&htim2, TIM_IT_UPDATE);
	HAL_NVIC_EnableIRQ(TIM2_IRQn);

	// start stepper pwm
	stepper_active = 1;
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
	HAL_TIM_Base_Start(&htim2);

	//debug force update
	//TIM2->EGR = TIM_EGR_UG;
}

// Use macro MOTOR_FULL_ROTATION_STEPS
int calculate_rotation(int x, int* dir){
	if(x < CAMERA_MID){
		*dir = STEP_CW;
	}
	else{
		*dir = STEP_CCW;
	}
	int offset = abs(x-CAMERA_MID);
	if(offset < ON_TARGET){
		return offset;
	}

	float degrees_to_step = 45/(1 + pow(2.7182, (-0.1*(offset-54))));
	int num_steps = MOTOR_FULL_ROTATION_STEPS*degrees_to_step/360;
	return num_steps/3;
}

//should be negative is appropriate
int calculate_pitch_change(int y){
    int diff = (CAMERA_MID - y);
    float scale = 0.08*(PITCH_MAX-PITCH_MIN)/(1+pow(2.7182, -0.05*(abs(diff)-39)));
    int res = 0;
    if (diff > 5) {
        // Move up
        res = current_pitch >= PITCH_MAX ? 0 : (int)scale;
    }
    else if(diff < -5){
        // Move down
        res = current_pitch <= PITCH_MIN ? 0 : -(int)(0.8*scale);
    }
    return res;
}

void aim_at_coords(int x, int y, enum color c){
	//calculate rotation

	draw_circle(x, y, c);

    static int locked = 0;
    if (abs(x - CAMERA_MID) <= H_SHOOT_RAD && abs(y - CAMERA_MID) <= V_SHOOT_RAD ) {
        locked++;
        if (locked >4) {
            shoot();
            locked = 0;
        }
        return;
    }
    locked = 0;

	int dir;
	int n_steps = calculate_rotation(x,&dir);
	printf("nsteps = %d \n\r",n_steps);
	current_pitch += calculate_pitch_change(y);

	//aim
	HAL_GPIO_WritePin(Stepper_Dir_GPIO_Port,Stepper_Dir_Pin,dir); //set the direction of the step
	if(n_steps){
		start_pwm_N_steps(n_steps);
	}
	set_pitch(current_pitch);
}

int color_lock(char c){
	//if color lock then return
	static char prev_c = 0;
	static int c_count = 0;

	if (c != prev_c){
		c_count++;
		if (c_count == 5){
			c_count = 0;
			prev_c = c;
			return 1;
		}
		return 0;
	}
	return 1;
}

void automatic_mode(){
	if(change_mode){
		lcd_display();
	}

	read_coords();
	printf("N coords received = %d \n\r", coord_cnt);
	for(int prio = 0; prio < sizeof(RBG_Prios); prio++){
		for(int i = 0; i < coord_cnt; i++){
			printf("color = %c, x = %d, y = %d \n\r ",coord_list[i].color,coord_list[i].x,coord_list[i].y);
			switch (current_mode){
			case AUTO_GBR:
				if(coord_list[i].color == GBR_Prios[prio]){
					printf("aiming\n\r");
					if(!color_lock(coord_list[i].color)) goto done_aiming;
					aim_at_coords(coord_list[i].x + CAMERA_X_OFFSET,coord_list[i].y, coord_list[i].color);
					goto done_aiming;
				}
				break;
			case AUTO_RBG:
				if(coord_list[i].color == RBG_Prios[prio]){
					printf("aiming\n\r");
					if(!color_lock(coord_list[i].color)) goto done_aiming;
					aim_at_coords(coord_list[i].x + CAMERA_X_OFFSET,coord_list[i].y, coord_list[i].color);
					goto done_aiming;
				}
				break;
			default:
				printf("turret mode error\n\r");
				return;
			}

		}
	}
done_aiming:
	static int zeros_seen = 0;
	if(coord_cnt == 0){ // no blooners on camera, reset priorities
		zeros_seen++;
		if (zeros_seen == 10){
			reset_prios();
			zeros_seen = 0;
		}
	}
	else{
		zeros_seen = 0;
	}
	coord_cnt = 0;
	memset(coord_list, 0, sizeof(coord_list));
	HAL_Delay(200);
}

void reload(void){
    mode_before_reload = current_mode;
    current_mode = RELOAD;
    set_cartridge_angle(CARTRIDGE_START);
    int bands = MAX_RUBBER_BANDS - cartridge_state;
    char l2[100];
    snprintf(l2, sizeof(l2), "Bands Left: %d", bands);
    LCD_WriteLines("Mode: RELOAD", l2);
    cartridge_state = 0;
    reset_prios();
    return;
}

void shoot(void){
	//Shoot
	set_trigger_angle(TRIGGER_SHOOT);
	HAL_Delay(1100);
	set_trigger_angle(TRIGGER_REST);
	HAL_Delay(1100);

	//Get next rubber band in position
	cartridge_state++;
	if(cartridge_state == MAX_RUBBER_BANDS){ //if out of bands, reload
            reload();
            return;
	}
	//TODO make this a function
	lcd_display();

	set_cartridge_angle(CARTRIDGE_ANGLE*cartridge_state+CARTRIDGE_OFFSET);
	rotate_prios();
	return;
}

void reload_done_check(void){
	static uint8_t PSX_RX[21];
	static uint8_t PSX_TX[21] = {
		 0x01, 0x42, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		 0x00, 0x00, 0x00, 0x00, 0x00
	 };
     // Get state of all buttons and store it in PSX_RX (Transmission length 21)
     HAL_GPIO_WritePin(PS2_CS_GPIO_Port, PS2_CS_Pin, GPIO_PIN_RESET); // CS LOW
     HAL_SPI_TransmitReceive(&hspi1, PSX_TX, PSX_RX, 21, 10);
     HAL_GPIO_WritePin(PS2_CS_GPIO_Port, PS2_CS_Pin, GPIO_PIN_SET); // CS HIGH

     int button_status = ~PSX_RX[4];
     int button_y = button_status & BUTTON_Y_MASK;

	 if(button_y){
		HAL_Delay(200);
		current_mode = mode_before_reload;
		change_mode = 1;
	 }
	 return;
}

void lcd_display(void){
	if (current_mode == MANUAL){ //UPDATING RUBBERBAND COUNTS
		int bands = MAX_RUBBER_BANDS - cartridge_state;
		char l2[100];
		snprintf(l2, sizeof(l2), "Bands Left: %d", bands);
		LCD_WriteLines("Mode: MANUAL", l2);
	}
	if(current_mode == AUTO_GBR){
		int bands = MAX_RUBBER_BANDS - cartridge_state;
		char l2[100];
		snprintf(l2, sizeof(l2), "Bands Left: %d", bands);
		LCD_WriteLines("Mode: GBR", l2);
	}
	if(current_mode == AUTO_RBG){
		int bands = MAX_RUBBER_BANDS - cartridge_state;
		char l2[100];
		snprintf(l2, sizeof(l2), "Bands Left: %d", bands);
		LCD_WriteLines("Mode: RBG", l2);
	}
	change_mode = 0;
}

void reset_prios(void){
	GBR_Prios[0] = 'G';
	GBR_Prios[1] = 'B';
	GBR_Prios[2] = 'R';
	RBG_Prios[0] = 'R';
	RBG_Prios[1] = 'B';
	RBG_Prios[2] = 'G';
}

void rotate_prios(void){
	char tmp = GBR_Prios[0];
	GBR_Prios[0] = GBR_Prios[1];
	GBR_Prios[1] = GBR_Prios[2];
	GBR_Prios[2] = tmp;

	tmp = RBG_Prios[0];
	RBG_Prios[0] = RBG_Prios[1];
	RBG_Prios[1] = RBG_Prios[2];
	RBG_Prios[2] = tmp;
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_LPUART1_UART_Init();
  MX_TIM4_Init();
  MX_SPI1_Init();
  MX_USART1_UART_Init();
  MX_TIM3_Init();
  MX_TIM2_Init();
  MX_TIM15_Init();
  MX_TIM16_Init();
  MX_FMC_Init();
  /* USER CODE BEGIN 2 */
  LCD_Init();
  {
  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  htim3.Init.Prescaler = STEPPER_PRESCALER;
  htim3.Init.Period = STEPPER_SPEED;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
	  Error_Handler();
  }
  	  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
      Error_Handler();
  }
  }
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2); //servo pwm
  HAL_TIM_PWM_Start(&htim15, TIM_CHANNEL_1); //trigger pwm
  HAL_TIM_PWM_Start(&htim16, TIM_CHANNEL_1); //cartridge pwm
  //initialize the servo with to be level with ground
  set_pitch(PITCH_REST);
  set_trigger_angle(TRIGGER_REST);
  set_cartridge_angle(CARTRIDGE_START);
  //set_cartridge_angle(5);
  while (1)
  {
	  if(current_mode == MANUAL){
		  manual_control();
	  }
	  else if (current_mode == RELOAD){
		  reload_done_check();
	  }
	  else if (current_mode == AUTO_GBR || current_mode == AUTO_RBG){
		  automatic_mode();
	  }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief LPUART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_LPUART1_UART_Init(void)
{

  /* USER CODE BEGIN LPUART1_Init 0 */

  /* USER CODE END LPUART1_Init 0 */

  /* USER CODE BEGIN LPUART1_Init 1 */

  /* USER CODE END LPUART1_Init 1 */
  hlpuart1.Instance = LPUART1;
  hlpuart1.Init.BaudRate = 115200;
  hlpuart1.Init.WordLength = UART_WORDLENGTH_8B;
  hlpuart1.Init.StopBits = UART_STOPBITS_1;
  hlpuart1.Init.Parity = UART_PARITY_NONE;
  hlpuart1.Init.Mode = UART_MODE_TX_RX;
  hlpuart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  hlpuart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  hlpuart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  hlpuart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  hlpuart1.FifoMode = UART_FIFOMODE_DISABLE;
  if (HAL_UART_Init(&hlpuart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&hlpuart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&hlpuart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&hlpuart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN LPUART1_Init 2 */

  /* USER CODE END LPUART1_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_128;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_LSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_SlaveConfigTypeDef sSlaveConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4294967295;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sSlaveConfig.SlaveMode = TIM_SLAVEMODE_EXTERNAL1;
  sSlaveConfig.InputTrigger = TIM_TS_ITR2;
  if (HAL_TIM_SlaveConfigSynchro(&htim2, &sSlaveConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 3999;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 1000;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 39;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 1999;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

}

/**
  * @brief TIM15 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM15_Init(void)
{

  /* USER CODE BEGIN TIM15_Init 0 */

  /* USER CODE END TIM15_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM15_Init 1 */

  /* USER CODE END TIM15_Init 1 */
  htim15.Instance = TIM15;
  htim15.Init.Prescaler = 39;
  htim15.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim15.Init.Period = 1999;
  htim15.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim15.Init.RepetitionCounter = 0;
  htim15.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim15) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim15, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim15) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim15, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim15, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim15, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM15_Init 2 */

  /* USER CODE END TIM15_Init 2 */
  HAL_TIM_MspPostInit(&htim15);

}

/**
  * @brief TIM16 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM16_Init(void)
{

  /* USER CODE BEGIN TIM16_Init 0 */

  /* USER CODE END TIM16_Init 0 */

  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM16_Init 1 */

  /* USER CODE END TIM16_Init 1 */
  htim16.Instance = TIM16;
  htim16.Init.Prescaler = 39;
  htim16.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim16.Init.Period = 1999;
  htim16.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim16.Init.RepetitionCounter = 0;
  htim16.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim16) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim16) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim16, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim16, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM16_Init 2 */

  /* USER CODE END TIM16_Init 2 */
  HAL_TIM_MspPostInit(&htim16);

}

/* FMC initialization function */
static void MX_FMC_Init(void)
{

  /* USER CODE BEGIN FMC_Init 0 */

  /* USER CODE END FMC_Init 0 */

  FMC_NORSRAM_TimingTypeDef Timing = {0};

  /* USER CODE BEGIN FMC_Init 1 */

  /* USER CODE END FMC_Init 1 */

  /** Perform the NOR1 memory initialization sequence
  */
  hnor1.Instance = FMC_NORSRAM_DEVICE;
  hnor1.Extended = FMC_NORSRAM_EXTENDED_DEVICE;
  /* hnor1.Init */
  hnor1.Init.NSBank = FMC_NORSRAM_BANK1;
  hnor1.Init.DataAddressMux = FMC_DATA_ADDRESS_MUX_DISABLE;
  hnor1.Init.MemoryType = FMC_MEMORY_TYPE_NOR;
  hnor1.Init.MemoryDataWidth = FMC_NORSRAM_MEM_BUS_WIDTH_8;
  hnor1.Init.BurstAccessMode = FMC_BURST_ACCESS_MODE_DISABLE;
  hnor1.Init.WaitSignalPolarity = FMC_WAIT_SIGNAL_POLARITY_LOW;
  hnor1.Init.WaitSignalActive = FMC_WAIT_TIMING_BEFORE_WS;
  hnor1.Init.WriteOperation = FMC_WRITE_OPERATION_ENABLE;
  hnor1.Init.WaitSignal = FMC_WAIT_SIGNAL_DISABLE;
  hnor1.Init.ExtendedMode = FMC_EXTENDED_MODE_DISABLE;
  hnor1.Init.AsynchronousWait = FMC_ASYNCHRONOUS_WAIT_ENABLE;
  hnor1.Init.WriteBurst = FMC_WRITE_BURST_DISABLE;
  hnor1.Init.ContinuousClock = FMC_CONTINUOUS_CLOCK_SYNC_ONLY;
  hnor1.Init.WriteFifo = FMC_WRITE_FIFO_ENABLE;
  hnor1.Init.NBLSetupTime = 0;
  hnor1.Init.PageSize = FMC_PAGE_SIZE_NONE;
  /* Timing */
  Timing.AddressSetupTime = 15;
  Timing.AddressHoldTime = 15;
  Timing.DataSetupTime = 255;
  Timing.DataHoldTime = 0;
  Timing.BusTurnAroundDuration = 15;
  Timing.CLKDivision = 16;
  Timing.DataLatency = 17;
  Timing.AccessMode = FMC_ACCESS_MODE_A;
  /* ExtTiming */

  if (HAL_NOR_Init(&hnor1, &Timing, NULL) != HAL_OK)
  {
    Error_Handler( );
  }

  /* USER CODE BEGIN FMC_Init 2 */

  /* USER CODE END FMC_Init 2 */
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */
  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  HAL_PWREx_EnableVddIO2();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_6|GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_13
                          |GPIO_PIN_14|GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOF, Stepper_Dir_Pin|Stepper_Step_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(PS2_CS_GPIO_Port, PS2_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : PE6 PE11 PE12 PE13
                           PE14 */
  GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_13
                          |GPIO_PIN_14;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : Stepper_Dir_Pin Stepper_Step_Pin */
  GPIO_InitStruct.Pin = Stepper_Dir_Pin|Stepper_Step_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pin : PE15 */
  GPIO_InitStruct.Pin = GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : PS2_CS_Pin */
  GPIO_InitStruct.Pin = PS2_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(PS2_CS_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
  #define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */
PUTCHAR_PROTOTYPE
{
  HAL_UART_Transmit(&hlpuart1, (uint8_t *)&ch, 1, 0xFFFF);
  return ch;
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
