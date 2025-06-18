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
#include "LCD_I2C.h"
#include "AS5600_Driver.h"
#include "RS232_Driver.h"
#include "ArmRobot_Math.h"
#include "DWT_Delay.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
Data_Get_t main_command;
Data_Get_t pendant_command;
Kinematics_t kinematics;

typedef enum{
	LINEAR_INTERPOLATION = 0x01,
	SPLINE_INTERPOLATION,
}Interpolation_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

//#define USE_BUTTON
//#define USE_MANUAL_ENCODER

#define TEST_STEPPER
//#define TEST_ENCODER
//#define TEST_COM
//#define TEST_KINEMATICS
//#define TEST_MOTION

#define JOINT_NUM		3
#define AXIS_NUM		3

#define MICROSTEP_VALUE1 			200
#define MICROSTEP_VALUE2 			400
#define MICROSTEP_VALUE3 			800
#define MICROSTEP_VALUE4 			1600
#define MICROSTEP_VALUE5			3200
#define MICROSTEP_VALUE6 			6400  

#define STEPPER1_RATIO				162.0f
#define	STEPPER2_RATIO				200.0f
#define	STEPPER3_RATIO				50.0f
#define	STEPPER4_RATIO				96.0f 
#define	STEPPER5_RATIO				22.5f
#define	STEPPER6_RATIO				25.0f

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim5;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

// PARAMETER VARIABLE
float
joint_d[6] = {
	191.0, 0.0, 0.0,
	575.0, 0.0, 65.0,
},

joint_a[6] = {
	23.5, 650.0, 0.0,
	0.0, 0.0, 0.0, 
},

joint_alpha[6] = {
	-90.0, 0.0, 90.0,
	-90.0, 90.0, 0.0,
},

move_ang,
delta_move_ang,
move_pos,
delta_move_pos;

uint16_t
welding_point_num,
current_welding_point;


// STEPPER VARIABLE
bool
direction = false,
step_start = false;

TIM_TypeDef*
stepper_tim_instance[JOINT_NUM] = {
	TIM3,
	TIM4,
	TIM5,
};

TIM_HandleTypeDef*
stepper_tim_handler[JOINT_NUM] = {
	&htim3,
	&htim4,
	&htim5,
};

GPIO_TypeDef*
stepper_gpio_port[6] = {
	PUL1_GPIO_Port,
	PUL2_GPIO_Port,
	PUL3_GPIO_Port,
	
	DIR1_GPIO_Port,
	DIR2_GPIO_Port,
	DIR3_GPIO_Port,
};


// RS-232 VARIABLE
bool get_data_flag;
unsigned long get_data_timer;


// ENCODER VARIABLE
bool
btn1_state = false,
btn2_state = false;

uint8_t
enc_mag_status;

uint32_t 
enc_counter,
pwm_freq, 
cycle_time;

float
min_duty_cycle = 2.9,
max_duty_cycle = 97.1,
duty_cycle,
enc_angle,
cal_value,
cal_enc_angle,
prev_cal_enc_angle,
reduced_cal_enc_angle,
enc_joint_angle,
prev_enc_joint_angle;

// Timer
unsigned long
start_timer,
end_timer;

// Custom Encoder
uint8_t dummy_counter;
int current_step, dummy_input_step;
float dummy_joint_angle;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM5_Init(void);
/* USER CODE BEGIN PFP */

void uart_receive(uint8_t *msg_buff, uint16_t len);
void uart_transmit(uint8_t *msg, uint16_t len);
void process_data(uint8_t *data, uint16_t len);

void move_stepper(uint32_t step, uint8_t dir, uint32_t freq);
void move_joint(double input_angle, Speed_t input_speed);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#ifdef USE_BUTTON 
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_PIN){
	if(GPIO_PIN == KEY1_Pin){
		btn1_state = (HAL_GPIO_ReadPin(KEY1_GPIO_Port, KEY1_Pin) == GPIO_PIN_RESET);
		cal_value = enc_angle;
	}
	else if(GPIO_PIN == KEY2_Pin){
		btn2_state = (HAL_GPIO_ReadPin(KEY2_GPIO_Port, KEY2_Pin) == GPIO_PIN_RESET);
	}
}
#endif

#ifdef TEST_STEPPER
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
	for(int i=0; i<JOINT_NUM; i++){
		if(htim->Instance != stepper_tim_instance[i]) continue;
			
		// Cache Variable
		uint16_t cnt = ++timer_counter[i];
		uint16_t t_on_val = t_on[i];
		uint16_t period = step_period[i];
		GPIO_TypeDef *port = stepper_gpio_port[i];
		uint16_t pin = stepper_gpio_pin[i];
		
		// Step Signal
		if(step_counter[i] <= target_step[i] || step_cont_run_j[i] || step_cal_run[i]){
			if(cnt == 1) port->BSRR = (uint32_t)pin;
			else if(cnt == t_on_val) port->BSRR = (uint32_t)pin << 16;
			else if (cnt >= period){
				timer_counter[i] = 0;
				step_counter[i]++;
			}
		}
		
		// Step Reached 
		if(step_counter[i] > target_step[i]){
			if(step_cont_run_w[i]) step_counter[i] = 0;
			step_reached[i] = true;
		}
		
		// Stop Timer 
		if(((step_counter[i] >= target_step[i]) && step_limit[i]) || (!step_limit[i] && !step_cont_run_j[i] && !step_cont_run_w[i] && !step_cal_run[i] && !robot_run) || joint_limit || robot_stop || end_point_reach){		
			port->BSRR = (uint32_t)pin << 16;
			HAL_TIM_Base_Stop_IT(stepper_tim_handler[i]);
			timer_counter[i] = 0;
			step_counter[i] = 0;
			target_step[i] = 0;
			step_state[i] = STOPPING;
		}
	}
}
#endif

#ifdef TEST_ENCODER
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim){
	if(htim->Instance == TIM2){
		cycle_time = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
		if(cycle_time != 0){
			duty_cycle = (HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2) *100.0)/cycle_time;
			pwm_freq = (84000000/cycle_time);
			enc_angle = (((duty_cycle - min_duty_cycle) * 360) / (max_duty_cycle - min_duty_cycle));
			cal_enc_angle = fmod((enc_angle - cal_value + 360), 360);
			reduced_cal_enc_angle = cal_enc_angle / stepper_ratio;
			
			if(prev_cal_enc_angle > 350 && cal_enc_angle < 10) enc_counter++;
			else if(prev_cal_enc_angle < 10 && cal_enc_angle > 350) enc_counter--;
			
			enc_joint_angle = reduced_cal_enc_angle + (enc_counter*(360.0/stepper_ratio));
			
			prev_cal_enc_angle = cal_enc_angle;
		}
	}
}
#endif

#ifdef TEST_COM
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){	
	if(huart->Instance == USART2){
		get_data_timer = HAL_GetTick();
		get_data_flag = true;
	}
}
#endif

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
  MX_I2C1_Init();
  MX_TIM2_Init();
  MX_USART1_UART_Init();
  MX_TIM3_Init();
  MX_USART2_UART_Init();
  MX_TIM4_Init();
  MX_TIM5_Init();
  /* USER CODE BEGIN 2 */
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
	HAL_Delay(1000);
	HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
	
	#ifdef TEST_STEPPER
	HAL_GPIO_WritePin(ENA_GPIO_Port, ENA_Pin, GPIO_PIN_RESET);
	DWT_Delay_Init();
	#endif
	
	#ifdef TEST_ENCODER
	AS5600_Init(&hi2c1, DIGITAL_PWM);
	enc_mag_status = Get_Magnet_Status();
	HAL_Delay(50);
	AS5600_Burn();
	
	HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_1);
	HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_2);
	#endif
	
	#ifdef TEST_COM
	RS232_Init(&main_command, &huart1);
	RS232_Init(&pendant_command, &huart2);
	#endif
	
	#ifdef TEST_KINEMATICS
	DHparam_init(&kinematics, joint_d, joint_a, joint_alpha);
	tollframe_init(&kinematics, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
	forward_transform_matrix(&kinematics);
	calculate_all_link(&kinematics);
	#endif
	
	#ifdef TEST_MOTION
	DHparam_init(&kinematics, joint_d, joint_a, joint_alpha);
	tollframe_init(&kinematics, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
	
	double
	dummy_start_pos[6] = {10, 20, 15, 0, 0, 0},
	dummy_end_pos[6] = {20, 15, 10, 0, 0, 0};
	
	// Move to home position
	
	// Move to first welding point
	
	while(1){
		if(current_welding_point <= welding_point_num){
			
		}
	}
	#endif
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
	{
		#ifdef USE_MANUAL_ENCODER
		if(step_counter != 0){
			if(direction){
				current_step = step_counter;
			}
			else current_step = -step_counter;
		}
		
		dummy_joint_angle = current_step * (0.0125);
		#endif
		
		#ifdef TEST_KINEMATICS
		fk_angle_input[0] = 0.0;
		fk_angle_input[1] = 19.9;
		fk_angle_input[2] = 20.3;
		fk_angle_input[3] = -0.2;
		fk_angle_input[4] = -40.0;
		fk_angle_input[5] = -0.3;
		  
		kinematics.j5_enc_angle = fk_angle_input[4];
		
		run_forward_kinematic(&kinematics, fk_angle_input);
		run_inverse_kinematic(&kinematics, kinematics.axis_pos_out[0], kinematics.axis_pos_out[1], kinematics.axis_pos_out[2], kinematics.axis_rot_out[0], kinematics.axis_rot_out[1], kinematics.axis_rot_out[2]);
		find_jacobian_variable(&kinematics);
		check_singularity(&kinematics);
		#endif
		
		#ifdef TEST_STEPPER
//		move_stepper(160, 0, 5000);
		move_joint(90+dummy_counter, LOW);
		
		if(btn1_state == true){
			step_start = false;
			if(dummy_counter < 10){
				dummy_counter += 10;
			}
			else if(dummy_counter >= 10){
				dummy_counter -= 10;
			}
		}
		#endif 
		
		#ifdef TEST_COM
		float 
		world_pos[3] = {12.3, 12.3, 12.3},
		world_rot[3] = {12.3, 12.3, 12.3},
		joint_angle[6] = {90.5, 90.5, 90.5, 90.5, 90.5, 90.5};
		
		if(btn1_state == true){
			for(int i=0; i<50; i++){
				Send_requested_data(&main_command, world_pos, world_rot, joint_angle, 0x01, LINEAR, HIGH);
			}
			HAL_Delay(1000);
		}
		
		Start_get_command(&pendant_command);
		Get_command(&pendant_command);
		#endif
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
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Enables the Clock Security System
  */
  HAL_RCC_EnableCSS();
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 400000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_SlaveConfigTypeDef sSlaveConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4294967295;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sSlaveConfig.SlaveMode = TIM_SLAVEMODE_RESET;
  sSlaveConfig.InputTrigger = TIM_TS_TI1FP1;
  sSlaveConfig.TriggerPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sSlaveConfig.TriggerPrescaler = TIM_ICPSC_DIV1;
  sSlaveConfig.TriggerFilter = 0;
  if (HAL_TIM_SlaveConfigSynchro(&htim2, &sSlaveConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_FALLING;
  sConfigIC.ICSelection = TIM_ICSELECTION_INDIRECTTI;
  if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_2) != HAL_OK)
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

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 839;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

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

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 839;
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
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief TIM5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM5_Init(void)
{

  /* USER CODE BEGIN TIM5_Init 0 */

  /* USER CODE END TIM5_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 0;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 839;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim5, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */

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
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, DIR_Pin|PUL_Pin|ENA_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LED_Pin */
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : KEY1_Pin KEY2_Pin */
  GPIO_InitStruct.Pin = KEY1_Pin|KEY2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : DIR_Pin ENA_Pin */
  GPIO_InitStruct.Pin = DIR_Pin|ENA_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PUL_Pin */
  GPIO_InitStruct.Pin = PUL_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(PUL_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 14, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI1_IRQn, 14, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/*  UART RECEIVE FUNCTION */
void uart_receive(uint8_t *msg_buff, uint16_t len){
	HAL_UART_Receive_IT(&huart1, msg_buff, len);
}

/*  UART TRANSMIT FUNCTION */
void uart_transmit(uint8_t *msg, uint16_t len){
	if(btn1_state == true){
		HAL_UART_Transmit_IT(&huart1, msg, len);
	}
}

/* UART DATA PROCESS FUNCTION */
void process_data(uint8_t *data, uint16_t len){
	if(get_data_flag == true){
		for(int i=0; i<sizeof(data); i++){
			processed_buff[i] = data[4-i];
		}
		get_data_flag = false;
	}
	
	if(HAL_GetTick() - get_data_timer > 100 && get_data_flag == false){
		memset(&processed_buff, 0x00, sizeof(processed_buff));
		get_data_timer = HAL_GetTick();
	}
}

/* STEPPER INTERVAL CALCULATION */
uint32_t calculate_exponential_step_interval(uint32_t step){
	if (step == 0) return 10000 / base_freq;  // Initial delay

	float freq = base_freq + (step_freq - base_freq) * (1 - expf(-0.001 * step));
	return (uint32_t)(10000.0f / freq);  // Convert frequency to step delay
}

/* STEPPER MOVE FUNCTION */
void move_stepper(uint32_t step, uint8_t dir, uint32_t freq){	
	step_input = step;
	step_freq = freq;
	
	if(dir == 0){
		direction = false;
		HAL_GPIO_WritePin(DIR_GPIO_Port, DIR_Pin, GPIO_PIN_RESET);
	}
	else if(dir == 1){
		direction = true;
		HAL_GPIO_WritePin(DIR_GPIO_Port, DIR_Pin, GPIO_PIN_SET);
	}
	
	if(!step_start){
		step_start = true;
		step_period = 10000/freq;
		HAL_TIM_Base_Start_IT(&htim3);
	}
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
