/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2023 STMicroelectronics.
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
#include "MOTOR_PID_CONTROL.h"
#include <string.h>
#include <stdio.h>
#include <math.h>
#include <stdbool.h>
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
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim5;
TIM_HandleTypeDef htim8;
TIM_HandleTypeDef htim9;

UART_HandleTypeDef huart3;
DMA_HandleTypeDef hdma_usart3_rx;

/* USER CODE BEGIN PV */

#define MAX_MESG 100
char uartLogBuffer[MAX_MESG];
uint8_t flag_uart_rx = 0;
uint16_t uartLogRxSize;
float t1, t2, t3, t4;
uint8_t setHomeOk = 0;
uint8_t setHomeJ1 = 0, setHomeJ2 = 0, setHomeJ3 = 0, setHomeJ4 = 0;
typedef enum {
	step1, step2, step3, step4
} setHomeState_e;
bool IsHome = false;
setHomeState_e flagState = step1;
uint8_t setHome234Flag = 0;
uint8_t setHome1234Flag = 0;
MOTOR_t motor1;
MOTOR_t motor2;
MOTOR_t motor3;
MOTOR_t motor4;

// Motor 1,2,3: NHuy: 308/15, Nhu: 33
MOTOR_DRIVER_t driver1 = {
		.htimPWM = &htim8,
		.htimENC = &htim1,
		.EncCount = 0,
		.PWM_CH1 = TIM_CHANNEL_3,
		.PWM_CH2 = TIM_CHANNEL_4,
		.ENC_CH1 = TIM_CHANNEL_1,
		.ENC_CH2 = TIM_CHANNEL_2,
		.speed = 0,
		.pos = 0,
		.preSpeed = 0,
		.prePos = 0,
#ifdef NamDHay
		.ratio = (float)308/15,
#elif
		.ratio = (float)33,
#endif
};
MOTOR_DRIVER_t driver2 = {
		.htimPWM = &htim4,
		.htimENC = &htim2,
		.EncCount = 0,
		.PWM_CH1 = TIM_CHANNEL_3,
		.PWM_CH2 = TIM_CHANNEL_4,
		.ENC_CH1 = TIM_CHANNEL_1,
		.ENC_CH2 = TIM_CHANNEL_2,
		.speed = 0,
		.pos = 0,
		.preSpeed = 0,
		.prePos = 0,
#ifdef NamDHay
		.ratio = (float)308/15,
#elif
		.ratio = (float)33,
#endif
};
MOTOR_DRIVER_t driver3 = {
		.htimPWM = &htim4,
		.htimENC = &htim3,
		.EncCount = 0,
		.PWM_CH1 = TIM_CHANNEL_2,
		.PWM_CH2 = TIM_CHANNEL_1,
		.ENC_CH1 = TIM_CHANNEL_1,
		.ENC_CH2 = TIM_CHANNEL_2,
		.speed = 0,
		.pos = 0,
		.preSpeed = 0,
		.prePos = 0,
#ifdef NamDHay
		.ratio = (float)308/15,
#elif
		.ratio = (float)33,
#endif
};
MOTOR_DRIVER_t driver4 = {
		.htimPWM = &htim9,
		.htimENC = &htim5,
		.EncCount = 0,
		.PWM_CH1 = TIM_CHANNEL_1,
		.PWM_CH2 = TIM_CHANNEL_2,
		.ENC_CH1 = TIM_CHANNEL_1,
		.ENC_CH2 = TIM_CHANNEL_2,
		.speed = 0,
		.pos = 0,
		.preSpeed = 0,
		.prePos = 0,
		.ratio = 5.5 };
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM5_Init(void);
static void MX_TIM8_Init(void);
static void MX_TIM9_Init(void);
static void MX_USART3_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
float T1, T2, T3, T4;
float Tf = 3000;
float setpoint1, setpoint2, setpoint3, setpoint4;
float preSetpoint1, preSetpoint2, preSetpoint3, preSetpoint4;
float p0_1 = 0, p0_2 = 0, p0_3 = 0, p0_4 = 0;
uint8_t MagState = 0;

void UART_Handle(char *data) {
	if (flag_uart_rx == 1 && strstr(data, "\n")) {
		if (strstr(data, "t1")) {
			sscanf(data, "t1:%f,t2:%f,t3:%f,t4:%f\n", &setpoint1, &setpoint2,
					&setpoint3, &setpoint4);
		}
		if (strstr(data, "qd")) {
			sscanf(data, "qd[%f,%f,%f,%f,%f,%hhd", &setpoint1, &setpoint2,
					&setpoint3, &setpoint4, &Tf, &MagState);
		}
		if (strstr(data, "home")) {
			IsHome = true;
		}
		if (strstr(data, "Reset")) {
			HAL_NVIC_SystemReset();
		}
		if (strstr(data, "hut")) {
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, 1);
		}
		if (strstr(data, "nha")) {
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, 0);
		}
		flag_uart_rx = 0;
		memset(data, 0, strlen(data));
	}
}

void UartIdle_Init() {
	HAL_UARTEx_ReceiveToIdle_DMA(&huart3, (uint8_t*) uartLogBuffer, MAX_MESG);
	__HAL_DMA_DISABLE_IT(&hdma_usart3_rx, DMA_IT_HT);
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) {
	if (huart->Instance == USART3) {
		uartLogRxSize = Size;
		flag_uart_rx = 1;
		HAL_UARTEx_ReceiveToIdle_DMA(huart, (uint8_t*) uartLogBuffer, MAX_MESG);
	}
}

float p(float p0, float pf, float tf, float v0, float vf, float T) {
	return p0 + v0 * T
			  + (3 * (pf - p0) / (tf * tf) - 2 * v0 / tf - vf / tf) * (T * T)
			  + (-2 * (pf - p0) / (tf * tf * tf) + (vf + v0) / (tf * tf)) * (T * T * T);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	if (htim == &htim9) {
		HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);

//		This is PID test part
//		MOTOR_runAngle(&motor1);
//		MOTOR_runAngle(&motor2);
//		MOTOR_runAngle(&motor3);
//		MOTOR_runAngle(&motor4);

//		Uncomment this for real run
		static uint8_t mode = 0;
		switch (mode) {
		case 0:
			if (T1 <= Tf) {
				T1 += 5;
				MOTOR_setAngle(&motor1, p(p0_1, setpoint1, Tf, 0, 0, T1));
			}
			MOTOR_runAngle(&motor1);
			mode = 1;
			break;
		case 1:
			if (T2 <= Tf) {
				T2 += 5;
				MOTOR_setAngle(&motor2, p(p0_2, setpoint2, Tf, 0, 0, T2));
			}
			MOTOR_runAngle(&motor2);
			mode = 2;
			break;
		case 2:
			if (T3 <= Tf) {
				T3 += 5;
				MOTOR_setAngle(&motor3, p(p0_3, setpoint3, Tf, 0, 0, T3));
			}
			MOTOR_runAngle(&motor3);
			mode = 3;
			break;
		case 3:
			if (T4 <= Tf) {
				T4 += 5;
				MOTOR_setAngle(&motor4, p(p0_4, setpoint4, Tf, 0, 0, T4));
			}
			MOTOR_runAngle(&motor4);
			mode = 4;
			break;
		case 4:
			if (setpoint1 != preSetpoint1) {
				T1 = 0;
				p0_1 = MOTOR_getPos(&motor1);
				preSetpoint1 = setpoint1;
			}
			if (setpoint2 != preSetpoint2) {
				T2 = 0;
				p0_2 = MOTOR_getPos(&motor2);
				preSetpoint2 = setpoint2;
			}
			if (setpoint3 != preSetpoint3) {
				T3 = 0;
				p0_3 = MOTOR_getPos(&motor3);
				preSetpoint3 = setpoint3;
			}
			if (setpoint4 != preSetpoint4) {
				T4 = 0;
				p0_4 = MOTOR_getPos(&motor4);
				preSetpoint4 = setpoint4;
			}
			mode = 0;
			break;
		default:
			break;
		}
	}
}

void SetHome(void) {
	if(IsHome) {
		MOTOR_reset(&motor1);
		MOTOR_reset(&motor2);
		MOTOR_reset(&motor3);
		MOTOR_reset(&motor4);
		setHomeOk = 0;
		setHome234Flag = 1;
		setHome1234Flag = 1;
		setHomeJ1 = setHomeJ2 = setHomeJ3 = setHomeJ4 = 0;
		if (setHomeOk == 0)
			MOTOR_setAngle(&motor3, 300);
		IsHome = false;
	}
	if (setHome1234Flag == 1 && setHomeOk == 0) {
		if (setHomeJ1 == 1 && setHomeJ2 == 1 && setHomeJ3 == 1 && setHomeJ4 == 1) {
#ifdef NamDHay
			MOTOR_setAngle(&motor1, 0);
			MOTOR_setAngle(&motor2, 50);
			MOTOR_setAngle(&motor3, -84);
			MOTOR_setAngle(&motor4, -133);
			if (fabs(MOTOR_getPos(&motor2) - 50)  < 1
			 && fabs(MOTOR_getPos(&motor3) + 84)  < 1
			 && fabs(MOTOR_getPos(&motor4) + 133) < 1) {
				setHome234Flag = 0;
				setHome1234Flag = 0;
				setHomeOk = 1;
				MOTOR_reset(&motor2);
				MOTOR_reset(&motor3);
				MOTOR_reset(&motor4);
			}
#elif
			MOTOR_setAngle(&motor1, 0);
			MOTOR_setAngle(&motor2, 40);
			MOTOR_setAngle(&motor3, -82);
			MOTOR_setAngle(&motor4, -125);
			if (fabs(MOTOR_getPos(&motor2) - 40)  < 1
			 && fabs(MOTOR_getPos(&motor3) + 82)  < 1
			 && fabs(MOTOR_getPos(&motor4) + 125) < 1) {
				setHome234Flag = 0;
				setHome1234Flag = 0;
				setHomeOk = 1;
				MOTOR_reset(&motor2);
				MOTOR_reset(&motor3);
				MOTOR_reset(&motor4);
			}
#endif
		} else if (setHomeJ2 == 1 && setHomeJ3 == 1 && setHomeJ4 == 1) {
			if (fabs(motor1.setPoint) <= 2) {
				MOTOR_setAngle(&motor1, -20.0f);
			} else if (fabs(MOTOR_getPos(&motor1) + 20.0f) < 2
				  && (int) (motor1.setPoint / motor1.ratioJoint) == -20) {
				MOTOR_setAngle(&motor1, 20.0f);
			} else if (fabs(MOTOR_getPos(&motor1) - 20.0f) < 2
				  && (int) (motor1.setPoint / motor1.ratioJoint) == 20) {
				MOTOR_setAngle(&motor1, -40.0f);
			} else if (fabs(MOTOR_getPos(&motor1) + 40.0f) < 2
				  && (int) (motor1.setPoint / motor1.ratioJoint) == -40) {
				MOTOR_setAngle(&motor1, 40.0f);
			} else if (fabs(MOTOR_getPos(&motor1) - 40.0f) < 2
				  && (int) (motor1.setPoint / motor1.ratioJoint) == 40) {
				MOTOR_setAngle(&motor1, -60.0f);
			} else if (fabs(MOTOR_getPos(&motor1) + 60.0f) < 2
				  && (int) (motor1.setPoint / motor1.ratioJoint) == -60) {
				MOTOR_setAngle(&motor1, 60.0f);
			} else if (fabs(MOTOR_getPos(&motor1) - 60.0f) < 2
				  && (int) (motor1.setPoint / motor1.ratioJoint) == 60) {
				MOTOR_setAngle(&motor1, -80.0f);
			} else if (fabs(MOTOR_getPos(&motor1) + 80.0f) < 2) {
				MOTOR_setAngle(&motor1, 80.0f);
			}
		}
		if (HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_6) == 0 && setHomeJ1 == 0) {
			setHomeJ1 = 1;
			MOTOR_reset(&motor1);
		}
		if (HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_2) == 0 && setHomeJ2 == 0) {
			setHomeJ2 = 1;
			MOTOR_reset(&motor2);
		}
		if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_11) == 0 && setHomeJ3 == 0) {
			setHomeJ3 = 1;
			MOTOR_reset(&motor3);
			if (setHomeJ2 == 0) {
				MOTOR_setAngle(&motor2, -300);
			}
			if (setHomeJ4 == 0) {
				MOTOR_setAngle(&motor4, 300);
			}
		}
		if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_7) == 0 && setHomeJ4 == 0) {
			setHomeJ4 = 1;
			MOTOR_reset(&motor4);
		}
	} else if (setHome1234Flag == 1 && setHomeOk == 1) {
		setpoint1 = 0;
		setpoint2 = 0;
		setpoint3 = 0;
		setpoint4 = 0;
		setHome1234Flag = 0;
	}
}
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	if (setHome1234Flag == 1 && setHomeOk == 0) {
		if (GPIO_Pin == GPIO_PIN_2 && setHomeJ2 == 0) {
			setHomeJ2 = 1;
			MOTOR_reset(&motor2);
		}
		if (GPIO_Pin == GPIO_PIN_11 && setHomeJ3 == 0) {
			setHomeJ3 = 1;
			MOTOR_reset(&motor3);
			if (setHomeJ2 == 0) {
				MOTOR_setAngle(&motor2, -300);
			}
			if (setHomeJ4 == 0) {
				MOTOR_setAngle(&motor4, 300);
			}
		}
		if (GPIO_Pin == GPIO_PIN_7 && setHomeJ4 == 0) {
			setHomeJ4 = 1;
			MOTOR_reset(&motor4);
		}
		if (GPIO_Pin == GPIO_PIN_6 && setHomeJ1 == 0) {
			setHomeJ1 = 1;
			MOTOR_reset(&motor1);
		}
	}
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
  MX_DMA_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM5_Init();
  MX_TIM8_Init();
  MX_TIM9_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */

  MOTOR_setPIDPosition(&motor1, 5, 0, 0, 5);
  MOTOR_setPIDVelocity(&motor1, 30, 150, 0, 5);
  MOTOR_setOutputRange(&motor1, -222, 222);
  MOTOR_setWindupRange(&motor1, -200, 200);
  MOTOR_init(&motor1, &driver1, 1, GPIO_PIN_6, 500);

  MOTOR_setPIDPosition(&motor2, 5, 0, 0, 5);
  MOTOR_setPIDVelocity(&motor2, 30, 150, 0, 5);
  MOTOR_setOutputRange(&motor2, -444, 444);
  MOTOR_setWindupRange(&motor2, -400, 400);
  MOTOR_init(&motor2, &driver2, 2, GPIO_PIN_2, 500);

  MOTOR_setPIDPosition(&motor3, 5, 0, 0, 5);
  MOTOR_setPIDVelocity(&motor3, 30, 150, 0, 5);
  MOTOR_setOutputRange(&motor3, -555, 555);
  MOTOR_setWindupRange(&motor3, -500, 500);
  MOTOR_init(&motor3, &driver3, 2, GPIO_PIN_11, 500);

  MOTOR_setPIDPosition(&motor4, 5, 0, 0, 5);
  MOTOR_setPIDVelocity(&motor4, 3, 50, 0, 5);
  MOTOR_setOutputRange(&motor4, -222, 222);
  MOTOR_setWindupRange(&motor4, -200, 200);
  MOTOR_init(&motor4, &driver4, 1, GPIO_PIN_7, 500);

  HAL_TIM_Base_Start_IT(&htim9);
  htim9.Instance->ARR = 999;
  UartIdle_Init();

  uint32_t pre_time = HAL_GetTick();
  char data_angle[30];
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
 	while (1) {
//		Uncomment this for real run
		UART_Handle(uartLogBuffer);
		SetHome();
		if (HAL_GetTick() - pre_time >= 200) {
			sprintf(data_angle, "t1:%.1f,t2:%.1f,t3:%.1f,t4:%.1f\n",
					(float) MOTOR_getPos(&motor1),
					(float) MOTOR_getPos(&motor2),
					(float) MOTOR_getPos(&motor3),
					(float) MOTOR_getPos(&motor4));
			HAL_UART_Transmit(&huart3, (uint8_t*) data_angle,
					strlen(data_angle), HAL_MAX_DELAY);

			pre_time = HAL_GetTick();
		}

//		This is DC test part
//		MOTOR_driver_rotary(&driver1, -200);
//		MOTOR_driver_rotary(&driver2, -200);
//		MOTOR_driver_rotary(&driver3, -200);
//		MOTOR_driver_rotary(&driver4, -200);
//
//		MOTOR_driver_readPosAndSpeed(&driver1, &motor1.pos, &motor1.speed,
//				motor1.PIDPosition.Sample_time);
//		MOTOR_driver_readPosAndSpeed(&driver2, &motor2.pos, &motor2.speed,
//				motor2.PIDPosition.Sample_time);
//		MOTOR_driver_readPosAndSpeed(&driver3, &motor3.pos, &motor3.speed,
//				motor3.PIDPosition.Sample_time);
//		MOTOR_driver_readPosAndSpeed(&driver4, &motor4.pos, &motor4.speed,
//				motor4.PIDPosition.Sample_time);
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

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

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4294967295;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim2, &sConfig) != HAL_OK)
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

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim3, &sConfig) != HAL_OK)
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
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 84-1;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 999;
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
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

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

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 0;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 4294967295;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim5, &sConfig) != HAL_OK)
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
  * @brief TIM8 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM8_Init(void)
{

  /* USER CODE BEGIN TIM8_Init 0 */

  /* USER CODE END TIM8_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM8_Init 1 */

  /* USER CODE END TIM8_Init 1 */
  htim8.Instance = TIM8;
  htim8.Init.Prescaler = 168-1;
  htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim8.Init.Period = 999;
  htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim8.Init.RepetitionCounter = 0;
  htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim8, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK)
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
  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
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
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim8, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM8_Init 2 */

  /* USER CODE END TIM8_Init 2 */
  HAL_TIM_MspPostInit(&htim8);

}

/**
  * @brief TIM9 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM9_Init(void)
{

  /* USER CODE BEGIN TIM9_Init 0 */

  /* USER CODE END TIM9_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM9_Init 1 */

  /* USER CODE END TIM9_Init 1 */
  htim9.Instance = TIM9;
  htim9.Init.Prescaler = 168-1;
  htim9.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim9.Init.Period = 999;
  htim9.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim9.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim9) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim9, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim9) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim9, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim9, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM9_Init 2 */

  /* USER CODE END TIM9_Init 2 */
  HAL_TIM_MspPostInit(&htim9);

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream1_IRQn, 3, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream1_IRQn);

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PC7 PC10 PC11 */
  GPIO_InitStruct.Pin = GPIO_PIN_7|GPIO_PIN_10|GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PD2 PD6 */
  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : PB6 */
  GPIO_InitStruct.Pin = GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI2_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

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
	while (1) {
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
