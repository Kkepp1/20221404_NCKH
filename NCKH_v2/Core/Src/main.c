/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stdlib.h"
#include "stdio.h"
#include "string.h"
#include "mpu6050.h"
#include "PID.h"
/* Private includes ----------------------------------------------------------*/

float time_0 = 0.0;
float time_1 = 0.0;
float timeCurrent=0.0;
// -----------------pid----------------------------
#define TEST_MODE (0)
#define NORMAL_MODE (1)
#define SP_VALUE 180.5
#define SP_UP			SP_VALUE+2
#define SP_DOWN			SP_VALUE-2
#define TURN_SPEED 200
#define TIME_SAMPLE 0.001


float  u=0;
float  yu=0;
float yuT=0;
float yuP=0;
float uT=0;
float uP=0;
float AddT=0;
float AddP=0;

// 200 30 1
float SP = SP_VALUE;
float Pitch=0;
float Kp=350 ;//5.5 175
float Ki=59;//1.5
float Kd =1;//2

float ySP=0;
float Yaw=0;
float yKp=2;//5.5 175
float yKi=0;//1.5
float yKd =0;//2
char Tx_data[10];
char Rx_data;
int Rx_index=0;
char Rx_data1[100];

float Error_calib=0.11;

float pTerm, iTerm, dTerm, last_error, error;
float ypTerm, yiTerm, ydTerm, ylast_error, yerror;

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart3;
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart3_rx;

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM4_Init(void);
static void MX_DMA_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART3_UART_Init(void);
//-------------------------------------------function---------------------

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
	if(htim->Instance==TIM4){
		 int dev=0;
		
//------------------------------------------------------UART----------------------------------------
#if NORMAL_MODE
			HAL_UART_Receive_DMA(&huart3,(uint8_t*)&Rx_data,1);
			SP=SP_VALUE;
			AddT=0;
			AddP=0;
//			uP=0;
//			uT=0;
		
			if(Rx_data=='U'){
			SP=SP_VALUE;
			ySP=0;
//			AddT=0;
//			AddP=0;
		}else if(Rx_data=='D'){
			SP=SP_VALUE;
			if(Yaw>0){
				ySP=180;
			}else ySP=-180;
			
//			AddT=0;
//			AddP=0;
			
		}else if(Rx_data=='R'){
			SP=SP_VALUE;
			ySP=90;

//			AddT=-TURN_SPEED;
//			AddP=TURN_SPEED;
		}else if(Rx_data=='L'){
			SP=SP_VALUE;
			ySP=-90;
//			AddT=TURN_SPEED;
//			AddP=-TURN_SPEED;
		}else if(Rx_data=='u'){
			SP=SP_UP;
			ySP=ySP;
//			AddT=TURN_SPEED;
//			AddP=-TURN_SPEED;
		}else if(Rx_data=='d'){
			SP=SP_DOWN;
			ySP=ySP;
//			AddT=TURN_SPEED;
//			AddP=-TURN_SPEED;
		}else if(Rx_data=='O'){
			SP=SP_VALUE;
			ySP=ySP;
//			AddT=0;
//			AddP=0;
		}else{
			SP=SP_VALUE;
			ySP=ySP;
//			AddT=0;
//			AddP=0;
		}
#endif	
//---------------------------------------------------PID---------------------------------------------------------
					MPU6050_GET_ANGLE(&hi2c1,&Pitch,&Yaw,Error_calib,TIME_SAMPLE);
					u = PID(SP,Pitch,Kp,Ki,Kd,&error,&last_error,&pTerm,&iTerm,&dTerm);
					yu= PID(ySP,Yaw,yKp,yKi,yKd,&yerror,&ylast_error,&ypTerm,&yiTerm,&ydTerm);

							yuT=yu;
							yuP=-yu;
				
//			if(yuT<0) // angle>180
//				{
//					yuT=0-yuT;
//					HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4,GPIO_PIN_SET);//A1
//					HAL_GPIO_WritePin(GPIOA,GPIO_PIN_5,GPIO_PIN_RESET);//B1
//					__HAL_TIM_SetCompare(&htim2,TIM_CHANNEL_1,yuT);//98

//				}
//				else
//				{
//					HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4,GPIO_PIN_RESET);//A1
//					HAL_GPIO_WritePin(GPIOA,GPIO_PIN_5,GPIO_PIN_SET);//B1
//					__HAL_TIM_SetCompare(&htim2,TIM_CHANNEL_1,yuT);
//		
//				}
//			if(yuP<0) // angle>180
//				{
//					yuP=0-yuP;
//					HAL_GPIO_WritePin(GPIOA,GPIO_PIN_6,GPIO_PIN_SET);//A1
//					HAL_GPIO_WritePin(GPIOA,GPIO_PIN_7,GPIO_PIN_RESET);//B1
//					__HAL_TIM_SetCompare(&htim2,TIM_CHANNEL_2,yuP);//98

//				}
//				else
//				{
//					HAL_GPIO_WritePin(GPIOA,GPIO_PIN_6,GPIO_PIN_RESET);//A1
//					HAL_GPIO_WritePin(GPIOA,GPIO_PIN_7,GPIO_PIN_SET);//B1
//					__HAL_TIM_SetCompare(&htim2,TIM_CHANNEL_2,yuP);
//		
//				}			
					
					uT=u+yuT;
					uP=u+yuP;
					Constant(&uT,999,-999);
					Constant(&uP,999,-999);
//					if ( uT > 999 ) {
//						uT = 999;}
//					else if ( uT< -999 ) uT= -999 ;	
//					if ( uP > 999 ) {
//						uP = 999;}
//					else if ( uP< -999 ) uP= -999 ;	
			if(uT<0) 
				{
					uT=0-uT;
					HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4,GPIO_PIN_SET);//A1
					HAL_GPIO_WritePin(GPIOA,GPIO_PIN_5,GPIO_PIN_RESET);//B1
					__HAL_TIM_SetCompare(&htim2,TIM_CHANNEL_1,uT);//98

				}
				else
				{
					HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4,GPIO_PIN_RESET);//A1
					HAL_GPIO_WritePin(GPIOA,GPIO_PIN_5,GPIO_PIN_SET);//B1
					__HAL_TIM_SetCompare(&htim2,TIM_CHANNEL_1,uT);
		
				}
			if(uP<0) // angle>180
				{
					uP=0-uP;
					HAL_GPIO_WritePin(GPIOA,GPIO_PIN_6,GPIO_PIN_SET);//A1
					HAL_GPIO_WritePin(GPIOA,GPIO_PIN_7,GPIO_PIN_RESET);//B1
					__HAL_TIM_SetCompare(&htim2,TIM_CHANNEL_2,uP);//98

				}
				else
				{
					HAL_GPIO_WritePin(GPIOA,GPIO_PIN_6,GPIO_PIN_RESET);//A1
					HAL_GPIO_WritePin(GPIOA,GPIO_PIN_7,GPIO_PIN_SET);//B1
					__HAL_TIM_SetCompare(&htim2,TIM_CHANNEL_2,uP);
		
				}			

		}
}
void Split_String(){
			const char s[5]=" .,-";
			char* str_Angle;
			char* str_Kp;
			char* str_Ki;	
			char* str_Kd;
	
			str_Angle=strtok(Rx_data1,s);
			str_Kp=strtok(NULL,s);
			str_Ki=strtok(NULL,s);
			str_Kd=strtok(NULL,s);
			if(strcmp(str_Angle,"pitch")==0){
					Kp=strtof(str_Kp,NULL);
					Ki=strtof(str_Ki,NULL);
					Kd=strtof(str_Kd,NULL);
			
			}else if(strcmp(str_Angle,"yaw")==0){
					yKp=strtof(str_Kp,NULL);
					yKi=strtof(str_Ki,NULL);
					yKd=strtof(str_Kd,NULL);
			}
			for(int i=0;i<100;i++){
					Rx_data1[i]=' ';
			}
			Rx_index=0;
}
#if TEST_MODE
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
	if(huart->Instance==huart3.Instance){
			HAL_UART_Receive_DMA(&huart3,(uint8_t*)&Rx_data,1);
			if(Rx_data!='\n'){
					Rx_data1[Rx_index++]=Rx_data;
			}else{
					Split_String();
			}
	}
}
#endif
/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM2_Init();
  
  MX_DMA_Init();
  MX_USART1_UART_Init();
  MX_I2C1_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */
	MPU6050_Init(&hi2c1);
	//calib_MPU6050(&hi2c1,2000,&Error_calib);
	// start timer 4 init
	MX_TIM4_Init();
	
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);	
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
	//HAL_TIM_Base_Start_IT(&htim3);
	HAL_TIM_Base_Start_IT(&htim4);
	HAL_UART_Receive_DMA(&huart3,(uint8_t*)&Rx_data,1);
	
	
  while (1)
  {
//		time_1=HAL_GetTick();
//		if((time_1-time_0)==1){
//					//AngleNow++;
//					sprintf(Tx_data, "%f", AngleNow);
////		     HAL_UART_Transmit(&huart3,(uint8_t*)Tx_data,sizeof(Tx_data),100);
//		
//					time_0=time_1;
//		}

  }
  /* USER CODE END 3 */
}
#ifdef __GNUC__
/* With GCC/RAISONANCE, small printf (option LD Linker->Libraries->Small printf
 set to 'Yes') calls __io_putchar() */
 #define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
 #define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */

/**
 * @brief Retargets the C library printf function to the USART.
 * @param None
 * @retval None
 */
PUTCHAR_PROTOTYPE 
{
 /* Place your implementation of fputc here */
 /* e.g. write a character to the USART */
 HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, 9);


 return ch;
}
/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
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
  hi2c1.Init.ClockSpeed = 100000;
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
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 71;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 999;
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
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

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
  htim4.Init.Prescaler = 3600-1;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 9;
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
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  huart3.Init.BaudRate = 9600;
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
  /* DMA1_Channel3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel3_IRQn);
  /* DMA1_Channel5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel5_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pins : PA4 PA5 PA6 PA7 */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
