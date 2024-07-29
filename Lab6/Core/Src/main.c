/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "adc.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "lcd1602.h"
#include "Delay.h"
#include <stdio.h>
#include <stdlib.h>

//#include <math.h>

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

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
	uint16_t  AD_Value ; //保存AD采样得到的值
	float voltage ;  // 保存浮点
	uint8_t  hello1[] = "Hello!\n";
	
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
	/* define constant strings for display */

	
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
  MX_USART1_UART_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */
   	LCD_init(); //  LCD 初始化
	
	uchar Str1[] = {"voltage"};
	uchar  Coma = 0x5A; //LCD显示‘：’
	uchar dot = 0x4E ;// LCD显示".";
	uchar  unit = 0x36 ;// LCD显示"v";
	uchar  Num0 = 0x50; //LCD显示数字“0”
	
	
	
	uint8_t  vol0;
	uint8_t  vol1;
	uint8_t  vol2;
	uint16_t  volt0;
	uint8_t  volt1;
	uint8_t  volt2;
	
	char buffer1[14];
	char buffer2[15];
	char buffer3[] = "Send Correct Order,Please!\n" ;
	uint8_t RxBuffer;
		
	 /*以下LCD初始显示电压AD采样值*/
	LCD_Display_String(1,0,Str1) ;  //显示"voltage:"
	LCD_Display_Char(1,12,unit);//显示”v" 	  
	LCD_Display_Char(1,7,Coma); //显示”：“  
	LCD_Display_Char(1,9, dot); //显示“.”
  
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	HAL_ADC_Start(&hadc1);//启动ADC转换
	HAL_ADC_PollForConversion(&hadc1,10);//等待转换完成，第二个参数表示超时时间，单位ms.
	if(HAL_IS_BIT_SET(HAL_ADC_GetState(&hadc1),HAL_ADC_STATE_REG_EOC)){
	 AD_Value =  HAL_ADC_GetValue(&hadc1);//读取ADC转换数据，数据为12位
		  }
	
	
		voltage =(float)AD_Value/4095 *3.3;//将AD值线性变换到0~3.3的范围，表示电压
		volt0 = (uchar)(voltage*100)%100; //小数后面两位
		volt1 = (uchar)(voltage*10)%10;  //小数后面第一位
		volt2 = (uchar)(volt0)%10;       // 小数后面第二位
	
		
		vol0 = Num0 + (int)voltage;  //模拟电压的整数部分值
		vol1 = Num0 + volt1;    //模拟电压的小数后面第一位
		vol2 = Num0 + volt2;    //模拟电压的小数后面第二位
	
		  /*以下LCD显示电压AD采样值*/ 	
	LCD_Display_Char(1,8, vol0); //显示电压值的整数部分 
	LCD_Display_Char(1,10, vol1); //显示电压值的小数第一位部分 
	LCD_Display_Char(1,11, vol2); //显示电压值的小数第二位部分 
		  
	
	
	
	/* 以下通过UART 和PC收发数据和指令*/
	HAL_UART_IRQHandler(&huart1);
		  
	HAL_UART_Receive(&huart1,&RxBuffer,2,50);
	switch(RxBuffer){
	
		case 'A': //收到指令“A”，上传电压值
			
			sprintf(buffer1,"Voltage:%.2fV\r\n",voltage);
			HAL_UART_Transmit(&huart1,(uint8_t*)buffer1,sizeof(buffer1),50);
			break;
		
		case 'B': //收到指令“B”, 上传ADC值
			sprintf(buffer2,"ADC Value:%d\r\n",AD_Value); 
			HAL_UART_Transmit(&huart1,(uint8_t*)buffer2,sizeof(buffer2),50);
			
			break;
		
		default :  //收到无效指令，显示要求输入正确指令
			
			HAL_UART_Transmit(&huart1,(uint8_t*)buffer3,sizeof(buffer3),50);
		
			break;
		
	}
		  
	
	Delay_ms(50);  
	  
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
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
