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
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define MaxBufferSize	19
#define TxBufferSize	73+3

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

union{
	float f;
	uint8_t b[4];

}converter;

uint8_t Rx_Buff[30]={0};
uint8_t test[23]={0};
uint8_t Rx;
uint8_t Tx_Buff [TxBufferSize];
int Rx_counter =0;
int test_flag=0;


float GPS_Enlem =0 , GPS_Boylam= 0,  GPS_yukseklik=0;
float Yaw_Data=0 , pitch_Data =0 , Roll_Data=0;

uint8_t ADH =0,
		ADL =0,
		CHN =0;

/************** PIF PARAMETRELERINI BURDAN GUNCELLE *****************************/
float roll_in_kd = 0.15 , roll_in_kp = 2 , roll_in_ki = 0.3,

	  roll_out_kd = 4.5 , roll_out_kp = 15 , roll_out_ki = 3.5,

      pitch_in_kd = 0.15 , pitch_in_kp = 2 , pitch_in_ki = 0.3 ,

	  pitch_out_kd = 4.5 , pitch_out_kp = 15.0 , pitch_out_ki = 3.5 ,

	  yaw_heading_kd = 15.0  , yaw_heading_kp  = 30.0 , yaw_heading_ki = 0.0001 ,

	  yaw_rate_kd = 0.05, yaw_rate_kp = 4 , yaw_rate_ki = 0.0;

/************** *************************************** *****************************/

uint8_t timer_1s_flag =0, new_data_flag = 0;;

uint8_t End_Command[3] = {0xFF, 0xFF, 0xFF};

uint8_t buffer[] = "DIGIDIK \n";

char GPS_Enlem1[33];
char GPS_Boylam1[33];
char Yaw_Data1[33];
char Roll_Data1[33];
char pitch_Data1[33];
char GPS_yukseklik1[33];
char combinedBuffer[61];
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
void PID_CONFIG_SEND(void);
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */
extern uint8_t CDC_Transmit_FS(uint8_t* Buf, uint16_t Len);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* Interrupt ile gelen verinin alınması */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){

	if(huart == &huart2)
		{

		 if ((Rx != '\n') && (Rx_counter < 30))
		 	 	 {
		            Rx_Buff[Rx_counter] = Rx;
		            Rx_counter++; }

		 else {
		            Rx=0;
		            Rx_counter = 0;
		            new_data_flag = 1;
		        }
	HAL_UART_Receive_IT(&huart2, &Rx, 1);
	//HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
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
  MX_USART2_UART_Init();
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 2 */

  HAL_UART_Receive_IT(&huart2, &Rx, 1);

//  HAL_UART_Receive_IT(&huart2, Rx_Buff, 1);
//  HAL_UART_Receive_IT(&huart1, (uint8_t *)&Rx_data, 1);


  PID_CONFIG_SEND();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

  	  converter.b[0]= Rx_Buff[0];
 	  converter.b[1]= Rx_Buff[1];
 	  converter.b[2]= Rx_Buff[2];
 	  converter.b[3]= Rx_Buff[3];

 	 Yaw_Data = converter.f;

 	  converter.b[0]= Rx_Buff[4];
 	  converter.b[1]= Rx_Buff[5];
 	  converter.b[2]= Rx_Buff[6];
 	  converter.b[3]= Rx_Buff[7];

 	 Roll_Data = converter.f;

 	  converter.b[0]= Rx_Buff[8];
 	  converter.b[1]= Rx_Buff[9];
 	  converter.b[2]= Rx_Buff[10];
 	  converter.b[3]= Rx_Buff[11];

 	 pitch_Data= converter.f;

 	  converter.b[0]= Rx_Buff[12];
 	  converter.b[1]= Rx_Buff[13];
 	  converter.b[2]= Rx_Buff[14];
 	  converter.b[3]= Rx_Buff[15];

 	  GPS_Enlem= converter.f;

 	  converter.b[0]= Rx_Buff[16];
 	  converter.b[1]= Rx_Buff[17];
 	  converter.b[2]= Rx_Buff[18];
 	  converter.b[3]= Rx_Buff[19];

 	  GPS_Boylam = converter.f;

 	  converter.b[0]= Rx_Buff[20];
 	  converter.b[1]= Rx_Buff[21];
 	  converter.b[2]= Rx_Buff[22];
 	  converter.b[3]= Rx_Buff[23];

 	  GPS_yukseklik = converter.f+110;



     snprintf(Yaw_Data1, sizeof(Yaw_Data1), "%f", Yaw_Data);
     snprintf(Roll_Data1, sizeof(Roll_Data1), "%f", Roll_Data);
     snprintf(pitch_Data1, sizeof(pitch_Data1), "%f", pitch_Data);
     snprintf(GPS_Enlem1, sizeof(GPS_Boylam1), "%f", GPS_Enlem);
     snprintf(GPS_Boylam1, sizeof(GPS_Boylam1), "%f", GPS_Boylam);
     snprintf(GPS_yukseklik1, sizeof(GPS_yukseklik1), "%f", GPS_yukseklik);


/*
     GPS_Enlem1[33]='|';
     GPS_Boylam1[33]='|';
     Yaw_Data1[33]='|';
     Roll_Data1[33]='|';
     pitch_Data1[33]='|';*/

     snprintf(combinedBuffer, sizeof(combinedBuffer), "%s,%s,%s,%s,%s,%s",
    		 Yaw_Data1,Roll_Data1, pitch_Data1, GPS_Enlem1, GPS_Boylam1,  GPS_yukseklik1);
     combinedBuffer[60]='\n';
     CDC_Transmit_FS(combinedBuffer,sizeof(combinedBuffer));
     //HAL_Delay(500);
     /*
     CDC_Transmit_FS(GPS_Enlem1,sizeof(GPS_Enlem1));
	 HAL_Delay(500);

	 CDC_Transmit_FS(GPS_Boylam1,sizeof(GPS_Boylam1));
	 HAL_Delay(500);

	 CDC_Transmit_FS(Yaw_Data1,sizeof(Yaw_Data1));
	 HAL_Delay(500);

	 CDC_Transmit_FS(Roll_Data1,sizeof(Roll_Data1));
	 HAL_Delay(500);

	 CDC_Transmit_FS(pitch_Data1,sizeof(pitch_Data1));

*/


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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 7;
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
  huart2.Init.BaudRate = 9600;
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

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void PID_CONFIG_SEND(void)
{
	 Tx_Buff[0] = ADL;
	 Tx_Buff[1] = ADH;
	 Tx_Buff[2] = CHN;

	 converter.f =roll_in_kd ;
	 Tx_Buff[3]=converter.b[0];
	 Tx_Buff[4]=converter.b[1];
	 Tx_Buff[5]=converter.b[2];
	 Tx_Buff[6]=converter.b[3];

	 converter.f =roll_in_ki ;
	 Tx_Buff[7]=converter.b[0];
	 Tx_Buff[8]=converter.b[1];
	 Tx_Buff[9]=converter.b[2];
	 Tx_Buff[10]=converter.b[3];

	 converter.f =roll_in_kp ;
	 Tx_Buff[11]=converter.b[0];
	 Tx_Buff[12]=converter.b[1];
	 Tx_Buff[13]=converter.b[2];
	 Tx_Buff[14]=converter.b[3];

	 converter.f =roll_out_kd ;
	 Tx_Buff[15]=converter.b[0];
	 Tx_Buff[16]=converter.b[1];
	 Tx_Buff[17]=converter.b[2];
	 Tx_Buff[18]=converter.b[3];

	 converter.f = roll_out_ki;
	 Tx_Buff[19]=converter.b[0];
	 Tx_Buff[20]=converter.b[1];
	 Tx_Buff[21]=converter.b[2];
	 Tx_Buff[22]=converter.b[3];

	 converter.f = roll_out_kp;
	 Tx_Buff[23]=converter.b[0];
	 Tx_Buff[24]=converter.b[1];
	 Tx_Buff[25]=converter.b[2];
	 Tx_Buff[26]=converter.b[3];

	 converter.f = pitch_in_kd;
	 Tx_Buff[27]=converter.b[0];
	 Tx_Buff[28]=converter.b[1];
	 Tx_Buff[29]=converter.b[2];
	 Tx_Buff[30]=converter.b[3];

	 converter.f = pitch_in_ki;
	 Tx_Buff[31]=converter.b[0];
	 Tx_Buff[32]=converter.b[1];
	 Tx_Buff[33]=converter.b[2];
	 Tx_Buff[34]=converter.b[3];

	 converter.f = pitch_in_kp;
	 Tx_Buff[35]=converter.b[0];
	 Tx_Buff[36]=converter.b[1];
	 Tx_Buff[37]=converter.b[2];
	 Tx_Buff[38]=converter.b[3];

	 converter.f = pitch_out_kd;
	 Tx_Buff[39]=converter.b[0];
	 Tx_Buff[40]=converter.b[1];
	 Tx_Buff[41]=converter.b[2];
	 Tx_Buff[42]=converter.b[3];

	 converter.f = pitch_out_ki;
	 Tx_Buff[43]=converter.b[0];
	 Tx_Buff[44]=converter.b[1];
	 Tx_Buff[45]=converter.b[2];
	 Tx_Buff[46]=converter.b[3];

	 converter.f = pitch_out_kp;
	 Tx_Buff[47]=converter.b[0];
	 Tx_Buff[48]=converter.b[1];
	 Tx_Buff[49]=converter.b[2];
	 Tx_Buff[50]=converter.b[3];

	 converter.f = yaw_heading_kd;
	 Tx_Buff[51]=converter.b[0];
	 Tx_Buff[52]=converter.b[1];
	 Tx_Buff[53]=converter.b[2];
	 Tx_Buff[54]=converter.b[3];

	 converter.f = yaw_heading_ki;
	 Tx_Buff[55]=converter.b[0];
	 Tx_Buff[56]=converter.b[1];
	 Tx_Buff[57]=converter.b[2];
	 Tx_Buff[58]=converter.b[3];

	 converter.f = yaw_heading_kp;
	 Tx_Buff[59]=converter.b[0];
	 Tx_Buff[60]=converter.b[1];
	 Tx_Buff[61]=converter.b[2];
	 Tx_Buff[62]=converter.b[3];

	 converter.f = yaw_rate_kd;
	 Tx_Buff[63]=converter.b[0];
	 Tx_Buff[64]=converter.b[1];
	 Tx_Buff[65]=converter.b[2];
	 Tx_Buff[66]=converter.b[3];

	 converter.f = yaw_rate_ki;
	 Tx_Buff[67]=converter.b[0];
	 Tx_Buff[68]=converter.b[1];
	 Tx_Buff[69]=converter.b[2];
	 Tx_Buff[70]=converter.b[3];

	 converter.f = yaw_rate_kp;
	 Tx_Buff[71]=converter.b[0];
	 Tx_Buff[72]=converter.b[1];
	 Tx_Buff[73]=converter.b[2];
	 Tx_Buff[74]=converter.b[3];

	 Tx_Buff[75]='\n';

	 HAL_UART_Transmit(&huart2, Tx_Buff, 76, 1000);
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
