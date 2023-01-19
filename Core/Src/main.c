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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "math.h"
#include "stm32f4xx_hal.h"
#include "stdio.h"
#include "string.h"
#include "bno055.h"
#include "bno055_stm32.h"
#include "CANSPI.h"
#include "MCP2515.h"
#include <stdlib.h>
#include "i2c-lcd.h"
#include "stm32_tm1637.h"
#include "i2c.h"
#include "wifi_ubidots.h"
//asdfasdf

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define min(a,b) (((a) < (b)) ? (a) : (b))
#define max(a,b) (((a) > (b)) ? (a) : (b))
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;
ADC_HandleTypeDef hadc3;

I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c3;

SPI_HandleTypeDef hspi2;

UART_HandleTypeDef huart4;
UART_HandleTypeDef huart5;
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI2_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_UART4_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_UART5_Init(void);
static void MX_ADC1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_ADC3_Init(void);
static void MX_ADC2_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/*
int32_t vADC1;
int32_t vADC2;
int32_t vADC3;
 */

// VALUE :  correlated to battery pack's temperature

#define ADC1_Channel_num 2
int ADCval;
float vADC1[ADC1_Channel_num];
float vADC2;
float vADC3;
float Btemp; 					// max temperature from 4 ADC to temp value

char buffer[100]=" ";				// buffer to read each battery pack's temperature
char string_temperature[7];			// buffer to send average temperature of 4 battery pack
char string_voltage[6];				// buffer to read each battery pack's voltage

int row;							// row line at LCD
int col;							// column line at LCD

uint8_t receive_data[7];			// received buffer from Arduino Master BMS
float percent_of_batt;				// percentage value of batter voltage

/* VALUE : CANOPEN

uCAN_MSG txMessage;
uCAN_MSG rxMessage;

uint32_t RPM;
float torque;
float temp;

 */

uint8_t Rbuff[100];
uint8_t msg[256];
uint8_t Tbuff[256];
uint8_t TempBuff[256];

void PrintString(uint8_t * string)
{
	HAL_UART_Transmit(&huart2, (uint8_t *)string, strlen((char *)string), 0xffff);
}

// void CANopen(void);
void ADC1_Select_CH0(void);
void ADC1_Select_CH9(void);
float ADCtoTemp(float ADCval);

char str[100]="22 KSAE FOURTOR\r\n";

char bad[100]="Thermal Protector : DISCONNECTED\r\n";

char good[100]="Thermal Protector : CONNECTED\r\n";

char bms[100]="Receive BMS signal from Arudino\r\n";

char BMSbuff[100]="";

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
  MX_SPI2_Init();
  MX_USART2_UART_Init();
  MX_UART4_Init();
  MX_USART3_UART_Init();
  MX_UART5_Init();
  MX_ADC1_Init();
  MX_I2C3_Init();
  MX_USART1_UART_Init();
  MX_ADC3_Init();
  MX_ADC2_Init();
  /* USER CODE BEGIN 2 */
  PrintString(str);

//	CANSPI_Initialize();

	lcd_init();
	lcd_clear();
	HAL_Delay(20);
	lcd_put_cur(0,0);
	HAL_Delay(20);
	lcd_send_string((char*)"B.Temp: Init");
	lcd_put_cur(1,0);
	HAL_Delay(20);
	lcd_send_string((char*)"B.Vol: Init");

	tm1637Init();
	tm1637SetBrightness(5);

	// ------------------------------------ **** IMU Setting ****------------------------------------

	bno055_calibration_data_t calData;
	bno055_assignI2C(&hi2c3);
	bno055_setup();
	bno055_setCalibrationData(calData);//Set init
	bno055_setOperationMode(BNO055_OPERATION_MODE_IMU);//?��?�� 모드
	bno055_setPowerMode(BNO55_POWER_MODE_LOWPOWER);//?��?�� 모드

	//  WifiSetup();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1)
	{
		//------------------------------------ **** CANopen Setting ****------------------------------------//

		// CANopen();

		//------------------------------------ **** Temperature sensing ****------------------------------------//

		/* [seki 22] Digital signal pin
		 * Temperature signal using seki 22
		 */

		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
		GPIO_PinState PinStat_seg1 = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_4);

		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_SET);
		GPIO_PinState PinStat_seg2 = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_6);

		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_SET);
		GPIO_PinState PinStat_seg3 = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_14);

		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_SET);
		GPIO_PinState PinStat_seg4 = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_11);

		GPIO_PinState Thermal=(PinStat_seg1&&PinStat_seg2)&&(PinStat_seg3&&PinStat_seg4);

		/*
		GPIO_PinState PinStat=PinStat_seg4;

		if(!PinStat){
			PrintString(bad);
		}
		PrintString(good);
		 */

		/*		[NTC] analog read pin
		 * 		Code for ADC1
		HAL_ADC_Start(&hadc1);
		if(HAL_ADC_PollForConversion(&hadc1, 5)==HAL_OK)
		{
			vADC1 = HAL_ADC_GetValue(&hadc1);
			Btemp = vADC1 * (3.3/4095);
			Btemp = (Btemp *9840)/(3.3-Btemp);

			Btemp = (298.15*3700)/(298.15*log(9840/Btemp)+3700);
			Btemp = Btemp - 273.15;

			sprintf(buffer,"ADC 1 : %.2f \r\n",Btemp);
		}
		if (isnan(Btemp) == 0)
		{
			HAL_UART_Transmit(&huart2,(uint8_t*)buffer,sizeof(buffer),1000);
			HAL_Delay(100);
		}
		*/
		/*

		HAL_ADC_Start(&hadc2);
		if(HAL_ADC_PollForConversion(&hadc2, 5)==HAL_OK)
		{
			vADC2 = HAL_ADC_GetValue(&hadc2);
			vADC2 = vADC2 * (3.3/4095);
			vADC2 = (vADC2 *9840)/(3.3-vADC2);

			vADC2 = (298.15*3700)/(298.15*log(9840/vADC2)+3700);
			vADC2 = vADC2 - 273.15;
		}

		HAL_ADC_Start(&hadc3);
		if(HAL_ADC_PollForConversion(&hadc3, 5)==HAL_OK)
		{
			vADC3 = HAL_ADC_GetValue(&hadc3);
			vADC3 = vADC3 * (3.3/4095);
			vADC3 = (vADC3 *9840)/(3.3-vADC3);

			vADC3 = (298.15*3700)/(298.15*log(9840/vADC3)+3700);
			vADC3 = vADC3 - 273.15;

		}
		sprintf(buffer,"ADC 2 : %.2f \r\n",vADC2);
		if (isnan(vADC1[1]) == 0)
		{
			HAL_UART_Transmit(&huart2,(uint8_t*)buffer,sizeof(buffer),1000);
			HAL_Delay(100);
		}
		*/
		float tempADC[4];
		ADC1_Select_CH0();
		HAL_ADC_Start(&hadc1);
		HAL_ADC_PollForConversion(&hadc1,1000);
		vADC1[0]=HAL_ADC_GetValue(&hadc1);
		HAL_ADC_Stop(&hadc1);
		tempADC[0]= ADCtoTemp(vADC1[0]);

		ADC1_Select_CH9();
		HAL_ADC_Start(&hadc1);
		HAL_ADC_PollForConversion(&hadc1,1000);
		vADC1[1]=HAL_ADC_GetValue(&hadc1);
		HAL_ADC_Stop(&hadc1);
		tempADC[1] = ADCtoTemp(vADC1[1]);

		HAL_ADC_Start(&hadc2);
		HAL_ADC_PollForConversion(&hadc2,5);
		vADC2=HAL_ADC_GetValue(&hadc2);
		HAL_ADC_Stop(&hadc2);
		tempADC[2]= ADCtoTemp(vADC2);

		HAL_ADC_Start(&hadc3);
		HAL_ADC_PollForConversion(&hadc3,5);
		vADC3=HAL_ADC_GetValue(&hadc3);
		HAL_ADC_Stop(&hadc3);
		tempADC[3]= ADCtoTemp(vADC3);

		Btemp = max(tempADC[0],tempADC[1]);
		Btemp = max(Btemp, tempADC[2]);
		Btemp = max(Btemp, tempADC[3]);



		// sprintf(buffer,"ADC1_CH0 : %.2f \r\nADC1_CH9 : %.2f\r\nADC2_CH1 : %.2f \r\nADC3_CH10: %.2f\r\nMAX: %.2f\r\n\n",vADC1[0],vADC1[1],vADC2,vADC3,Btemp);
		sprintf(buffer,"SEGMENT[1] : %.2f \r\nSEGMENT[2] : %.2f\r\nSEGMENT[3] : %.2f \r\nSEGMENT[4]: %.2f\r\nMAX: %.2f\r\n\n",vADC1[0],vADC1[1],vADC2,vADC3,Btemp);
		// PrintString(buffer);

		//------------------------------------ **** Master BMS ****------------------------------------//

		HAL_UART_Receive(&huart3,receive_data,7,200);						// Read Voltage and Celus temperature Data
		// sprintf(string_voltage,"%d.%dV",receive_data[1],receive_data[2]);	//Make String Voltage Data
		// sprintf(string_temperature,"%d'C",receive_data[0]);					//Make String temperature Data
		// HAL_UART_Transmit(&huart2,(char*)receive_data,strlen((char*)receive_data),1000);
		// HAL_UART_Transmit(&huart2,BMSbuff,strlen(BMSbuff),1000);

		// Tester
		/*
		sprintf(BMSbuff,"[1] : %c ",receive_data[0]);
		HAL_UART_Transmit(&huart2,BMSbuff,strlen(BMSbuff),1000);
		sprintf(BMSbuff,"[2] : %c ",receive_data[1]);
		HAL_UART_Transmit(&huart2,BMSbuff,strlen(BMSbuff),1000);
		sprintf(BMSbuff,"[3] : %c ",receive_data[2]);
		HAL_UART_Transmit(&huart2,BMSbuff,strlen(BMSbuff),1000);
		sprintf(BMSbuff,"[4] : %c ",receive_data[3]);
		HAL_UART_Transmit(&huart2,BMSbuff,strlen(BMSbuff),1000);
		sprintf(BMSbuff,"[5] : %c ",receive_data[4]);
		HAL_UART_Transmit(&huart2,BMSbuff,strlen(BMSbuff),1000);
		sprintf(BMSbuff,"[6] : %c ",receive_data[5]);
		HAL_UART_Transmit(&huart2,BMSbuff,strlen(BMSbuff),1000);
		sprintf(BMSbuff,"[7] : %c\n\r",receive_data[6]);
		HAL_UART_Transmit(&huart2,BMSbuff,strlen(BMSbuff),1000);
		HAL_Delay(100);
		*/

		// Transmit voltage value from Arduino
		if(receive_data[1]==1){ //the solution that last bit is garbage value when the voltage lower than 100.
		sprintf(BMSbuff,"%c%c%c%c%c%c",receive_data[1],receive_data[2],receive_data[3],receive_data[4],receive_data[5],receive_data[6]);
		}
		else{
		sprintf(BMSbuff,"%c%c%c%c%c ",receive_data[1],receive_data[2],receive_data[3],receive_data[4],receive_data[5]);
		}

		HAL_UART_Transmit(&huart2,BMSbuff,strlen(BMSbuff),1000);

		//------------------------------------ **** Shut Down Circuit & Over load current shut down ****------------------------------------//

		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);

		GPIO_PinState StateOfBMS = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_8);			// rx sig from Arduino
		GPIO_PinState SDC_ON =StateOfBMS&&Thermal;
		//GPIO_PinState SDC_ON;

		//  check code for Arduino Uno SDC signal
		/*
		if(StateOfBMS){
			PrintString("BMS STATUS : OK\n\r");
		}
		else{
			PrintString("BMS STATUS : FAULT\n\r");
		}
		 */

		// check code for thermal protector
		/*
		if(Thermal){
			PrintString("THERMAL : OK\n\r");
		}
		else{
			PrintString("THERMAL : FAULT\n\r");
		}
		*/

		if(!SDC_ON)
		{
			// PrintString("SDC activate\n\r");
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);			//  SDC 3output
		}

		//------------------------------------ **** BNO055 IMU sensor ****------------------------------------//
/*
		 bno055_vector_t gyro = bno055_getVectorGyroscope();
		 bno055_vector_t acc = bno055_getVectorAccelerometer();
		 bno055_vector_t v = bno055_getVectorEuler();
		 v = bno055_getVectorQuaternion();

		 sprintf(buffer,"%.2f,%.2f,%.2f\r\n",gyro.x,gyro.y,gyro.z);
		 PrintString(buffer);
		 HAL_Delay(100);
*/
		//------------------------------------ **** 7 Segment Setting ****------------------------------------//
		// Before : RPM from motor
		// After  : percentage value from Arduino Master BMS

		float total_v  = atof(BMSbuff);
		percent_of_batt = (double)(total_v - 78.4)*2.55;
		int percent_of_batt_int=(int)(percent_of_batt*100);
		tm1637DisplayDecimal(percent_of_batt_int,2); // express the dot in 7segment
		// tm1637DisplayDecimal(percent_of_batt,1);

		//------------------------------------ **** LCD Setting ****------------------------------------//

		lcd_put_cur(0,7);							// Move Cursor
		// HAL_Delay(50);
		sprintf(string_temperature,"%.2f   ",Btemp);	// Make String for Temperature Data
		lcd_send_string((char*)string_temperature); // Battery Temperature

		lcd_put_cur(1,6);
		// HAL_Delay(50);
		// sprintf(string_voltage,"%d     ",4);		// Make String for Voltage Data
		// lcd_send_string((char*)string_voltage); 	// Battery Voltage
//		char a[6];
//		for(int i=0;i<6;i++){
//			a[i]=BMSbuff[i];
//		}
//		lcd_send_string((char*)a);
		lcd_send_string((char*)BMSbuff);

		//------------------------------------ **** Telemetry using WiFi-Ubidots ****------------------------------------//

		// WifiUbidots();

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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 2;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_9;
  sConfig.Rank = 2;
  sConfig.SamplingTime = ADC_SAMPLETIME_15CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */
  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief ADC2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC2_Init(void)
{

  /* USER CODE BEGIN ADC2_Init 0 */

  /* USER CODE END ADC2_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC2_Init 1 */

  /* USER CODE END ADC2_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc2.Instance = ADC2;
  hadc2.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc2.Init.Resolution = ADC_RESOLUTION_12B;
  hadc2.Init.ScanConvMode = DISABLE;
  hadc2.Init.ContinuousConvMode = DISABLE;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.NbrOfConversion = 1;
  hadc2.Init.DMAContinuousRequests = DISABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */

}

/**
  * @brief ADC3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC3_Init(void)
{

  /* USER CODE BEGIN ADC3_Init 0 */

  /* USER CODE END ADC3_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC3_Init 1 */

  /* USER CODE END ADC3_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc3.Instance = ADC3;
  hadc3.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc3.Init.Resolution = ADC_RESOLUTION_12B;
  hadc3.Init.ScanConvMode = DISABLE;
  hadc3.Init.ContinuousConvMode = DISABLE;
  hadc3.Init.DiscontinuousConvMode = DISABLE;
  hadc3.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc3.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc3.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc3.Init.NbrOfConversion = 1;
  hadc3.Init.DMAContinuousRequests = DISABLE;
  hadc3.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc3) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_10;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC3_Init 2 */

  /* USER CODE END ADC3_Init 2 */

}


/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */
	//
  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */
	//
  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */
	//
  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief UART4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART4_Init(void)
{

  /* USER CODE BEGIN UART4_Init 0 */

  /* USER CODE END UART4_Init 0 */

  /* USER CODE BEGIN UART4_Init 1 */

  /* USER CODE END UART4_Init 1 */
  huart4.Instance = UART4;
  huart4.Init.BaudRate = 115200;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART4_Init 2 */

  /* USER CODE END UART4_Init 2 */

}

/**
  * @brief UART5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART5_Init(void)
{

  /* USER CODE BEGIN UART5_Init 0 */

  /* USER CODE END UART5_Init 0 */

  /* USER CODE BEGIN UART5_Init 1 */

  /* USER CODE END UART5_Init 1 */
  huart5.Instance = UART5;
  huart5.Init.BaudRate = 115200;
  huart5.Init.WordLength = UART_WORDLENGTH_8B;
  huart5.Init.StopBits = UART_STOPBITS_1;
  huart5.Init.Parity = UART_PARITY_NONE;
  huart5.Init.Mode = UART_MODE_TX_RX;
  huart5.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart5.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart5) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART5_Init 2 */

  /* USER CODE END UART5_Init 2 */

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
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5|GPIO_PIN_7|GPIO_PIN_12, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|GPIO_PIN_12|GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pins : PA4 PA6 PA11 */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_6|GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PA5 PA7 PA12 */
  GPIO_InitStruct.Pin = GPIO_PIN_5|GPIO_PIN_7|GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB12 PB15 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_12|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PB14 */
  GPIO_InitStruct.Pin = GPIO_PIN_14;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : ArduinoSDC_Pin */
  GPIO_InitStruct.Pin = ArduinoSDC_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(ArduinoSDC_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
/*
void CANopen(void){
	txMessage.frame.id=0x80;		//0x81
	txMessage.frame.idType=0x00;
	txMessage.frame.dlc=8;
	txMessage.frame.data0=0x00;
	txMessage.frame.data1=0x00;
	txMessage.frame.data2=0x00;
	txMessage.frame.data3=0x00;
	txMessage.frame.data4=0x00;
	txMessage.frame.data5=0x00;
	txMessage.frame.data6=0x00;
	txMessage.frame.data7=0x00;
	CANSPI_Transmit(&txMessage);
	HAL_Delay(50);

	if(CANSPI_Receive(&rxMessage))
	{
		uint16_t RPM_1= ((uint16_t)rxMessage.frame.data1 << 8) | rxMessage.frame.data0;
		uint16_t RPM_2= ((uint16_t)rxMessage.frame.data3 << 8) | rxMessage.frame.data2;
		RPM=	((uint32_t)RPM_2 << 16) | RPM_1;

		// if(torque_buff!=0)PrintString("torque is not zero!\r\n");
		// TX torque doesn't work
		torque= ((uint16_t)rxMessage.frame.data7 << 8) | rxMessage.frame.data6;
		temp= ((uint16_t)rxMessage.frame.data5 << 8) | rxMessage.frame.data4;
		//   	uint16_t torque_buff= ((uint16_t)rxMessage.frame.data5 << 8) | rxMessage.frame.data4;
		//		uint16_t temp_buff= ((uint16_t)rxMessage.frame.data1 << 8) | rxMessage.frame.data0;

		sprintf(Rbuff,"RPM: %ld\r\n",RPM);
		sprintf(Tbuff,"Torque: %.2f \r\n",(float)torque*0.1);
		sprintf(TempBuff,"Temp:%.2f \r\n",temp);

		HAL_UART_Transmit(&huart2, (uint8_t*)Rbuff, sizeof(Rbuff), 100);
		HAL_UART_Transmit(&huart2, (uint8_t*)TempBuff, sizeof(TempBuff), 100);
		HAL_UART_Transmit(&huart2, (uint8_t*)Tbuff, sizeof(Tbuff), 100);

		if(RPM>=1000){
			HAL_UART_Transmit(&huart2, (uint8_t*)"RPM : warning\r\n", sizeof("RPM : warning\r\n"), 100);
		}
		else{
			HAL_UART_Transmit(&huart2, (uint8_t*)"RPM : normal\r\n", sizeof("RPM : normal\r\n"), 100);
		}

		if(temp>=80){
			HAL_UART_Transmit(&huart2, (uint8_t*)"Temperature : over-heat\r\n", sizeof("Temperature : over-heat\r\n"), 100);
		}
		else{
			HAL_UART_Transmit(&huart2, (uint8_t*)"Temperature : normal\r\n", sizeof("Temperature : normal\r\n"), 100);
		}
		HAL_UART_Transmit(&huart2, (uint8_t*)"\n", sizeof("\n"), 100);
		HAL_Delay(1000);
	}
//	else{
//		HAL_UART_Transmit(&huart2, (uint8_t*)"No CAN connection\r\n", sizeof("No CAN connection\r\n"), 100);
//	}
}
 */

void ADC1_Select_CH0(void)
{
	ADC_ChannelConfTypeDef sConfig={0};
	sConfig.Channel = ADC_CHANNEL_0;
	sConfig.Rank = 1;
	sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	{
		Error_Handler();
	}
}

void ADC1_Select_CH9(void)
{
	ADC_ChannelConfTypeDef sConfig={0};
	sConfig.Channel = ADC_CHANNEL_9;
	sConfig.Rank = 1;
	sConfig.SamplingTime = ADC_SAMPLETIME_15CYCLES;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	{
		Error_Handler();
	}
}

float ADCtoTemp(float ADCval){

//	float R = (50*100-10*100*ADCval)/ADCval;
	float R = 10000*(4096/ADCval -1);
//	float T = 4073*298/(4073+298*log(R/10000));
	float T = 1/(1/298.15+1/4073*log(R/10000));
	T -= 273.15;
//	return T;

	if (T>0&&T<80) return T;
	else return 999;
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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
