/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body, reads gyroscope data and transmits over USB
  ******************************************************************************
  *
  * Copyright (c) 2017 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/*
 * @periph
 * USB connection in CDC (communication device class) mode for virtual comm port. Connects on
 * my system as /dev/ttyACM0, connect with stty -F /dev/ttyACM0 then view with CuteCom or
 * similar
 *
 * MEMS gyroscope L3GD20H on SPI 1, reading x, y, and z angular rates. The device operates
 * through SPI, with the following registers:
 * 		- WHO_AM_I		0x0F		000 1111		Returns device ID 11010111 (0xD7)
 * 		- CTRL1			0x20		010 0000		Controls data rate, bandwidth, power mode, axis enables
 * 		- CTRL2			0x21		010 0001		Trigger and digital filtering modes
 * 		- CTRL3			0x22		010 0010		Interrupts, data ready enable
 * 		- CTRL4			0x23		010 0011		Data formats, block update, self test, SPI interface mode
 * 		- CTRL5			0x24		010 0100		Filter enable, FIFO enable, sensing chain
 * 		- REFERENCE		0x25		010 0101		Digital high pass filter reference value
 * 		- OUT_TEMP		0x26		010 0110		Temperature data (2's complement)
 * 		- STATUS		0x27		010 0111		Data overruns, new data available
 * 		- OUT_X_L		0x28		010 1000		16-bit block of angular rate data for x,y, and z
 * 		- OUT_X_H		0x29		010 1001		|
 * 		- OUT_Y_L		0x2A		010 1010		|
 * 		- OUT_Y_H		0x2B		010 1011		|
 * 		- OUT_Z_L		0x2C		010 1100		|
 * 		- OUT_Z_H		0x2D		010 1101		|
 * 		- FIFO_CTRL		0x2E		010 1110		FIFO mode selection
 * 		- FIFO_SRC		0x2F		010 1111		FIFO threshold, overrun bit status
 * 		- IG_CFG		0x30		011 0000
 * 		- IG_SRC		0x31		011 0001
 * 		- IG_THS_XH		0x32		011 0010
 * 		- IG_THS_XL		0x33		011 0011
 * 		- IG_THS_YH		0x34		011 0100
 * 		- IG_THS_YL		0x35		011 0101
 * 		- IG_THS_ZH		0x36		011 0110
 * 		- IG_THS_ZL		0x37		011 0111
 * 		- IG_DURATION	0x38		011 1000
 * 		- LOW_ODR		0x39		011 1001
 * See L3GD20 datasheet for register details and more
 * Operating baud rate: 3.125 MHz (10MHz max)
 * To read, send the following byte:
 * 	bit 0		Read/write (read = 1)
 * 	bit 1		Increment register address in multiple readings
 * 	bit 2-7		Address of the indexed register
 * Data is returned MSB first
 *
 * @pinout
 * SPI 1, for the STM32F401C Discovery board, is connected to the L3GD20H with the
 * following pinout:
 * 		- PA5		SCK
 * 		- PA6		SDO/MOSI
 * 		- PA7		SDI/MISO
 * 		- PE0		Interrupt 1
 * 		- PE1		Interrupt 2
 * 		- PE3		!CS
 *
 */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_hal.h"
#include "usbd_cdc_if.h"
#include "usb_device.h"
#include "L3GD20.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspiloc;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void Error_Handler(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
void getInt16Str(int16_t d, char * data_str);
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USB_DEVICE_Init();
  MX_SPI1_Init();
  L3GD20_SetSPI(hspiloc); //Set the SPI handler

  /* USER CODE BEGIN 2 */
  //Initialize gyroscope data buffer
  uint8_t whoami_buf[2];//whoami buffer
  uint8_t ctrl1_cmd = ((uint8_t)0xF);//Data rate = 100Hz, BW = 12.5Hz
  uint8_t readxyz_cmd[7] = {READWRITE_CMD | MULTIPLEBYTE_CMD | L3GD20_OUT_X_L_ADDR, DUMMY_BYTE, DUMMY_BYTE, DUMMY_BYTE, DUMMY_BYTE, DUMMY_BYTE, DUMMY_BYTE};
  uint8_t startbyte[5] = {'s','t','a','r','t'};
  float sensitivity_250 = 114.285f;

  //Initialize clock select (!CS) to HIGH
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_SET);

  //Initialize control registers in L3GD20=========
  //Check for Who Am I, should turn on green LED
  L3GD20_Read(whoami_buf, L3GD20_WHO_AM_I_ADDR,1);
  if(whoami_buf[1] == I_AM_L3GD20)
	  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_SET);

  //Set control register 1
  L3GD20_Write(ctrl1_cmd,L3GD20_CTRL_REG1_ADDR);

  //Axis rate data
  int16_t xdata = 0;
  int16_t ydata = 0;
  int16_t zdata = 0;
  uint8_t send = 1; //Send or don't send flag

  volatile char xdata_str[6] = {'0','0','0','0','0','0'};
  volatile char ydata_str[6] = {'0','0','0','0','0','0'};
  volatile char zdata_str[6] = {'0','0','0','0','0','0'};

  volatile uint8_t rxData;
  uint32_t lRxData = 1;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
	  lRxData = 1;
	  CDC_Receive_FS(&rxData, &lRxData);
	  if(rxData == 'x')
	  {
		  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_RESET);
		  xdata = (int16_t)L3GD20_GetAngularRateX(1); //No sensitivity adjustment
		  getInt16Str(xdata,xdata_str);
		  CDC_Transmit_FS(xdata_str,6);
		  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_SET);
		  HAL_Delay(1);
	  }
	  if(rxData == 'y')
	  {
		  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_RESET);
		  ydata = (int16_t)L3GD20_GetAngularRateY(1); //No sensitivity adjustment
		  getInt16Str(ydata,ydata_str);
		  CDC_Transmit_FS(ydata_str,6);
		  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_SET);
		  HAL_Delay(1);
	  }
	  if(rxData == 'z')
	  {
		  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_RESET);
		  zdata = (int16_t)L3GD20_GetAngularRateZ(1); //No sensitivity adjustment
		  getInt16Str(zdata,zdata_str);
		  CDC_Transmit_FS(zdata_str,6);
		  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_SET);
		  HAL_Delay(1);
	  }
  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Configure the main internal regulator output voltage 
    */
  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 72;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 3;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV8;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV8;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* SPI1 init function */
static void MX_SPI1_Init(void)
{

  hspiloc.Instance = SPI1;
  hspiloc.Init.Mode = SPI_MODE_MASTER;
  hspiloc.Init.Direction = SPI_DIRECTION_2LINES;
  hspiloc.Init.DataSize = SPI_DATASIZE_8BIT;
  hspiloc.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspiloc.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspiloc.Init.NSS = SPI_NSS_SOFT;
  hspiloc.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  hspiloc.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspiloc.Init.TIMode = SPI_TIMODE_DISABLE;
  hspiloc.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspiloc.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspiloc) != HAL_OK)
  {
    Error_Handler();
  }

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_SET);

  /*Configure GPIO pin : PE3 (CS)*/
  GPIO_InitStruct.Pin = GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : PE0 PE1 (INT) */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : PD12 (LED)*/
  GPIO_InitStruct.Pin = GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

void getInt16Str(int16_t d, char * data_str)
{
	//Max number is 32768 (16 bit), 5 digits and sign (+/-)
	//Instead of processing binary data in Octave we'll send a string
	//Strip the bits, +48 for ASCII 0 - 9, manually to avoid pow()
	if(d > 0)
	{
		data_str[5] = d%10+48;
		data_str[4] = (d%100 - d%10)/10 + 48;
		data_str[3] = (d%1000 - d%100)/100 + 48;
		data_str[2] = (d%10000 - d%1000)/1000 + 48;
		data_str[1] = (d%100000 - d%10000)/10000 + 48;
		data_str[0] = '+';
	}
	else if(d < 0)
	{
		d = -d;
		data_str[5] = d%10+48;
		data_str[4] = (d%100 - d%10)/10 + 48;
		data_str[3] = (d%1000 - d%100)/100 + 48;
		data_str[2] = (d%10000 - d%1000)/1000 + 48;
		data_str[1] = (d%100000 - d%10000)/10000 + 48;
		data_str[0] = '-';
	}
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler */ 
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
