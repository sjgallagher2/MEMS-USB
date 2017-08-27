/*
 * L3GD20.c
 *
 *  Created on: June 3, 2017
 *      Author: Sam Gallagher
 */

#include "L3GD20.h"
/** @addtogroup Utilities
  * @{
  */

/** @addtogroup STM32F401_DISCOVERY
  * @{
  */

/** @addtogroup STM32F401_DISCOVERY_L3GD20
  * @{
  */


/** @defgroup STM32F401_DISCOVERY_L3GD20_Private_TypesDefinitions
  * @{
  */

/**
  * @}
  */

/** @defgroup STM32F401_DISCOVERY_L3GD20_Private_Defines
  * @{
  */

/**
  * @}
  */

/** @defgroup STM32F401_DISCOVERY_L3GD20_Private_Macros
  * @{
  */

/**
  * @}
  */

/** @defgroup STM32F401_DISCOVERY_L3GD20_Private_Variables
  * @{
  */
__IO uint32_t  L3GD20Timeout = L3GD20_FLAG_TIMEOUT;
SPI_HandleTypeDef hspi1;
/**
  * @}
  */

/** @defgroup STM32F401_DISCOVERY_L3GD20_Private_FunctionPrototypes
  * @{
  */
//static uint8_t L3GD20_SendByte(uint8_t byte);
static void L3GD20_LowLevel_Init(void);
/**
  * @}
  */

/** @defgroup STM32F401_DISCOVERY_L3GD20_Private_Functions
  * @{
  */

/**
  * @brief  Set L3GD20 Initialization.
  * @param  L3GD20_InitStruct: pointer to a L3GD20_InitTypeDef structure
  *         that contains the configuration setting for the L3GD20.
  * @retval None
  */
void L3GD20_Init(L3GD20_InitTypeDef *L3GD20_InitStruct)
{
  uint8_t ctrl1 = 0x00, ctrl4 = 0x00;

  /* Configure the low level interface ---------------------------------------*/
  L3GD20_LowLevel_Init();

  /* Configure MEMS: data rate, power mode, full scale and axes */
  ctrl1 |= (uint8_t) (L3GD20_InitStruct->Power_Mode | L3GD20_InitStruct->Output_DataRate | \
                    L3GD20_InitStruct->Axes_Enable | L3GD20_InitStruct->Band_Width);

  ctrl4 |= (uint8_t) (L3GD20_InitStruct->BlockData_Update | L3GD20_InitStruct->Endianness | \
                    L3GD20_InitStruct->Full_Scale);

  /* Write value to MEMS CTRL_REG1 regsister */
  L3GD20_Write(&ctrl1, L3GD20_CTRL_REG1_ADDR);

  /* Write value to MEMS CTRL_REG4 regsister */
  L3GD20_Write(&ctrl4, L3GD20_CTRL_REG4_ADDR);
}

/**
 * @brief	Assign the SPI handle (if initialization was done outside
 * @param	SPI handler pointer
 * @retval	None
 */
void L3GD20_SetSPI(SPI_HandleTypeDef h)
{
	hspi1 = h;
}

/**
  * @brief  Reboot memory content of L3GD20
  * @param  None
  * @retval None
  */
void L3GD20_RebootCmd(void)
{
  uint8_t tmpreg;

  /* Read CTRL_REG5 register */
  L3GD20_Read(&tmpreg, L3GD20_CTRL_REG5_ADDR, 1);

  /* Enable or Disable the reboot memory */
  tmpreg |= L3GD20_BOOT_REBOOTMEMORY;

  /* Write value to MEMS CTRL_REG5 regsister */
  L3GD20_Write(&tmpreg, L3GD20_CTRL_REG5_ADDR);
}

/**
  * @brief Set L3GD20 Interrupt configuration
  * @param  L3GD20_InterruptConfig_TypeDef: pointer to a L3GD20_InterruptConfig_TypeDef
  *         structure that contains the configuration setting for the L3GD20 Interrupt.
  * @retval None
  */
void L3GD20_INT1InterruptConfig(L3GD20_InterruptConfigTypeDef *L3GD20_IntConfigStruct)
{
  uint8_t ctrl_cfr = 0x00, ctrl3 = 0x00;

  /* Read INT1_CFG register */
  L3GD20_Read(&ctrl_cfr, L3GD20_INT1_CFG_ADDR, 1);

  /* Read CTRL_REG3 register */
  L3GD20_Read(&ctrl3, L3GD20_CTRL_REG3_ADDR, 1);

  ctrl_cfr &= 0x80;

  ctrl3 &= 0xDF;

  /* Configure latch Interrupt request and axe interrupts */
  ctrl_cfr |= (uint8_t)(L3GD20_IntConfigStruct->Latch_Request| \
                   L3GD20_IntConfigStruct->Interrupt_Axes);

  ctrl3 |= (uint8_t)(L3GD20_IntConfigStruct->Interrupt_ActiveEdge);

  /* Write value to MEMS INT1_CFG register */
  L3GD20_Write(&ctrl_cfr, L3GD20_INT1_CFG_ADDR);

  /* Write value to MEMS CTRL_REG3 register */
  L3GD20_Write(&ctrl3, L3GD20_CTRL_REG3_ADDR);
}

/**
  * @brief  Enable or disable INT1 interrupt
  * @param  InterruptState: State of INT1 Interrupt
  *      This parameter can be:
  *        @arg L3GD20_INT1INTERRUPT_DISABLE
  *        @arg L3GD20_INT1INTERRUPT_ENABLE
  * @retval None
  */
void L3GD20_INT1InterruptCmd(uint8_t InterruptState)
{
  uint8_t tmpreg;

  /* Read CTRL_REG3 register */
  L3GD20_Read(&tmpreg, L3GD20_CTRL_REG3_ADDR, 1);

  tmpreg &= 0x7F;
  tmpreg |= InterruptState;

  /* Write value to MEMS CTRL_REG3 regsister */
  L3GD20_Write(&tmpreg, L3GD20_CTRL_REG3_ADDR);
}

/**
  * @brief  Enable or disable INT2 interrupt
  * @param  InterruptState: State of INT1 Interrupt
  *      This parameter can be:
  *        @arg L3GD20_INT2INTERRUPT_DISABLE
  *        @arg L3GD20_INT2INTERRUPT_ENABLE
  * @retval None
  */
void L3GD20_INT2InterruptCmd(uint8_t InterruptState)
{
  uint8_t tmpreg;

  /* Read CTRL_REG3 register */
  L3GD20_Read(&tmpreg, L3GD20_CTRL_REG3_ADDR, 1);

  tmpreg &= 0xF7;
  tmpreg |= InterruptState;

  /* Write value to MEMS CTRL_REG3 regsister */
  L3GD20_Write(&tmpreg, L3GD20_CTRL_REG3_ADDR);
}

/**
  * @brief  Set High Pass Filter Modality
  * @param  L3GD20_FilterStruct: pointer to a L3GD20_FilterConfigTypeDef structure
  *         that contains the configuration setting for the L3GD20.
  * @retval None
  */
void L3GD20_FilterConfig(L3GD20_FilterConfigTypeDef *L3GD20_FilterStruct)
{
  uint8_t tmpreg;

  /* Read CTRL_REG2 register */
  L3GD20_Read(&tmpreg, L3GD20_CTRL_REG2_ADDR, 1);

  tmpreg &= 0xC0;

  /* Configure MEMS: mode and cutoff frquency */
  tmpreg |= (uint8_t) (L3GD20_FilterStruct->HighPassFilter_Mode_Selection |\
                      L3GD20_FilterStruct->HighPassFilter_CutOff_Frequency);

  /* Write value to MEMS CTRL_REG2 regsister */
  L3GD20_Write(&tmpreg, L3GD20_CTRL_REG2_ADDR);
}

/**
  * @brief  Enable or Disable High Pass Filter
  * @param  HighPassFilterState: new state of the High Pass Filter feature.
  *      This parameter can be:
  *         @arg: L3GD20_HIGHPASSFILTER_DISABLE
  *         @arg: L3GD20_HIGHPASSFILTER_ENABLE
  * @retval None
  */
void L3GD20_FilterCmd(uint8_t HighPassFilterState)
{
  uint8_t tmpreg;

  /* Read CTRL_REG5 register */
  L3GD20_Read(&tmpreg, L3GD20_CTRL_REG5_ADDR, 1);

  tmpreg &= 0xEF;

  tmpreg |= HighPassFilterState;

  /* Write value to MEMS CTRL_REG5 regsister */
  L3GD20_Write(&tmpreg, L3GD20_CTRL_REG5_ADDR);
}

/**
  * @brief  Get status for L3GD20 data
  * @param  None
  * @retval Data status in a L3GD20 Data
  */
uint8_t L3GD20_GetDataStatus(void)
{
  uint8_t tmpreg;

  /* Read STATUS_REG register */
  L3GD20_Read(&tmpreg, L3GD20_STATUS_REG_ADDR, 1);

  return tmpreg;
}

/**
  * @brief  Writes one byte to the L3GD20.
  * @param  pBuffer : the data to be written to the L3GD20.
  * @param  WriteAddr : L3GD20's internal address to write to.
  * @retval None
  */
void L3GD20_Write(uint8_t buffer, uint8_t WriteAddr)
{
  /* Chip select (CS) to begin */
  L3GD20_CS_LOW();

  uint8_t cmd[2] = {0x00 | WriteAddr, buffer};
  HAL_SPI_Transmit(&hspi1,cmd,2,L3GD20_FLAG_TIMEOUT);

  L3GD20_CS_HIGH();
}

/**
  * @brief  Reads a block of data from the L3GD20.
  * @param  pBuffer : pointer to the buffer that receives the data read from the L3GD20.
  * @param  ReadAddr : L3GD20's internal address to read from.
  * @param  NumByteToRead : number of bytes to read from the L3GD20.
  * @retval None
  */
void L3GD20_Read(uint8_t* pBuffer,uint8_t ReadAddr, uint16_t NumByteToRead)
{
  /* Chip select (CS) to begin */
  L3GD20_CS_LOW();

  /* Format HAL command */
  uint8_t cmd[1+NumByteToRead];// = {READWRITE_CMD | WriteAddr, pBuffer[0]};
  cmd[0] = READWRITE_CMD | MULTIPLEBYTE_CMD | ReadAddr;
  for(int i = 1; i <= NumByteToRead; i++)
	  cmd[i] = DUMMY_BYTE;
  /* Transmit/Receive over SPI */
  HAL_SPI_TransmitReceive(&hspi1,cmd,pBuffer,NumByteToRead+1,L3GD20_FLAG_TIMEOUT);

  L3GD20_CS_HIGH();
}

/**
 * @brief Reads the x-axis angular rate data and returns it as a float
 * @param None
 * @retval X-axis angular rate
 */
float L3GD20_GetAngularRateX(float sensitivity)
{
	float ang = 0;
	uint8_t x_buffer[3] = {0,0,0};
	int16_t xdata_raw = 0;

	L3GD20_Read(x_buffer, L3GD20_OUT_X_L_ADDR, 2);

	xdata_raw = (int16_t)( (uint16_t)(x_buffer[2] << 8) + x_buffer[1] );
	ang = (float)xdata_raw/sensitivity;
	return ang;
}

/**
 * @brief Reads the y-axis angular rate data and returns it as a float
 * @param None
 * @retval Y-axis angular rate
 */
float L3GD20_GetAngularRateY(float sensitivity)
{
	float ang = 0;
	uint8_t y_buffer[3];
	int16_t ydata_raw = 0;

	L3GD20_Read(y_buffer, L3GD20_OUT_Y_L_ADDR, 2);

	ydata_raw = (int16_t)( (uint16_t)(y_buffer[2] << 8) + y_buffer[1] );
	ang = (float)ydata_raw/sensitivity;

	return ang;
}


/**
 * @brief Reads the z-axis angular rate data and returns it as a float
 * @param None
 * @retval Z-axis angular rate
 */
float L3GD20_GetAngularRateZ(float sensitivity)
{
	float ang = 0;
	uint8_t z_buffer[3];
	int16_t zdata_raw = 0;

	L3GD20_Read(z_buffer, L3GD20_OUT_Z_L_ADDR, 2);

	zdata_raw = (int16_t)( (uint16_t)(z_buffer[2] << 8) + z_buffer[1] );
	ang = (float)zdata_raw/sensitivity;

	return ang;
}


/**
  * @brief  Initializes the low level interface used to drive the L3GD20
  * @param  None
  * @retval None
  */
static void L3GD20_LowLevel_Init(void)
{
	hspi1.Instance = SPI1;
	hspi1.Init.Mode = SPI_MODE_MASTER;
	hspi1.Init.Direction = SPI_DIRECTION_2LINES;
	hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
	hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
	hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
	hspi1.Init.NSS = SPI_NSS_SOFT;
	hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
	hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
	hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
	hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
	hspi1.Init.CRCPolynomial = 10;
	if (HAL_SPI_Init(&hspi1) != HAL_OK)
	{
	Error_Handler();
	}
}

#ifdef USE_DEFAULT_TIMEOUT_CALLBACK
/**
  * @brief  Basic management of the timeout situation.
  * @param  None.
  * @retval None.
  */
uint32_t L3GD20_TIMEOUT_UserCallback(void)
{
  /* Block communication and all processes */
  while (1)
  {
  }
}
#endif /* USE_DEFAULT_TIMEOUT_CALLBACK */

/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
