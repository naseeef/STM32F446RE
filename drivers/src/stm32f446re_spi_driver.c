
/*
 * stm32f466re_spi_driver.c
 *
 *  Created on: 17-Jun-2023
 *      Author: Naseef Azhikulangara
 */
#include <stm32f446re_spi_driver.h>




/**********************************************************************************
 * @fn			- SPIPeripheralClockControl(SPI_RegDef_t *pSPIx, uint8_t ENorDI)
 *
 * @brief		- SPI Peripheral clock enable or disable
 *
 * @param[]		- SPI Port
 * @param[]		- Enable or Disable
 *
 * @return		-
 *
 * @Note		-
 *
 */
void SPIPeripheralClockControl(spi_reg_t *pSPIx, uint8_t ENorDI)
{
	if (ENorDI == ENABLE)
	{
		if(pSPIx == SPI1)
		{
			SPI1_PCLK_EN();
		}else if(pSPIx == SPI2)
		{
			SPI2_PCLK_EN();
		}else if(pSPIx == SPI3)
		{
			SPI3_PCLK_EN();
		}else if(pSPIx == SPI4)
		{
			SPI4_PCLK_EN();
		}
	}
	else
	{
		if(pSPIx == SPI1)
		{
			SPI1_PCLK_DI();
		}else if(pSPIx == SPI2)
		{
			SPI2_PCLK_DI();
		}else if(pSPIx == SPI3)
		{
			SPI3_PCLK_DI();
		}else if(pSPIx == SPI4)
		{
			SPI4_PCLK_DI();
		}
	}
}

/**********************************************************************************
 * @fn			- SPIInit(spi_reg_t pSPIx)
 *
 * @brief		- SPI Initialization for a SPI port
 *
 * @param[]		- SPI Port
 * @param[]		-
 *
 * @return		-
 *
 * @Note		-
 *
 */
void SPIInit(spi_handle_t *pSPIHandle)
{
	uint32_t temp = 0;

	/*Enabling Peripheral Clock*/
	SPIPeripheralClockControl(pSPIHandle->pSPIx, ENABLE);

	/*Configuring SPI_CR1 Register*/
	/*Setting up device mode*/
	temp |= pSPIHandle->spiConfig.spiDeviceMode << SPI_CR1_MSTR;

	/*Setting bus configuration*/
	if(pSPIHandle->spiConfig.spiBusConfig == SPI_BUS_CONFIG_FD)
	{
		//Clear the Bi-Di mode
		temp &= ~(1 << SPI_CR1_BIDIMODE);
	}
	else if(pSPIHandle->spiConfig.spiBusConfig == SPI_BUS_CONFIG_HD)
	{
		//Set the Bi-Di mode
		temp |= (1 << SPI_CR1_BIDIMODE);
	}
	else if(pSPIHandle->spiConfig.spiBusConfig == SPI_BUS_CONFIG_RX)
	{
		//Clear the Bi-Di mode
		temp &= ~(1 << SPI_CR1_BIDIMODE);
		//Set the RXONLY mode
		temp |= (1 << SPI_CR1_RXONLY);
	}

	/*Setting clock speed*/
	temp |= pSPIHandle->spiConfig.spiSclkSpeed << SPI_CR1_BR;

	/*Setting data frame format*/
	temp |= pSPIHandle->spiConfig.spiDFF << SPI_CR1_DFF;

	/*Setting CPOL & CPHA*/
	temp |= pSPIHandle->spiConfig.spiCPOL << SPI_CR1_CPOL;
	temp |= pSPIHandle->spiConfig.spiCPHA << SPI_CR1_CPHA;

	/*Setting Software Slave Management*/
	temp |= pSPIHandle->spiConfig.spiSSM << SPI_CR1_SSM;

	/*Setting SSI, to avoid MODF Error*/
	temp |= pSPIHandle->spiConfig.spiSSI << SPI_CR1_SSI;

	pSPIHandle->pSPIx->CR1 = temp;
}

/**********************************************************************************
 * @fn			- SPIPeripheralControl(spi_reg_t *pSPIx, uint8_t ENorDI)
 *
 * @brief		- SPI Peripheral enable or disable
 *
 * @param[]		- SPI Port
 * @param[]		- Enable/Disable
 *
 * @return		-
 *
 * @Note		- SPI CR1 will not take anything if it's not enabled
 *
 */
void SPIPeripheralControl(spi_reg_t *pSPIx, uint8_t ENorDI)
{
	if(ENorDI == ENABLE)
	{
		pSPIx->CR1 |= ( 1 << SPI_CR1_SPE );
	}
	else
	{
		pSPIx->CR1 &= ~( 1 << SPI_CR1_SPE );
	}
}

/**********************************************************************************
 * @fn			- SPISSIConfig(spi_reg_t *pSPIx, uint8_t ENorDI)
 *
 * @brief		- SPI Peripheral enable or disable
 *
 * @param[]		- SPI Port
 * @param[]		- Enable/Disable
 *
 * @return		-
 *
 * @Note		- to prevent MODF error
 *
 */
void SPISSIConfig(spi_reg_t *pSPIx, uint8_t ENorDI)
{
	if(ENorDI == ENABLE)
	{
		pSPIx->CR1 |= ( 1 << SPI_CR1_SSI );
	}
	else
	{
		pSPIx->CR1 &= ~( 1 << SPI_CR1_SSI );
	}
}

/**********************************************************************************
 * @fn			- SPISSOEConfig(spi_reg_t *pSPIx, uint8_t ENorDI)
 *
 * @brief		- SPI Peripheral enable or disable
 *
 * @param[]		- SPI Port
 * @param[]		- Enable/Disable
 *
 * @return		-
 *
 * @Note		- for Hardware management SSOE configuration
 *
 */
void SPISSOEConfig(spi_reg_t *pSPIx, uint8_t ENorDI)
{
	if(ENorDI == ENABLE)
	{
		pSPIx->CR2 |= ( 1 << SPI_CR2_SSOE );
	}
	else
	{
		pSPIx->CR2 &= ~( 1 << SPI_CR2_SSOE );
	}
}

/**********************************************************************************
 * @fn			- SPIDeInit(spi_reg_t *pSPIx)
 *
 * @brief		- SPI De-Initialization for a SPI port
 *
 * @param[]		- SPI Port
 * @param[]		-
 *
 * @return		-
 *
 * @Note		-
 *
 */
void SPIDeInit(spi_reg_t *pSPIx)
{
	if(pSPIx == SPI1)
	{
		SPI1_REG_RESET();
	}
	else if(pSPIx == SPI2)
	{
		SPI2_REG_RESET();
	}
	else if(pSPIx == SPI3)
	{
		SPI3_REG_RESET();
	}
	else if(pSPIx == SPI4)
	{
		SPI4_REG_RESET();
	}
}

/**********************************************************************************
 * @fn			- SPISendData(spi_reg_t *pSPIx, uint8_t *pTxBuffer, uint8_t len)
 *
 * @brief		- SPI Send data through a SPI port
 *
 * @param[]		- SPI Port
 * @param[]		- Transmit Buffer - Data
 * @param[]		- len of data
 * 
 * @return		-
 *
 * @Note		- THIS IS A BLOCKING FUNCTION CALL (POLLING TYPE)
 *
 */
void SPISendData(spi_reg_t *pSPIx, uint8_t *pTxBuffer, uint8_t len)
{
	while(len>0)
	{
		/*Wait until SPI SR TXE flag is empty*/
		while(SPIGetFlagStatus(pSPIx, SPI_TXE_FLAG) == FLAG_RESET);

		/*Check the Data Frame Format in SPI_CR1*/
		if((pSPIx->CR1 & ( 1<< SPI_CR1_DFF)))
		{
			//16 Bit DFF

			/*Load the data to SPI DR (Data register)*/
			/*typecaste into uint16_t becuase we getting 8 bit data*/
			pSPIx->DR = *((uint16_t*)pTxBuffer); 
			len--;
			len--;
			/*increment*/
			(uint16_t*)pTxBuffer++;
		}
		else
		{
			//8 Bit DFF

			/*Load the data to SPI DR (Data register)*/
			pSPIx->DR = *pTxBuffer; //Becuase we getting 8 bit data
			len--;
			/*increment*/
			pTxBuffer++;
		}
	}
}

/**********************************************************************************
 * @fn			- SPIReceiveData(spi_reg_t *pSPIx, uint8_t *pTxBuffer, uint8_t len)
 *
 * @brief		- SPI Send data through a SPI port
 *
 * @param[]		- SPI Port
 * @param[]		- Receiver Buffer - Data
 * @param[]		- len of data
 * 
 * @return		-
 *
 * @Note		- THIS IS A BLOCKING FUNCTION CALL (POLLING TYPE)
 *
 */
void SPIReceiveData(spi_reg_t *pSPIx, uint8_t *pRxBuffer, uint8_t len)
{
	while(len>0)
	{
		/*Wait until SPI SR RXNE flag is empty*/
		while(SPIGetFlagStatus(pSPIx, SPI_RXNE_FLAG) == FLAG_RESET);

		/*Check the Data Frame Format in SPI_CR1*/
		if((pSPIx->CR1 & ( 1<< SPI_CR1_DFF)))
		{
			//16 Bit DFF

			/*Load the SPI DR (Data register) to RxBuffer*/
			/*typecaste into uint16_t becuase we getting 8 bit data*/
			*((uint16_t*)pRxBuffer) = pSPIx->DR; 
			len--;
			len--;
			/*increment*/
			(uint16_t*)pRxBuffer++;
		}
		else
		{
			//8 Bit DFF

			/*Load the SPI DR (Data register) to RxBuffer*/
			*pRxBuffer = pSPIx->DR; //Becuase we getting 8 bit data
			len--;
			/*increment*/
			pRxBuffer++;
		}
	}
}

uint8_t SPIGetFlagStatus(spi_reg_t *pSPIx, uint32_t flagName)
{
	if(pSPIx->SR & flagName)
	{
		return FLAG_SET;
	}
	else
	{
		return FLAG_RESET;
	}
}
