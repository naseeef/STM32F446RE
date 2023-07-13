
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
void SPIInit(spi_handle_t pSPIHandle)
{
	uint32_t temp = 0;

	/*Configuring SPI_CR1 Register*/

	/*Setting up device mode*/
	temp |= pSPIHandle->spiConfig.spiDeviceMode << SPI_CR1_MSTR;

	/*Setting bus configuration*/
	if(pSPIHandle->spiConfig.spiBusConfig == SPI_BUS_CONFIG_FD)
	{
		//Clear the Bi-Di mode
		temp &= ~(1 << SPI_CR1_BIDIMODE);
	}else if(pSPIHandle->spiConfig.spiBusConfig == SPI_BUS_CONFIG_HD)
	{
		//Set the Bi-Di mode
		temp |= (1 << SPI_CR1_BIDIMODE);
	}else if(pSPIHandle->spiConfig.spiBusConfig == SPI_BUS_CONFIG_RX)
	{
		//Clear the Bi-Di mode
		temp &= ~(1 << SPI_CR1_BIDIMODE);
		//Set the RXONLY mode
		temp |= (1 << SPI_CR1_RXONLY);
	}

	/*Setting clock speed*/
	temp |= pSPIHandle->spiConfig.spiSclkConfig << SPI_CR1_BR;

	/*Setting data frame format*/
	temp |= pSPIHandle->spiConfig.spiDFF << SPI_CR1_DFF;

	/*Setting CPOL & CPHA*/
	temp |= pSPIHandle->spiConfig.spiCPOL << SPI_CR1_CPOL;
	temp |= pSPIHandle->spiConfig.spiCPHA << SPI_CR1_CPHA;

	/*Setting Software Slave Management*/
	/*temp |= pSPIHandle->spiConfig.spiSSM << SPI_CR1_SSM;*/

	pSPIHandle->pSPIx->CR1 = temp;
}
