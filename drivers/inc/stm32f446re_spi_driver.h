/*
 * stm32f466re_spi_driver.h
 *
 *  Created on: 17-Jun-2023
 *      Author: Naseef Azhikulangara
 */


#ifndef INC_STM32F446RE_SPI_DRIVER_H_
#define INC_STM32F446RE_SPI_DRIVER_H_

#include <stm32f446re.h>

typedef struct{
	uint8_t spiDeviceMode;
	uint8_t spiBusConfig;
	uint8_t spiSclkSpeed;
	uint8_t spiDFF;
	uint8_t spiCPOL;
	uint8_t spiCPHA;
	uint8_t spiSSM;
	uint8_t spiSSI;
}spi_config_t;

typedef struct{
	spi_reg_t *pSPIx;
	spi_config_t spiConfig;
}spi_handle_t;

/*
 * Device Modes
 */
#define SPI_DEVICE_MODE_MASTER	1
#define SPI_DEVICE_MODE_SLAVE	0

/*
 * Bus Configurations
 */
#define SPI_BUS_CONFIG_FD		1
#define SPI_BUS_CONFIG_HD		2
#define SPI_BUS_CONFIG_RX		3

/*
 * Clock Speed
 */
#define SPI_SCLK_SPEED_DIV2		0
#define SPI_SCLK_SPEED_DIV4		1
#define SPI_SCLK_SPEED_DIV8		2
#define SPI_SCLK_SPEED_DIV16	3
#define SPI_SCLK_SPEED_DIV32	4
#define SPI_SCLK_SPEED_DIV64	5
#define SPI_SCLK_SPEED_DIV128	6
#define SPI_SCLK_SPEED_DIV256	7

/*
 * DFF - Data Frame Format
 */
#define SPI_DFF_8BIT			0
#define SPI_DFF_16BIT			1

/*
 * Clock Polarity CPOL
 */
#define SPI_CPOL_HIGH 			1
#define SPI_CPOL_LOW 			0

/*
 * Clock Phase CPHA
 */
#define SPI_CPHA_HIGH 			1
#define SPI_CPHA_LOW 			0

/*
 * SSM - Software Slave Management
 */
#define SPI_SSM_EN				1
#define SPI_SSM_DI				0

/*
 * SSI - Software Slave Management
 */
#define SPI_SSI_EN				1
#define SPI_SSI_DI				0

/*
 * SPI SR (Status Registers) Flags
 */
#define SPI_TXE_FLAG			( 1 << SPI_SR_TXE )
#define SPI_RXNE_FLAG			( 1 << SPI_SR_RXNE )
#define SPI_BSY_FLAG			( 1 << SPI_SR_BSY )

void SPIPeripheralClockControl(spi_reg_t *pSPIx, uint8_t ENorDI);
void SPIInit(spi_handle_t *pSPIHandle);
void SPISendData(spi_reg_t *pSPIx, uint8_t *pTxBuffer, uint8_t len);
uint8_t SPIGetFlagStatus(spi_reg_t *pSPIx, uint32_t flagName);
void SPIPeripheralControl(spi_reg_t *pSPIx, uint8_t ENorDI);
void SPISSIConfig(spi_reg_t *pSPIx, uint8_t ENorDI);
void SPISSOEConfig(spi_reg_t *pSPIx, uint8_t ENorDI);

void SPIReceiveData(spi_reg_t *pSPIx, uint8_t *pTxBuffer, uint8_t len);


#endif /* INC_STM32F446RE_SPI_DRIVER_H_ */
