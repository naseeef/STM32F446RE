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
	uint8_t      *pTxBuffer;    /* Store application Tx buffer address                     */
    uint8_t      *pRxBuffer;    /* Store application Rx buffer address                     */
    uint32_t     TxLen;         /* Store Tx length                                         */
    uint32_t     RxLen;         /* Store Rx length                                         */
    uint32_t     TxState;       /* Store Tx state                                          */
    uint32_t     RxState;       /* Store Rx state                                          */
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

/*
 * Possible SPI Application states
 */
#define SPI_READY           0
#define SPI_BUSY_IN_RX      1
#define SPI_BUSY_IN_TX      2

/*
 * Possible SPI Application events
 */
#define SPI_EVENT_TX_CMPLT      1
#define SPI_EVENT_RX_CMPLT      2
#define SPI_EVENT_OVR_ERR       3
#define SPI_EVENT_CRC_ERR       4

/*APIs*/
void SPIPeripheralClockControl(spi_reg_t *pSPIx, uint8_t ENorDI);
void SPIInit(spi_handle_t *pSPIHandle);
void SPISendData(spi_reg_t *pSPIx, uint8_t *pTxBuffer, uint8_t len);
void SPIReceiveData(spi_reg_t *pSPIx, uint8_t *pTxBuffer, uint8_t len);
uint8_t SPIGetFlagStatus(spi_reg_t *pSPIx, uint32_t flagName);
void SPIPeripheralControl(spi_reg_t *pSPIx, uint8_t ENorDI);
void SPISSIConfig(spi_reg_t *pSPIx, uint8_t ENorDI);
void SPISSOEConfig(spi_reg_t *pSPIx, uint8_t ENorDI);
/*Interrupt Based APIs*/
uint8_t SPIReceiveDataInterrupt(spi_handle_t *pSPIHandle, uint8_t *pRxBuffer, uint32_t Length);
uint8_t SPISendDataInterrupt(spi_handle_t *pSPIHandle, uint8_t *pTxBuffer, uint32_t Length);
void SPIIRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi);
void SPIIRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);
void SPIIRQHandling(spi_handle_t *pSPIHandle);
void SPICloseTransmission(spi_handle_t *pSPIHandle);
void SPICloseReception(spi_handle_t *pSPIHandle);
void SPIClearOVRFlag(spi_reg_t *pSPIx);


#endif /* INC_STM32F446RE_SPI_DRIVER_H_ */
