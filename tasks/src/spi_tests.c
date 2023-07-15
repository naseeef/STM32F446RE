/*
 * spi_tests.c
 *
 *  Created on: 14-Jul-2023
 *  Author: Naseef Azhikulangara
 */

#include <stdint.h>
#include <string.h>

#include <stm32f446re.h>
#include <stm32f446re_spi_driver.h>
#include <stm32f446re_gpio_driver.h>

#include <spi_tests.h>

static void InitGpioPins(void);
static void InitSpi(void);

void TestSPISendData()
{
    /*Declare data to transmit*/
    char txMsg[] = "Hello";

    /*Initialize GPIO Pins as SPI in Alternate Function Mode*/
    InitGpioPins();

    /*Initialize SPI*/
    InitSpi();

    /*Enable SSI Peripheral -> NSS Internally HIGH*/
    //SPISSIConfig(SPI2, ENABLE);

    /*Enable SPI Peripheral*/
    SPIPeripheralControl(SPI2, ENABLE);

    /*Send Data in DFF- 8 BIT*/
    SPISendData(SPI2, (uint8_t*)txMsg, strlen(txMsg));

    /*Disable SPI Peripheral after data transmit*/
    SPIPeripheralControl(SPI2, DISABLE);

    /*Halt*/
    while(1);
}

static void InitGpioPins(void)
{
    Gpio_Handle_t spiTestPins;
    memset(&spiTestPins, 0, sizeof(spiTestPins));

    spiTestPins.pGPIOx = GPIOB;
    spiTestPins.Gpio_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
    spiTestPins.Gpio_PinConfig.GPIO_AltFnMode = 5;
    spiTestPins.Gpio_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
    spiTestPins.Gpio_PinConfig.GPIO_PuPdControl = GPIO_NO_PUPD;
    spiTestPins.Gpio_PinConfig.GPIO_OPType = GPIO_OP_PP;

    /*Configure CS Pin*/
    spiTestPins.Gpio_PinConfig.GPIO_PinNumber  = GPIO_PIN_12;
    GPIO_Init(&spiTestPins);

    /*Configure SCLK Pin*/
    spiTestPins.Gpio_PinConfig.GPIO_PinNumber  = GPIO_PIN_13;
    GPIO_Init(&spiTestPins);

    /*Configure MISO Pin*/
    spiTestPins.Gpio_PinConfig.GPIO_PinNumber  = GPIO_PIN_14;
    GPIO_Init(&spiTestPins);

    /*Configure MOSI Pin*/
    spiTestPins.Gpio_PinConfig.GPIO_PinNumber  = GPIO_PIN_15;
    GPIO_Init(&spiTestPins);
}

static void InitSpi(void)
{
    spi_handle_t spiTest;
    
    spiTest.pSPIx = SPI2;
    spiTest.spiConfig.spiDeviceMode = SPI_DEVICE_MODE_MASTER;
    spiTest.spiConfig.spiBusConfig = SPI_BUS_CONFIG_FD;
    spiTest.spiConfig.spiSclkSpeed = SPI_SCLK_SPEED_DIV2;
    spiTest.spiConfig.spiDFF = SPI_DFF_8BIT;
    spiTest.spiConfig.spiCPOL = SPI_CPOL_LOW;
    spiTest.spiConfig.spiCPHA = SPI_CPHA_LOW;
    spiTest.spiConfig.spiSSM = SPI_SSM_EN;
    spiTest.spiConfig.spiSSI = SPI_SSI_EN;

    SPIInit(&spiTest);
}
