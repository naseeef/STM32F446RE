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
static void InitSpiForArduinoComm(void);
static void GpioInitOnBoardBtn(void);
static void InitGpioInterruptPin(void);

#ifndef SPI_INT_ARDUINO_SLAVE_TEST
	spi_handle_t spiTest;

	#define MAX_LEN 500

	char RcvBuff[MAX_LEN];

	__vo char ReadByte;


	__vo uint8_t rcvStop = 0;

	/*This flag will be set in the interrupt handler of the Arduino interrupt GPIO */
	__vo uint8_t dataAvailable = 0;
#endif

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

void TestSPISendDataToArduino()
{
    /*Declare data to transmit*/
    char txMsg[] = "Hello";

    /*Initialize GPIO Pins as SPI in Alternate Function Mode*/
    InitGpioPins();
    GpioInitOnBoardBtn();

    /*Initialize SPI*/
    InitSpiForArduinoComm();

    while (1)
    {
        /*Wait until button press*/
        while(!GPIO_ReadPin(GPIOC, GPIO_PIN_13));

        /*Enable SPI Peripheral*/
        SPIPeripheralControl(SPI2, ENABLE);

        /*Config SSOE in SPI CR2 for hardware slave management*/
        SPISSOEConfig(SPI2, ENABLE);

        /*Send the length data first, Slave doesn't know how many data need to receive*/
        uint8_t dataLen = strlen(txMsg);
        SPISendData(SPI2, &dataLen, 1);

        /*Send Data in DFF- 8 BIT*/
        SPISendData(SPI2, (uint8_t*)txMsg, strlen(txMsg));

        /*Wait before disabling SPI, check whether it's busy or not*/
        while(! SPIGetFlagStatus(SPI2, SPI_SR_BSY));

        /*Disable SPI Peripheral after data transmit*/
        SPIPeripheralControl(SPI2, DISABLE);
    }

}

void TestSPIMasterSlave()
{
    /*Declare data to transmit*/
    uint8_t txData = 0xA5;
    uint8_t dummyWrite = 0xFF;
    uint8_t dummyCode;
    uint8_t rxData;
    uint8_t args[2] = {0x09,0x01};

    /*Initialize GPIO Pins as SPI in Alternate Function Mode*/
    InitGpioPins();
    GpioInitOnBoardBtn();

    /*Initialize SPI*/
    InitSpiForArduinoComm();

    while (1)
    {
        /*Wait until button press*/
        while(!GPIO_ReadPin(GPIOC, GPIO_PIN_13));

        /*Enable SPI Peripheral*/
        SPIPeripheralControl(SPI2, ENABLE);

        /*Config SSOE in SPI CR2 for hardware slave management*/
        SPISSOEConfig(SPI2, ENABLE);

        /*Send command to Slave*/
        SPISendData(SPI2, &txData, 1);
        /*Read dummy data from slave inorder to clear RXNE*/
        SPIReceiveData(SPI2, &dummyCode, 1);
        /*Send 1Byte of Dummy data to slave inorder to push the data from shift register*/
        SPISendData(SPI2, &dummyWrite, 1);
        /*Receive data from slave*/
        SPIReceiveData(SPI2, &rxData, 1);

        if(rxData == 0xF5)
        {
            //printf("Slave Verified");
            SPISendData(SPI2, args, 2);
        }
        else
        {
            //printf("ERRORRRRRRRR");
        }

        /*Wait before disabling SPI, check whether it's busy or not*/
        while(! SPIGetFlagStatus(SPI2, SPI_SR_BSY));

        /*Disable SPI Peripheral after data transmit*/
        SPIPeripheralControl(SPI2, DISABLE);
    }

}


void TestSPIReceiveOverInterrupt(void)
{
    uint8_t dummy = 0xff;

    InitGpioPins();
    InitGpioInterruptPin();
    InitSpi();

    /*Enable SPI Peripheral*/
    SPIPeripheralControl(SPI2, ENABLE);

    /*Config SSOE in SPI CR2 for hardware slave management*/
    SPISSOEConfig(SPI2, ENABLE);

    SPIIRQInterruptConfig(IRQ_NO_SPI2, ENABLE);

    while(1)
    {
		rcvStop = 0;

		while(!dataAvailable); //wait till data available interrupt from transmitter device(slave)

		GPIO_IRQInterruptConfig(IRQ_NO_EXTI9_5,DISABLE);

		//enable the SPI2 peripheral
		SPIPeripheralControl(SPI2,ENABLE);


		while(!rcvStop)
		{
			/* fetch the data from the SPI peripheral byte by byte in interrupt mode */
			while ( SPISendDataInterrupt(&spiTest,&dummy,1) == SPI_BUSY_IN_TX);
			while ( SPIReceiveDataInterrupt(&spiTest,&ReadByte,1) == SPI_BUSY_IN_RX );
		}


		// confirm SPI is not busy
		while( SPIGetFlagStatus(SPI2, SPI_BSY_FLAG) );

		//Disable the SPI2 peripheral
		SPIPeripheralControl(SPI2,DISABLE);

		printf("Rcvd data = %s\n",RcvBuff);

		dataAvailable = 0;

		GPIO_IRQInterruptConfig(IRQ_NO_EXTI9_5,ENABLE);
	}
    
    //return 0;
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

static void GpioInitOnBoardBtn(void)
{
	Gpio_Handle_t onBoardBtn;
	memset(&onBoardBtn,0,sizeof(onBoardBtn));

	onBoardBtn.pGPIOx = GPIOC;

	onBoardBtn.Gpio_PinConfig.GPIO_PinNumber 	= GPIO_PIN_13;
	onBoardBtn.Gpio_PinConfig.GPIO_PinMode 		= GPIO_MODE_IN;
	onBoardBtn.Gpio_PinConfig.GPIO_PinSpeed 	= GPIO_SPEED_FAST;
	onBoardBtn.Gpio_PinConfig.GPIO_OPType 		= GPIO_OP_PP;
	onBoardBtn.Gpio_PinConfig.GPIO_PuPdControl 	= GPIO_NO_PUPD;

	GPIO_Init(&onBoardBtn);
}

static void InitSpiForArduinoComm(void)
{

    spi_handle_t spiTestArduino;
    
    spiTestArduino.pSPIx = SPI2;
    spiTestArduino.spiConfig.spiDeviceMode = SPI_DEVICE_MODE_MASTER;
    spiTestArduino.spiConfig.spiBusConfig = SPI_BUS_CONFIG_FD;
    spiTestArduino.spiConfig.spiSclkSpeed = SPI_SCLK_SPEED_DIV2;
    spiTestArduino.spiConfig.spiDFF = SPI_DFF_8BIT;
    spiTestArduino.spiConfig.spiCPOL = SPI_CPOL_LOW;
    spiTestArduino.spiConfig.spiCPHA = SPI_CPHA_LOW;
    spiTestArduino.spiConfig.spiSSM = SPI_SSM_DI;
    spiTestArduino.spiConfig.spiSSI = SPI_SSI_DI;

    SPIInit(&spiTestArduino);
}

static void InitGpioInterruptPin(void)
{
	Gpio_Handle_t spiInterruptPin;
	memset(&spiInterruptPin,0,sizeof(spiInterruptPin));

	spiInterruptPin.pGPIOx = GPIOD;

	spiInterruptPin.Gpio_PinConfig.GPIO_PinNumber 	= GPIO_PIN_6;
	spiInterruptPin.Gpio_PinConfig.GPIO_PinMode 		= GPIO_MODE_INT_FT;
	spiInterruptPin.Gpio_PinConfig.GPIO_PinSpeed 	= GPIO_SPEED_LOW;
	spiInterruptPin.Gpio_PinConfig.GPIO_PuPdControl 	= GPIO_PIN_PU;

	GPIO_Init(&spiInterruptPin);

    GPIO_IRQInterruptConfig(IRQ_NO_EXTI9_5, ENABLE);
    GPIO_IRQPriorityConfig(IRQ_NO_EXTI9_5, NVIC_IRQ_PRI15);
}


/* Runs when a data byte is received from the peripheral over SPI*/
void SPI2IRQHandler(void)
{

	SPIIRQHandling(&spiTest);
}



void SPIApplicationEventCB(spi_handle_t *pSPIHandle,uint8_t AppEv)
{
	static uint32_t i = 0;
	/* In the RX complete event , copy data in to rcv buffer . '\0' indicates end of message(rcvStop = 1) */
	if(AppEv == SPI_EVENT_RX_CMPLT)
	{
				RcvBuff[i++] = ReadByte;
				if(ReadByte == '\0' || ( i == MAX_LEN)){
					rcvStop = 1;
					RcvBuff[i-1] = '\0';
					i = 0;
				}
	}

}

/* Slave data available interrupt handler */
void EXTI9_5_IRQHandler(void)
{
	GPIO_IRQHandler(GPIO_PIN_6);
	dataAvailable = 1;
}
