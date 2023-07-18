
/*
 * stm32f466re_spi_driver.c
 *
 *  Created on: 17-Jun-2023
 *      Author: Naseef Azhikulangara
 */
#include <stm32f446re_spi_driver.h>
//#include <stm32f446re.h>

static void SPITXEInterruptHandle(spi_handle_t *pSPIHandle);
static void SPIRXNEInterruptHandle(spi_handle_t *pSPIHandle);
static void SPIOVRErrInterruptHandle(spi_handle_t *pSPIHandle);


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

/*****************************************************************
 * @fn          - SPISendDataInterrupt
 *
 * @brief       - This function sends data over SPI
 *                peripheral in Interrupt mode
 *
 * @param[in]   - Pointer to SPI Handle structure
 * @param[in]   - Transmit buffer
 * @param[in]   - Length of transmit buffer
 *
 * @return      - Tx State
 *
 * @Note        - None
 *
 *****************************************************************/
uint8_t SPISendDataInterrupt(spi_handle_t *pSPIHandle, uint8_t *pTxBuffer, uint32_t Length)
{
    uint8_t state = pSPIHandle->TxState;

    if(state != SPI_BUSY_IN_TX)
    {
        /* Save Tx buffer address and length information in global variables */
        pSPIHandle->pTxBuffer = pTxBuffer;
        pSPIHandle->TxLen = Length;

        /* Mark SPI state as busy so that no other code can take over SPI peripheral until transmission is over */
        pSPIHandle->TxState = SPI_BUSY_IN_TX;

        /* Enable TXEIE control bit to get interrupt whenever TXE flag is set in SR */
        pSPIHandle->pSPIx->CR2 |= (1 << SPI_CR2_TXEIE);
    }
    /* DBG->Data transmission*/
    return state;
}


/*****************************************************************
 * @fn          - SPIReceiveDataInterrupt
 *
 * @brief       - This function receives data over SPI
 *                peripheral in Interrupt mode
 *
 * @param[in]   - Pointer to SPI Handle structure
 * @param[in]   - Transmit buffer
 * @param[in]   - Length of transmit buffer
 *
 * @return      - Rx State
 *
 * @Note        - None
 *
 *****************************************************************/
uint8_t SPIReceiveDataInterrupt(spi_handle_t *pSPIHandle, uint8_t *pRxBuffer, uint32_t Length)
{
    uint8_t state = pSPIHandle->RxState;

    if(state != SPI_BUSY_IN_RX)
    {
        /* Save Rx buffer address and length information in global variables */
        pSPIHandle->pRxBuffer = pRxBuffer;
        pSPIHandle->RxLen = Length;

        /* Mark SPI state as busy so that no other code can take over SPI peripheral until transmission is over */
        pSPIHandle->RxState = SPI_BUSY_IN_RX;

        /* Enable RXNEIE control bit to get interrupt whenever RXE flag is set in SR */
        pSPIHandle->pSPIx->CR2 |= (1 << SPI_CR2_RXNEIE);
    }

    return state;
}

/*****************************************************************
 * @fn          - SPIIRQInterruptConfig
 *
 * @brief       - This function configures interrupt
 *
 * @param[in]   - IRQ Interrupt number
 * @param[in]   - Macro: Enable/Disable
 *
 * @return      - None
 *
 * @Note        - None
 *
 *****************************************************************/
void SPIIRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi)
{
    if(EnorDi == ENABLE)
    {
        if(IRQNumber <= 31)
        {
            /* Program ISER0 register */
            *NVIC_ISER0 |= (1 << IRQNumber);
        }
        else if(IRQNumber > 31 && IRQNumber < 64)
        {
            /* Program ISER1 register (32 to 63) */
            *NVIC_ISER1 |= (1 << (IRQNumber % 32));
        }
        else if(IRQNumber >= 64 && IRQNumber < 96)
        {
            /* Program ISER2 register (64 to 95) */
            *NVIC_ISER2 |= (1 << (IRQNumber % 64));
        }
    }
    else
    {
        if(IRQNumber <= 31)
        {
            /* Program ICER0 register */
            *NVIC_ICER0 |= (1 << IRQNumber);
        }
        else if(IRQNumber > 31 && IRQNumber < 64)
        {
            /* Program ICER1 register (32 to 63) */
            *NVIC_ICER1 |= (1 << (IRQNumber % 32));
        }
        else if(IRQNumber >= 64 && IRQNumber < 96)
        {
            /* Program ICER2 register (64 to 95) */
            *NVIC_ICER2 |= (1 << (IRQNumber % 64));
        }
    }
}


/*****************************************************************
 * @fn          - SPIIRQPriorityConfig
 *
 * @brief       - This function configures interrupt priority
 *
 * @param[in]   - IRQ Interrupt number
 * @param[in]   - IRQ interrupt priority
 *
 * @return      - None
 *
 * @Note        - None
 *
 *****************************************************************/
void SPIIRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority)
{
    uint8_t iprx = IRQNumber / 4;
    uint8_t iprx_section = IRQNumber % 4;

    uint8_t shift_amount = (8 * iprx_section) + (8 - NO_PR_BITSIMPLENETED);
    *(NVIC_PR_BASE_ADDR + iprx) |= (IRQPriority << shift_amount);
}

/*****************************************************************
 * @fn          - SPIIRQHandling
 *
 * @brief       - This function handle interrupts
 *
 * @param[in]   - Handle structure
 *
 * @return      - None
 *
 * @Note        - None
 *
 *****************************************************************/
void SPIIRQHandling(spi_handle_t *pSPIHandle)
{
	uint8_t temp1;
    uint8_t temp2;

    /* Check for TXE */
    temp1 = pSPIHandle->pSPIx->SR & (1 << SPI_SR_TXE);
    temp2 = pSPIHandle->pSPIx->CR2 & (1 << SPI_CR2_TXEIE);

    if(temp1 && temp2)
    {
        /* Handle TXE */
        SPITXEInterruptHandle(pSPIHandle);
    }

    /* Check for RXNE */
    temp1 = pSPIHandle->pSPIx->SR & (1 << SPI_SR_RXNE);
    temp2 = pSPIHandle->pSPIx->CR2 & (1 << SPI_CR2_RXNEIE);

    if(temp1 && temp2)
    {
        /* Handle RXNE */
        SPIRXNEInterruptHandle(pSPIHandle);
    }

    /* Check for OVR flag */
    temp1 = pSPIHandle->pSPIx->SR & (1 << SPI_SR_OVR);
    temp2 = pSPIHandle->pSPIx->CR2 & (1 << SPI_CR2_ERRIE);

    if(temp1 && temp2)
    {
        /* Handle OVR Error */
        SPIOVRErrInterruptHandle(pSPIHandle);
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

/*****************************************************************
 * @fn          - SPICloseTransmission
 *
 * @brief       - This function close SPI transmission
 *
 * @param[in]   - Handle structure
 *
 * @return      - None
 *
 * @Note        - None
 *
 *****************************************************************/
void SPICloseTransmission(spi_handle_t *pSPIHandle)
{
    pSPIHandle->pSPIx->CR2 &= ~(1 << SPI_CR2_TXEIE);
    pSPIHandle->pTxBuffer = NULL;
    pSPIHandle->TxLen = 0;
    pSPIHandle->TxState = SPI_READY;
}


/*****************************************************************
 * @fn          - SPICloseReception
 *
 * @brief       - This function close SPI reception
 *
 * @param[in]   - Handle structure
 *
 * @return      - None
 *
 * @Note        - None
 *
 *****************************************************************/
void SPICloseReception(spi_handle_t *pSPIHandle)
{
    pSPIHandle->pSPIx->CR2 &= ~(1 << SPI_CR2_RXNEIE);
    pSPIHandle->pRxBuffer = NULL;
    pSPIHandle->RxLen = 0;
    pSPIHandle->RxState = SPI_READY;
}


/*****************************************************************
 * @fn          - SPI_ClearOVRFlag
 *
 * @brief       - This function clears OVR flag
 *
 * @param[in]   - Base address of the SPI peripheral
 *
 * @return      - None
 *
 * @Note        - None
 *
 *****************************************************************/
void SPIClearOVRFlag(spi_reg_t *pSPIx)
{
    uint8_t temp;
    temp = pSPIx->DR;
    temp = pSPIx->SR;
    (void)temp;
}

/*****************************************************************
 *               Helper functions implementation                 *
 *****************************************************************/
/*****************************************************************
 * @fn          - SPIApplicationEventCB
 *
 * @brief       - Application event callback function
 *
 * @param[in]   - Handle structure
 * @param[in]   - Application event
 *
 * @return      - None
 *
 * @Note        - None
 *
 *****************************************************************/
__weak void SPIApplicationEventCB(spi_handle_t *pSPIHandle, uint8_t AppEvent)
{
    /* This is a week implementation. The application may override this function. */
}

/*****************************************************************
 * @fn          - SPITXEInterruptHandle
 *
 * @brief       - This function handles TXE in interrupt mode
 *
 * @param[in]   - Pointer to SPI Handle structure
 *
 * @return      - None
 *
 * @Note        - None
 *
 *****************************************************************/
static void SPITXEInterruptHandle(spi_handle_t *pSPIHandle)
{
	if( pSPIHandle->pSPIx->CR1 & (1 << SPI_CR1_DFF) )
    {
        /* Load data into data register */
        /* 16 bit */
        pSPIHandle->pSPIx->DR = *((uint16_t*)pSPIHandle->pTxBuffer);
        pSPIHandle->TxLen--;
        pSPIHandle->TxLen--;
        (uint16_t*)pSPIHandle->pTxBuffer++;
    }
    else
    {
        /* 8 bit */
        pSPIHandle->pSPIx->DR = *pSPIHandle->pTxBuffer;
        pSPIHandle->TxLen--;
        pSPIHandle->pTxBuffer++;
    }

    if(!pSPIHandle->TxLen)
    {
        /* Tx is zero. Close SPI communication and inform application about it.
         * Prevents interrupts from setting up of TXE flag. */
        SPICloseTransmission(pSPIHandle);
        SPIApplicationEventCB(pSPIHandle, SPI_EVENT_TX_CMPLT);
    }
}

/*****************************************************************
 * @fn          - SPIRXNEInterruptHandle
 *
 * @brief       - This function handles RXNE in interrupt mode
 *
 * @param[in]   - Pointer to SPI Handle structure
 *
 * @return      - None
 *
 * @Note        - None
 *
 *****************************************************************/
static void SPIRXNEInterruptHandle(spi_handle_t *pSPIHandle)
{
	 if( pSPIHandle->pSPIx->CR1 & (1 << SPI_CR1_DFF) )
    {
        /* Load data from data register into buffer */
        /* 16 bit */
        *((uint16_t*)pSPIHandle->pRxBuffer) = pSPIHandle->pSPIx->DR;
        pSPIHandle->RxLen--;
        pSPIHandle->RxLen--;
        (uint16_t*)pSPIHandle->pRxBuffer++;
    }
    else
    {
        /* 8 bit */
        *pSPIHandle->pRxBuffer = pSPIHandle->pSPIx->DR;//DBG->Check brackets
        pSPIHandle->RxLen--;
        pSPIHandle->pRxBuffer++;
    }

    if(!pSPIHandle->RxLen)
    {
        /* Rx is zero. Close SPI communication and inform application about it.
         * Prevents interrupts from setting up of RXNE flag. */
        SPICloseReception(pSPIHandle);
        SPIApplicationEventCB(pSPIHandle, SPI_EVENT_RX_CMPLT);
    }
}

/*****************************************************************
 * @fn          - SPIOVRErrInterruptHandle
 *
 * @brief       - This function handles OVR_ERR in
 *                interrupt mode
 *
 * @param[in]   - Pointer to SPI Handle structure
 *
 * @return      - None
 *
 * @Note        - None
 *
 *****************************************************************/
static void SPIOVRErrInterruptHandle(spi_handle_t *pSPIHandle)
{
	uint8_t temp;

    /* Clear OVR flag */
    if(pSPIHandle->TxState != SPI_BUSY_IN_TX)
    {
        temp = pSPIHandle->pSPIx->DR;
        temp = pSPIHandle->pSPIx->SR;
    }
    (void)temp;

    /* Inform application */
    SPIApplicationEventCB(pSPIHandle, SPI_EVENT_OVR_ERR);
}
