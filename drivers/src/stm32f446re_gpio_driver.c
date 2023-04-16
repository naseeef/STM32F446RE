/*
 * stm32f446re_gpio_driver.c
 *
 *  Created on: 12-Apr-2023
 *      Author: Naseef Azhikulangara
 */
#include <stdint.h>
#include <stm32f446re.h>
#include <stm32f446re_gpio_driver.h>


/**********************************************************************************
 * @fn			- GPIO_PCLK_Control
 *
 * @brief		- Enables peripheral clock for GPIO pins
 *
 * @param[]		- Base address of GPIO Peripheral
 * @param[]		- ENABLE or DISABLE macro
 *
 * @return		- None
 *
 * @Note		- None
 *
 */
void GPIO_PCLK_Control(Gpio_Reg_t *pGPIOx, uint8_t ENorDI)
{
	if(ENorDI == ENABLE)
	{
		if(pGPIOx == GPIOA)
		{
			GPIOA_PCLK_EN();
		}
		else if(pGPIOx == GPIOB)
		{
			GPIOB_PCLK_EN();
		}
		else if(pGPIOx == GPIOC)
		{
			GPIOC_PCLK_EN();
		}
		else if(pGPIOx == GPIOD)
		{
			GPIOD_PCLK_EN();
		}
		else if(pGPIOx == GPIOE)
		{
			GPIOE_PCLK_EN();
		}
		else if(pGPIOx == GPIOF)
		{
			GPIOF_PCLK_EN();
		}
		else if(pGPIOx == GPIOG)
		{
			GPIOG_PCLK_EN();
		}
		else if(pGPIOx == GPIOH)
		{
			GPIOH_PCLK_EN();
		}
	}
	else
	{
		if(pGPIOx == GPIOA)
		{
			GPIOA_PCLK_DI();
		}
		else if(pGPIOx == GPIOB)
		{
			GPIOB_PCLK_DI();
		}
		else if(pGPIOx == GPIOC)
		{
			GPIOC_PCLK_DI();
		}
		else if(pGPIOx == GPIOD)
		{
			GPIOD_PCLK_DI();
		}
		else if(pGPIOx == GPIOE)
		{
			GPIOE_PCLK_DI();
		}
		else if(pGPIOx == GPIOF)
		{
			GPIOF_PCLK_DI();
		}
		else if(pGPIOx == GPIOG)
		{
			GPIOG_PCLK_DI();
		}
		else if(pGPIOx == GPIOH)
		{
			GPIOH_PCLK_DI();
		}
	}
}


/**********************************************************************************
 * @fn			- GPIO_Init
 *
 * @brief		- Initializing GPIO Ports
 *
 * @param[]		- Base address of GPIO Port
 *
 * @return		- None
 *
 * @Note		- None
 *
 */
void GPIO_Init(Gpio_Handle_t *pGPIOHandle)
{
	static uint32_t temp = 0;
	static uint8_t ALTFNReg = 0; 		/*ALTFN Register Selection*/
	static uint8_t ALTFNBpos = 0;		/*ALTFN Bit Position*/
	static uint8_t SYSCFGRReg = 0;		/*SYSCFG Register Selection*/
	static uint8_t SYSCFGRBpos = 0;		/*SYSCFG Bit Position*/

	/*
	 * Enable Peripheral Clock
	 */
	GPIO_PCLK_Control(pGPIOHandle->pGPIOx, ENABLE);

	/*
	 * Configuring Pin Modes
	 */
	//Only configuring Non-INT based Modes.
	if(pGPIOHandle->Gpio_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG)
	{
		temp = (pGPIOHandle->Gpio_PinConfig.GPIO_PinMode << (2* pGPIOHandle->Gpio_PinConfig.GPIO_PinNumber));
		pGPIOHandle->pGPIOx->MODER &= ~(0x3 << (2* pGPIOHandle->Gpio_PinConfig.GPIO_PinNumber));
		pGPIOHandle->pGPIOx->MODER |= temp;
	}
	else
	{
		if (pGPIOHandle->Gpio_PinConfig.GPIO_PinMode == GPIO_MODE_INT_FT)
		{
			/*Configuring FTSR*/
			EXTI->FTSR |= (1 << pGPIOHandle->Gpio_PinConfig.GPIO_PinNumber);
			//clear the corresponding RTSR Bit, Safety Measure
			EXTI->RTSR &= ~(1 << pGPIOHandle->Gpio_PinConfig.GPIO_PinNumber);
		}
		else if (pGPIOHandle->Gpio_PinConfig.GPIO_PinMode == GPIO_MODE_INT_RT)
		{
			/*Configuring RTSR*/
			EXTI->RTSR |= (1 << pGPIOHandle->Gpio_PinConfig.GPIO_PinNumber);
			//clear the corresponding FTSR Bit, Safety Measure
			EXTI->FTSR &= ~(1 << pGPIOHandle->Gpio_PinConfig.GPIO_PinNumber);
		}
		else if (pGPIOHandle->Gpio_PinConfig.GPIO_PinMode == GPIO_MODE_INT_RFT)
		{
			/*Configuring FTSR and RTSR*/
			EXTI->FTSR |= (1 << pGPIOHandle->Gpio_PinConfig.GPIO_PinNumber);
			EXTI->RTSR |= (1 << pGPIOHandle->Gpio_PinConfig.GPIO_PinNumber);
		}

		/*Enabling Peripheral Clock for SYSCFG*/
		SYSCFG_PCLK_EN();

		/*Configuring GPIO Port in SYSCFG-EXTICR*/

		/*SYSCFG Register Selection*/
		SYSCFGRReg = pGPIOHandle->Gpio_PinConfig.GPIO_PinNumber / 4;
		/*SYSCFG Bit Position*/
		SYSCFGRBpos = (pGPIOHandle->Gpio_PinConfig.GPIO_PinNumber % 4);

		SYSCFG->EXTICR[SYSCFGRReg] = GPIO_BASEADDR_TO_CODE(pGPIOHandle->pGPIOx) << (SYSCFGRBpos * 4);


		/*Enable EXTI Interrupt Delivery using IMR*/
		EXTI->IMR |= (1 << pGPIOHandle->Gpio_PinConfig.GPIO_PinNumber);

	}
	temp = 0;

	/* 
	 * Configuring Pin Speed
	 */
	temp = (pGPIOHandle->Gpio_PinConfig.GPIO_PinSpeed << (2* pGPIOHandle->Gpio_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->OSPEEDR &= ~(0x3 << (2* pGPIOHandle->Gpio_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->OSPEEDR |= temp;
	temp = 0;

	/*
	 * Configuring Pull-up and Pull-Down
	 */
	temp = (pGPIOHandle->Gpio_PinConfig.GPIO_PuPdControl << (2* pGPIOHandle->Gpio_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->PUPDR &= ~(0x3 << (2* pGPIOHandle->Gpio_PinConfig.GPIO_PinNumber));
	pGPIOHandle -> pGPIOx->PUPDR |= temp;
	temp = 0;

	/*
	 * Configuring Output Type
	 */
	temp = (pGPIOHandle-> Gpio_PinConfig.GPIO_OPType << (1* pGPIOHandle->Gpio_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->OTYPER &= ~(0x1 << pGPIOHandle->Gpio_PinConfig.GPIO_PinNumber);
	pGPIOHandle->pGPIOx->OTYPER |= temp;
	temp = 0;

	/*
	 * Configuring Alternate Function
	 */
	if(pGPIOHandle-> Gpio_PinConfig.GPIO_PinMode == pGPIOHandle->Gpio_PinConfig.GPIO_AltFnMode)
	{
		/*ALTFN Register Selection*/
		ALTFNReg = (pGPIOHandle->Gpio_PinConfig.GPIO_PinNumber/8);
		/*ALTFN Bit Position*/
		ALTFNBpos = (pGPIOHandle->Gpio_PinConfig.GPIO_PinNumber%8);

		pGPIOHandle->pGPIOx->AFR[ALTFNReg] &= ~(0xF << (4* ALTFNBpos));

		temp = (pGPIOHandle-> Gpio_PinConfig.GPIO_AltFnMode << (4* ALTFNBpos));
		pGPIOHandle->pGPIOx->AFR[ALTFNReg] |= temp;
	}

}
/**********************************************************************************
 * @fn			- GPIO_DeInit
 *
 * @brief		- GPIO De-Initialization
 *
 * @param[]		- GPIO Base Address
 *
 * @return		- None
 *
 * @Note		- None
 *
 */
void GPIO_DeInit(Gpio_Reg_t *pGPIOx)
{
	if(pGPIOx == GPIOA)
	{
		GPIOA_REG_RESET();
	}
	else if(pGPIOx == GPIOB)
	{
		GPIOB_REG_RESET();
	}
	else if(pGPIOx == GPIOC)
	{
		GPIOC_REG_RESET();
	}
	else if(pGPIOx == GPIOD)
	{
		GPIOD_REG_RESET();
	}
	else if(pGPIOx == GPIOE)
	{
		GPIOE_REG_RESET();
	}
	else if(pGPIOx == GPIOF)
	{
		GPIOF_REG_RESET();
	}
	else if(pGPIOx == GPIOG)
	{
		GPIOG_REG_RESET();
	}
	else if(pGPIOx == GPIOH)
	{
		GPIOH_REG_RESET();
	}
}


/*
 * Data read and write
 */
/*****************************************************************
 * @fn          - GPIO_ReadFromInputPin
 *
 * @brief       - This function reads value of input pin, on
 *                a specific port
 *
 * @param[in]   - Base address of the GPIO peripheral
 * @param[in]   - Pin number
 *
 * @return      - Content of the input data
 *
 * @Note        - 0 or 1
 *
 *****************************************************************/
uint8_t GPIO_ReadPin(Gpio_Reg_t *pGPIOx, uint8_t PinNumber)
{
    uint8_t value;

    value = (uint8_t )((pGPIOx->IDR >> PinNumber) & 0x00000001 );

    return value;
}


/*****************************************************************
 * @fn          - GPIO_ReadFromInputPort
 *
 * @brief       - This function reads value of input port
 *
 * @param[in]   - Base address of the GPIO peripheral
 *
 * @return      - Content of the input data
 *
 * @Note        - None
 *
 *****************************************************************/
uint16_t GPIO_ReadPort(Gpio_Reg_t *pGPIOx)
{
    uint16_t value;

    value = (uint16_t )pGPIOx->IDR;

    return value;
}


/*****************************************************************
 * @fn          - GPIO_WriteToOutputPin
 *
 * @brief       - This function writes value on a specific
 *                output pin
 *
 * @param[in]   - Base address of the GPIO peripheral
 * @param[in]   - Pin number
 * @param[in]   - Value (Set/Reset Macro)
 *
 * @return      - None
 *
 * @Note        - None
 *
 *****************************************************************/
void GPIO_WritePin(Gpio_Reg_t *pGPIOx, uint8_t PinNumber, uint8_t value)
{
    if(value == GPIO_PIN_SET)
    {
        /* Write 1 to the output data register at the bit field corresponding to the pin number */
        pGPIOx->ODR |= ( 1 << PinNumber );
    }
    else
    {
        /* Write 0 to the output data register at the bit field corresponding to the pin number */
        pGPIOx->ODR &= ~( 1 << PinNumber );	//Clear pin
    }
}


/*****************************************************************
 * @fn          - GPIO_WriteToOutputPort
 *
 * @brief       - This function writes value on a specific
 *                output port
 *
 * @param[in]   - Base address of the GPIO peripheral
 * @param[in]   - Value (Set/Reset Macro)
 *
 * @return      - None
 *
 * @Note        - None
 *
 *****************************************************************/
void GPIO_WritePort(Gpio_Reg_t *pGPIOx, uint16_t value)
{
    pGPIOx->ODR = value;
}


/*****************************************************************
 * @fn          - GPIO_ToggleOutputPin
 *
 * @brief       - This function toggles specific output pin
 *
 * @param[in]   - Base address of the GPIO peripheral
 * @param[in]   - Pin number
 *
 * @return      - None
 *
 * @Note        - None
 *
 *****************************************************************/
void GPIO_ToggleOutputPin(Gpio_Reg_t *pGPIOx, uint8_t PinNumber)
{
    pGPIOx->ODR ^= (1 << PinNumber);
}

/**********************************************************************************
 * @fn			- 
 *
 * @brief		- 
 *
 * @param[]		- 
 * @param[]		- 
 *
 * @return		- 
 *
 * @Note		- 
 *
 */
void GPIO_IRQConfig(uint8_t IRQNumber, uint8_t IRQPriority, uint8_t ENorDI);
/**********************************************************************************
 * @fn			- 
 *
 * @brief		- 
 *
 * @param[]		- 
 * @param[]		- 
 *
 * @return		- 
 *
 * @Note		- 
 *
 */
void GPIO_IRQHandler(uint8_t pinNumber);
