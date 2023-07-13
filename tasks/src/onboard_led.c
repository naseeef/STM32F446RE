/*
 * onboard_led.c
 *
 *  Created on: 13-Apr-2023
 *      Author: Naseef Azhikulangara
 */

#include <stdint.h>
#include <string.h>
#include <stm32f446re.h>
#include <stm32f446re_gpio_driver.h>
#include "onboard_led.h"

/*
 * Delay Using For Loop
 */
void delay(uint16_t duration_ms)
{
	/*
	 * (clock frequency / 1000ms) * (delay duration needed).
	 * Note that the actual delay may be slightly longer or shorter
	 * than the desired duration due to variations in the clock frequency
	 * and other factors.
	 */
	uint32_t delayCounter = (CLOCK_FREQUENCY / ONE_SECOND_IN_MS) * (duration_ms);

	for(uint32_t i = 0; i < delayCounter; i++);
}

void GpioInitOnBoardLed(void)
{
	Gpio_Handle_t onBoardLed;
	memset(&onBoardLed,0,sizeof(onBoardLed));

	onBoardLed.pGPIOx = GPIOA;

	onBoardLed.Gpio_PinConfig.GPIO_PinNumber 	= GPIO_PIN_5;
	onBoardLed.Gpio_PinConfig.GPIO_PinMode 	= GPIO_MODE_OUT;
	onBoardLed.Gpio_PinConfig.GPIO_PinSpeed 	= GPIO_SPEED_FAST;
	onBoardLed.Gpio_PinConfig.GPIO_PuPdControl = GPIO_NO_PUPD;

	GPIO_Init(&onBoardLed);
}

void GpioInitOnBoardBtn(void)
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

void GpioInitOnBoardBtnInt(void)
{
	Gpio_Handle_t onBoardBtnInt;
	memset(&onBoardBtnInt,0,sizeof(onBoardBtnInt));

	onBoardBtnInt.pGPIOx = GPIOC;

	onBoardBtnInt.Gpio_PinConfig.GPIO_PinNumber 	= GPIO_PIN_13;
	onBoardBtnInt.Gpio_PinConfig.GPIO_PinMode 		= GPIO_MODE_INT_FT;
	onBoardBtnInt.Gpio_PinConfig.GPIO_PinSpeed 		= GPIO_SPEED_FAST;
	onBoardBtnInt.Gpio_PinConfig.GPIO_PuPdControl 	= GPIO_PIN_PU;

	GPIO_Init(&onBoardBtnInt);
}

/*******************************************************************************/

void BlinkOnBoardLed(void)
{
	/*
	 * Initializing GPIO Driver for On Board LED Pin
	 */
	GpioInitOnBoardLed();

	/*
	 * Toggling On Board LED Every One Second
	 */
	while(1)
	{
		GPIO_ToggleOutputPin(GPIOA, GPIO_PIN_5);
		delay(1000);
	}

}

void BlinkOBLedWithButton(void)
{
	/*
	 * Initializing GPIO Driver for On Board Button and LED
	 */
	GpioInitOnBoardBtn();
	GpioInitOnBoardLed();

	while(1)
	{
		uint8_t buttonStatus = GPIO_ReadPin(GPIOC, GPIO_PIN_13);

		if (buttonStatus == GPIO_PIN_SET)
		{
			GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
		}
		else
		{
			GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
		}
	}

}

void InterruptToggleLED(void)
{
	/*
	 * Initializing GPIO Driver for On Board Button and LED
	 */
	GpioInitOnBoardBtnInt();
	GpioInitOnBoardLed();

	//IRQ Configuration
	GPIO_IRQPriorityConfig(IRQ_NO_EXTI15_10, 1);
	GPIO_IRQInterruptConfig(IRQ_NO_EXTI15_10, ENABLE);

}
