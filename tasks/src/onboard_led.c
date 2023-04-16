/*
 * onboard_led.c
 *
 *  Created on: 13-Apr-2023
 *      Author: Naseef Azhikulangara
 */

#include <stdint.h>
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

	onBoardBtn.pGPIOx = GPIOC;

	onBoardBtn.Gpio_PinConfig.GPIO_PinNumber 	= GPIO_PIN_13;
	onBoardBtn.Gpio_PinConfig.GPIO_PinMode 	= GPIO_MODE_IN;
	onBoardBtn.Gpio_PinConfig.GPIO_PinSpeed 	= GPIO_SPEED_FAST;
	onBoardBtn.Gpio_PinConfig.GPIO_OPType 		= GPIO_OP_PP;
	onBoardBtn.Gpio_PinConfig.GPIO_PuPdControl = GPIO_NO_PUPD;

	GPIO_Init(&onBoardBtn);
}

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
