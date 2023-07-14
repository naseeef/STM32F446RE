/**
 ******************************************************************************
 * @file           : main.c
 * @author         : Naseef Azhikulangara
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2023 Naseef Azhikulangara.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
//#if !defined(__SOFT_FP__) && defined(__ARM_FP)
//  #warning "FPU is not initialized, but the project is compiling for an FPU. Please initialize the FPU before use."
//#endif

#include <stdint.h>

#include <appconfig.h>
#include <stm32f446re.h>
#include <stm32f446re_gpio_driver.h>
#include <stm32f446re_spi_driver.h>

#include <onboard_led.h>
#include <spi_tests.h>

int main(void)
{

#ifdef ENABLE_ONBOARD_LED
		BlinkOnBoardLed();
#endif

#ifdef ONBOARD_LED_BUTTON
		BlinkOBLedWithButton();
#endif

#ifdef INTERRUPT_LED_BUTTON
		InterruptToggleLED();
#endif

#ifdef SPI_SEND_DATA
		TestSPISendData();
#endif


}

void EXTI15_10_IRQHandler(void)
{
	GPIO_IRQHandler(GPIO_PIN_13);

	//Toggle LED
	GPIO_ToggleOutputPin(GPIOC, GPIO_PIN_5);
}
