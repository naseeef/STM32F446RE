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
#include <onboard_led.h>
#include <stm32f446re.h>
#include <stm32f446re_gpio_driver.h>

int main(void)
{

#ifdef ENABLE_ONBOARD_LED
		BlinkOnBoardLed();
#endif

#ifdef ONBOARD_LED_BUTTON
		BlinkOBLedWithButton();
#endif


}
