/*
 * onboard_led.h
 *
 *  Created on: 13-Apr-2023
 *      Author: Naseef Azhikulangara
 */

#ifndef ONBOARD_LED_H_
#define ONBOARD_LED_H_

/*
 * For Generating Delay
 */
#define CLOCK_FREQUENCY			2000000
#define ONE_SECOND_IN_MS		1000

void GpioInitOnBoardLed(void);
void BlinkOnBoardLed(void);
void delay(uint16_t duration_ms);
void BlinkOBLedWithButton(void);
void GpioInitOnBoardBtn(void);


#endif /* ONBOARD_LED_H_ */
