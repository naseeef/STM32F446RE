/*
 * appconfig.h
 *
 *  Created on: 13-Apr-2023
 *      Author: Naseef Azhikulangara
 */

#ifndef APPCONFIG_H_
#define APPCONFIG_H_

/***************************GPIO******************************************/
/*
 ***Enable On-Board LED.
 *LED will blink in every one second
 */
//#define ENABLE_ONBOARD_LED		1

/*
 * Enable On-Board LED with On-Board Button
 */
//#define ONBOARD_LED_BUTTON		1

/*
 * Toggle On-Board LED with On-Board Button using Interrupt
 */
//#define INTERRUPT_LED_BUTTON		1

/***************************SPI********************************************/
/*
 * SPI Testing.
 * Used to Send Data in SPI
 */
//#define SPI_SEND_DATA		1

/*
 * SPI Testing. STM as Master
 * Used to Send Data  to Arduino in SPI
 */
//#define SPI_SEND_DATA_ARDUINO		1

/*
 * SPI Testing with Arduini in Full Duplex Mode
 * Used to Send Data to Arduino in SPI
 */
//#define SPI_MASTER_SLAVE_FD		1

/*
 * SPI Interrupt Testing with Arduino.
 * Used to Receive Data in SPI Interrupt mode
 */
#define SPI_INT_ARDUINO_SLAVE_TEST  1

#endif /* APPCONFIG_H_ */
