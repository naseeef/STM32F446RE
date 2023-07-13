/*
 * stm32f446re_gpio_driver.h
 *
 *  Created on: 12-Apr-2023
 *      Author: Naseef Azhikulangara
 */

#ifndef INC_STM32F446RE_GPIO_DRIVER_H_
#define INC_STM32F446RE_GPIO_DRIVER_H_

#include<stm32f446re.h>

/*
 * GPIO Pin Configuration Structure
 */
typedef struct
{
	uint8_t GPIO_PinNumber;		/*!<@GPIO_PIN_NUMBERS>*/
	uint8_t GPIO_PinMode;		/*!<@GPIO_PIN_MODES>*/
	uint8_t GPIO_PinSpeed;		/*!<@GPIO_SPEED_MODES>*/
	uint8_t GPIO_PuPdControl;	/*!<@GPIO_PUPD_CONTROL>*/
	uint8_t GPIO_OPType;		/*!<@GPIO_OP_MODES>*/
	uint8_t GPIO_AltFnMode;		/*!<>*/
}Gpio_PinConfig_t;


/*
 * GPIO Handle Structure
 */
typedef struct
{
	Gpio_Reg_t *pGPIOx; //holds base address of GPIO structure
	Gpio_PinConfig_t Gpio_PinConfig;
}Gpio_Handle_t;

/*
 * @GPIO_PIN_NUMBERS
 * GPIO Pin Numbers
 */
#define GPIO_PIN_0			0
#define GPIO_PIN_1			1
#define GPIO_PIN_2			2
#define GPIO_PIN_3			3
#define GPIO_PIN_4			4
#define GPIO_PIN_5			5
#define GPIO_PIN_6			6
#define GPIO_PIN_7			7
#define GPIO_PIN_8			8
#define GPIO_PIN_9			9
#define GPIO_PIN_10			10
#define GPIO_PIN_11			11
#define GPIO_PIN_12			12
#define GPIO_PIN_13			13
#define GPIO_PIN_14			14
#define GPIO_PIN_15			15

/*
 * @GPIO_PIN_MODES
 * GPIO Pin Mode Macros
 */
#define GPIO_MODE_IN			0
#define GPIO_MODE_OUT			1
#define GPIO_MODE_ALTFN			2
#define GPIO_MODE_ANALOG		3
#define GPIO_MODE_INT_FT		4
#define GPIO_MODE_INT_RT		5
#define GPIO_MODE_INT_RFT		6

/*
 * @GPIO_OP_MODES
 * GPIO Output Modes
 */
#define GPIO_OP_PP				0
#define GPIO_OP_OD				1

/*
 * @GPIO_SPEED_MODES
 * GPIO Speed Modes
 */
#define GPIO_SPEED_LOW			0
#define GPIO_SPEED_MEDIUM		1
#define GPIO_SPEED_FAST			2
#define GPIO_SPEED_HIGH			3

/*
 * @GPIO_PUPD_CONTROL
 * GPIO Pull-up Pull-down Modes
 */
#define GPIO_NO_PUPD			0
#define GPIO_PIN_PU				1
#define GPIO_PIN_PD				2

/*******************************************************
 *                       APIs
 ******************************************************/

//GPIO Peripheral clock control
void GPIO_PCLK_Control(Gpio_Reg_t *pGPIOx, uint8_t ENorDI);

//APIs for Initialization of GPIO
void GPIO_Init(Gpio_Handle_t *pGPIOHandle);
void GPIO_DeInit(Gpio_Reg_t *pGPIOx);

//APIs for GPIO input and output operations
uint8_t GPIO_ReadPin(Gpio_Reg_t *pGPIOx, uint8_t pinNumber);
uint16_t GPIO_ReadPort(Gpio_Reg_t *pGPIOx);
void GPIO_WritePin(Gpio_Reg_t *pGPIOx, uint8_t pinNumber, uint8_t value);
void GPIO_WritePort(Gpio_Reg_t *pGPIOx, uint16_t value);
void GPIO_ToggleOutputPin(Gpio_Reg_t *pGPIOx, uint8_t pinNumber);

//APIs for GPIO interrupts
void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t ENorDI);
void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority);
void GPIO_IRQHandler(uint8_t pinNumber);


#endif /* INC_STM32F446RE_GPIO_DRIVER_H_ */
