# STM32F446RE
STM32F446RE Drivers and Sample Codes

The STM32F446RE is a microcontroller from STMicroelectronics that is based on the ARM Cortex-M4 core. Here are some of its specifications:

- Core: ARM Cortex-M4F at 180 MHz
- Memory:
    512 KB Flash memory
    128 KB SRAM
    4 KB backup SRAM
- Connectivity:
    USB 2.0 full speed
    I2C, USART, SPI, CAN, and SDIO interfaces
- ADC:
    12-bit ADC with up to 20 channels
- Timers:
    2 general-purpose timers
    4 advanced-control timers
    2 basic timers
- Operating voltage range: 1.7V to 3.6V
- Operating temperature range: -40°C to 85°C
- Package: LQFP64 (10mm x 10mm)


## Folder Structure

### - Src
_Source foledr conatins main.c_

### - Drivers
_Folder contains MCU and perpheral related driver files_

> MCU Drivers : inc/stm32f446re.h

> Other Drivers:
 >- GPIO
 >- SPI
 >- I2C
 >- UART/USART

### - Tasks
_Folder contains sample codes to test driver files_

### - Appconfig
_Folder contains application configuration macros_
- Enable specific macros (uncomment) here to execute them.
- main.c will execute those macro related application task.