/****************************************************************************
 *
 *   Copyright (c) 2021 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file board_config.h
 *
 * Board internal definitions
 */

#pragma once

/****************************************************************************************************
 * Included Files
 ****************************************************************************************************/

#include <px4_platform_common/px4_config.h>
#include <nuttx/compiler.h>
#include <stdint.h>

#include <stm32_gpio.h>

/****************************************************************************************************
 * Definitions
 ****************************************************************************************************/

#define FLASH_BASED_PARAMS


/* LEDs are active high. */

#define GPIO_LED_BLUE           /* PD10 */ (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTD|GPIO_PIN10)
#define GPIO_LED_GREEN          /* PA8  */ (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTA|GPIO_PIN8)
#define GPIO_LED_RED            /* PA10 */ (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTA|GPIO_PIN10)

#define BOARD_HAS_CONTROL_STATUS_LEDS   1
#define BOARD_OVERLOAD_LED              LED_RED
#define BOARD_ARMED_LED                 LED_BLUE
#define BOARD_ARMED_STATE_LED           LED_GREEN


/*
 * ADC channels
 *
 * These are the channel numbers of the ADCs of the microcontroller that
 * can be used by the Px4 Firmware in the adc driver
 */

/* ADC defines to be used in sensors.cpp to read from a particular channel */
#define ADC1_CH(n)                  (n)

/* Define GPIO pins used as ADC N.B. Channel numbers must match below  */
#define PX4_ADC_GPIO  \
	/* PA4  */  GPIO_ADC12_INP18_0

/* Define Channel numbers must match above GPIO pin IN(n)*/
#define ADC_BATTERY_VOLTAGE_CHANNEL     /* PA4  */  ADC1_CH(18)
#define ADC_BATTERY_CURRENT_CHANNEL     (-1)

#define ADC_CHANNELS \
	(1 << ADC_BATTERY_VOLTAGE_CHANNEL)


/* Define Battery Voltage Divider and A per V */
#define BOARD_BATTERY1_V_DIV         (38.0f)


/* PWM
 */
#define DIRECT_PWM_OUTPUT_CHANNELS   16
#define DIRECT_INPUT_TIMER_CHANNELS  16

#define BOARD_HAS_PWM  DIRECT_PWM_OUTPUT_CHANNELS

#define GPIO_PWM_CHIP_ENABLE      /* PE15 */ (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTE|GPIO_PIN15)

#define LIS2ML_INTF_MODE_PIN      /* PB10 */ (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_SET|GPIO_PORTB|GPIO_PIN10)

/* Spare GPIO */

// #define GPIO_PG6                        /* PG6  */  (GPIO_INPUT|GPIO_PULLUP|GPIO_PORTG|GPIO_PIN6)
// #define GPIO_PD15                       /* PD15 */  (GPIO_INPUT|GPIO_FLOAT|GPIO_PORTD|GPIO_PIN15)
// #define GPIO_PG15                       /* PG15 */  (GPIO_INPUT|GPIO_PULLUP|GPIO_PORTG|GPIO_PIN15)


/* USB OTG FS uses PA11/PA12. No VBUS sensing GPIO is configured. */
#define BOARD_USB_VBUS_SENSE_DISABLED 1


/* High-resolution timer */
#define HRT_TIMER               8  /* use timer8 for the HRT */
#define HRT_TIMER_CHANNEL       3  /* use capture/compare channel 3 */


/* RC Serial port */
#define RC_SERIAL_PORT          "/dev/ttyS6"
#define BOARD_SUPPORTS_RC_SERIAL_PORT_OUTPUT
/* SBUS is already inverted by external hardware, so no MCU UART inversion is needed. */
#define RC_INVERT_INPUT(_invert_true) ((void)(_invert_true))

// #define GPIO_RSSI_IN            /* PC5  */ (GPIO_INPUT|GPIO_PULLUP|GPIO_PORTC|GPIO_PIN5)


/* SD Card */
#define SDIO_SLOTNO             0  /* Only one slot */
#define SDIO_MINOR              0

/* This board provides a DMA pool and APIs */
#define BOARD_DMA_ALLOC_POOL_SIZE 5120

/* This board provides the board_on_reset interface */
#define BOARD_HAS_ON_RESET 1

#define PX4_GPIO_INIT_LIST { \
		PX4_ADC_GPIO, \
		GPIO_CAN1_TX, \
		GPIO_CAN1_RX, \
		GPIO_CAN2_TX, \
		GPIO_CAN2_RX, \
		GPIO_PWM_CHIP_ENABLE, \
		GPIO_LED_BLUE, \
		GPIO_LED_GREEN, \
		GPIO_LED_RED, \
		LIS2ML_INTF_MODE_PIN, \
	}

#define BOARD_ENABLE_CONSOLE_BUFFER

#define BOARD_NUM_IO_TIMERS 5


__BEGIN_DECLS

/****************************************************************************************************
 * Public Types
 ****************************************************************************************************/

/****************************************************************************************************
 * Public data
 ****************************************************************************************************/

#ifndef __ASSEMBLY__

/****************************************************************************************************
 * Public Functions
 ****************************************************************************************************/

/****************************************************************************
 * Name: stm32_sdio_initialize
 *
 * Description:
 *   Initialize SDIO-based MMC/SD card support
 *
 ****************************************************************************/

int stm32_sdio_initialize(void);

/****************************************************************************************************
 * Name: stm32_spiinitialize
 *
 * Description:
 *   Called to configure SPI chip select GPIO pins for the board.
 *
 ****************************************************************************************************/

extern void stm32_spiinitialize(void);

extern void stm32_usbinitialize(void);

extern void board_peripheral_reset(int ms);

#include <px4_platform_common/board_common.h>

#endif /* __ASSEMBLY__ */

__END_DECLS
