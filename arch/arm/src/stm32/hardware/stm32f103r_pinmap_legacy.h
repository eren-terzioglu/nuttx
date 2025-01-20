/****************************************************************************
 * arch/arm/src/stm32/hardware/stm32f103r_pinmap_legacy.h
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 ****************************************************************************/

#ifndef __ARCH_ARM_SRC_STM32_HARDWARE_STM32F103R_PINMAP_LEGACY_H
#define __ARCH_ARM_SRC_STM32_HARDWARE_STM32F103R_PINMAP_LEGACY_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "stm32_gpio.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* ADC */

#define GPIO_ADC123_IN0       (GPIO_INPUT|GPIO_CNF_ANALOGIN|GPIO_MODE_INPUT|GPIO_PORTA|GPIO_PIN0)
#define GPIO_ADC123_IN1       (GPIO_INPUT|GPIO_CNF_ANALOGIN|GPIO_MODE_INPUT|GPIO_PORTA|GPIO_PIN1)
#define GPIO_ADC123_IN2       (GPIO_INPUT|GPIO_CNF_ANALOGIN|GPIO_MODE_INPUT|GPIO_PORTA|GPIO_PIN2)
#define GPIO_ADC123_IN3       (GPIO_INPUT|GPIO_CNF_ANALOGIN|GPIO_MODE_INPUT|GPIO_PORTA|GPIO_PIN3)
#define GPIO_ADC12_IN4        (GPIO_INPUT|GPIO_CNF_ANALOGIN|GPIO_MODE_INPUT|GPIO_PORTA|GPIO_PIN4)
#define GPIO_ADC3_IN4         (GPIO_INPUT|GPIO_CNF_ANALOGIN|GPIO_MODE_INPUT|GPIO_PORTF|GPIO_PIN6)
#define GPIO_ADC12_IN5        (GPIO_INPUT|GPIO_CNF_ANALOGIN|GPIO_MODE_INPUT|GPIO_PORTA|GPIO_PIN5)
#define GPIO_ADC3_IN5         (GPIO_INPUT|GPIO_CNF_ANALOGIN|GPIO_MODE_INPUT|GPIO_PORTF|GPIO_PIN7)
#define GPIO_ADC12_IN6        (GPIO_INPUT|GPIO_CNF_ANALOGIN|GPIO_MODE_INPUT|GPIO_PORTA|GPIO_PIN6)
#define GPIO_ADC3_IN6         (GPIO_INPUT|GPIO_CNF_ANALOGIN|GPIO_MODE_INPUT|GPIO_PORTF|GPIO_PIN8)
#define GPIO_ADC12_IN7        (GPIO_INPUT|GPIO_CNF_ANALOGIN|GPIO_MODE_INPUT|GPIO_PORTA|GPIO_PIN7)
#define GPIO_ADC3_IN7         (GPIO_INPUT|GPIO_CNF_ANALOGIN|GPIO_MODE_INPUT|GPIO_PORTF|GPIO_PIN9)
#define GPIO_ADC12_IN8        (GPIO_INPUT|GPIO_CNF_ANALOGIN|GPIO_MODE_INPUT|GPIO_PORTB|GPIO_PIN0)
#define GPIO_ADC3_IN8         (GPIO_INPUT|GPIO_CNF_ANALOGIN|GPIO_MODE_INPUT|GPIO_PORTF|GPIO_PIN10)
#define GPIO_ADC12_IN9        (GPIO_INPUT|GPIO_CNF_ANALOGIN|GPIO_MODE_INPUT|GPIO_PORTB|GPIO_PIN1)
#define GPIO_ADC123_IN10      (GPIO_INPUT|GPIO_CNF_ANALOGIN|GPIO_MODE_INPUT|GPIO_PORTC|GPIO_PIN0)
#define GPIO_ADC123_IN11      (GPIO_INPUT|GPIO_CNF_ANALOGIN|GPIO_MODE_INPUT|GPIO_PORTC|GPIO_PIN1)
#define GPIO_ADC123_IN12      (GPIO_INPUT|GPIO_CNF_ANALOGIN|GPIO_MODE_INPUT|GPIO_PORTC|GPIO_PIN2)
#define GPIO_ADC123_IN13      (GPIO_INPUT|GPIO_CNF_ANALOGIN|GPIO_MODE_INPUT|GPIO_PORTC|GPIO_PIN3)
#define GPIO_ADC12_IN14       (GPIO_INPUT|GPIO_CNF_ANALOGIN|GPIO_MODE_INPUT|GPIO_PORTC|GPIO_PIN4)
#define GPIO_ADC12_IN15       (GPIO_INPUT|GPIO_CNF_ANALOGIN|GPIO_MODE_INPUT|GPIO_PORTC|GPIO_PIN5)

/* DAC - "Once the DAC channelx is enabled, the corresponding GPIO pin
 * (PA4 or PA5) is automatically connected to the analog converter output
 * (DAC_OUTx). In order to avoid parasitic consumption, the PA4 or PA5 pin
 * should first be configured to analog (AIN)."
 */

#define GPIO_DAC1_OUT1      (GPIO_INPUT|GPIO_CNF_ANALOGIN|GPIO_MODE_INPUT|GPIO_PORTA|GPIO_PIN4)
#define GPIO_DAC1_OUT2      (GPIO_INPUT|GPIO_CNF_ANALOGIN|GPIO_MODE_INPUT|GPIO_PORTA|GPIO_PIN5)

/* TIMERS */

#if defined(CONFIG_STM32_TIM1_FULL_REMAP)
#  define GPIO_TIM1_ETR     (GPIO_INPUT|GPIO_CNF_INFLOAT|GPIO_MODE_INPUT|GPIO_PORTE|GPIO_PIN7)
#  define GPIO_TIM1_CH1IN   (GPIO_INPUT|GPIO_CNF_INFLOAT|GPIO_MODE_INPUT|GPIO_PORTE|GPIO_PIN9)
#  define GPIO_TIM1_CH1OUT  (GPIO_ALT|GPIO_CNF_AFPP|GPIO_MODE_50MHz|GPIO_PORTE|GPIO_PIN9)
#  define GPIO_TIM1_CH2IN   (GPIO_INPUT|GPIO_CNF_INFLOAT|GPIO_MODE_INPUT|GPIO_PORTE|GPIO_PIN11)
#  define GPIO_TIM1_CH2OUT  (GPIO_ALT|GPIO_CNF_AFPP|GPIO_MODE_50MHz|GPIO_PORTE|GPIO_PIN11)
#  define GPIO_TIM1_CH3IN   (GPIO_INPUT|GPIO_CNF_INFLOAT|GPIO_MODE_INPUT|GPIO_PORTE|GPIO_PIN13)
#  define GPIO_TIM1_CH3OUT  (GPIO_ALT|GPIO_CNF_AFPP|GPIO_MODE_50MHz|GPIO_PORTE|GPIO_PIN13)
#  define GPIO_TIM1_CH4IN   (GPIO_INPUT|GPIO_CNF_INFLOAT|GPIO_MODE_INPUT|GPIO_PORTE|GPIO_PIN14)
#  define GPIO_TIM1_CH4OUT  (GPIO_ALT|GPIO_CNF_AFPP|GPIO_MODE_50MHz|GPIO_PORTE|GPIO_PIN14)
#  define GPIO_TIM1_BKIN    (GPIO_INPUT|GPIO_CNF_INFLOAT|GPIO_MODE_INPUT|GPIO_PORTE|GPIO_PIN15)
#  define GPIO_TIM1_CH1NOUT (GPIO_ALT|GPIO_CNF_AFPP|GPIO_MODE_50MHz|GPIO_PORTE|GPIO_PIN8)
#  define GPIO_TIM1_CH2NOUT (GPIO_ALT|GPIO_CNF_AFPP|GPIO_MODE_50MHz|GPIO_PORTE|GPIO_PIN10)
#  define GPIO_TIM1_CH3NOUT (GPIO_ALT|GPIO_CNF_AFPP|GPIO_MODE_50MHz|GPIO_PORTE|GPIO_PIN12)
#elif defined(CONFIG_STM32_TIM1_PARTIAL_REMAP)
#  define GPIO_TIM1_ETR     (GPIO_INPUT|GPIO_CNF_INFLOAT|GPIO_MODE_INPUT|GPIO_PORTA|GPIO_PIN12)
#  define GPIO_TIM1_CH1IN   (GPIO_INPUT|GPIO_CNF_INFLOAT|GPIO_MODE_INPUT|GPIO_PORTA|GPIO_PIN8)
#  define GPIO_TIM1_CH1OUT  (GPIO_ALT|GPIO_CNF_AFPP|GPIO_MODE_50MHz|GPIO_PORTA|GPIO_PIN8)
#  define GPIO_TIM1_CH2IN   (GPIO_INPUT|GPIO_CNF_INFLOAT|GPIO_MODE_INPUT|GPIO_PORTA|GPIO_PIN9)
#  define GPIO_TIM1_CH2OUT  (GPIO_ALT|GPIO_CNF_AFPP|GPIO_MODE_50MHz|GPIO_PORTA|GPIO_PIN9)
#  define GPIO_TIM1_CH3IN   (GPIO_INPUT|GPIO_CNF_INFLOAT|GPIO_MODE_INPUT|GPIO_PORTA|GPIO_PIN10)
#  define GPIO_TIM1_CH3OUT  (GPIO_ALT|GPIO_CNF_AFPP|GPIO_MODE_50MHz|GPIO_PORTA|GPIO_PIN10)
#  define GPIO_TIM1_CH4IN   (GPIO_INPUT|GPIO_CNF_INFLOAT|GPIO_MODE_INPUT|GPIO_PORTA|GPIO_PIN11)
#  define GPIO_TIM1_CH4OUT  (GPIO_ALT|GPIO_CNF_AFPP|GPIO_MODE_50MHz|GPIO_PORTA|GPIO_PIN11)
#  define GPIO_TIM1_BKIN    (GPIO_INPUT|GPIO_CNF_INFLOAT|GPIO_MODE_INPUT|GPIO_PORTA|GPIO_PIN6)
#  define GPIO_TIM1_CH1NOUT (GPIO_ALT|GPIO_CNF_AFPP|GPIO_MODE_50MHz|GPIO_PORTA|GPIO_PIN7)
#  define GPIO_TIM1_CH2NOUT (GPIO_ALT|GPIO_CNF_AFPP|GPIO_MODE_50MHz|GPIO_PORTB|GPIO_PIN0)
#  define GPIO_TIM1_CH3NOUT (GPIO_ALT|GPIO_CNF_AFPP|GPIO_MODE_50MHz|GPIO_PORTB|GPIO_PIN1)
#else
#  define GPIO_TIM1_ETR     (GPIO_INPUT|GPIO_CNF_INFLOAT|GPIO_MODE_INPUT|GPIO_PORTA|GPIO_PIN12)
#  define GPIO_TIM1_CH1IN   (GPIO_INPUT|GPIO_CNF_INFLOAT|GPIO_MODE_INPUT|GPIO_PORTA|GPIO_PIN8)
#  define GPIO_TIM1_CH1OUT  (GPIO_ALT|GPIO_CNF_AFPP|GPIO_MODE_50MHz|GPIO_PORTA|GPIO_PIN8)
#  define GPIO_TIM1_CH2IN   (GPIO_INPUT|GPIO_CNF_INFLOAT|GPIO_MODE_INPUT|GPIO_PORTA|GPIO_PIN9)
#  define GPIO_TIM1_CH2OUT  (GPIO_ALT|GPIO_CNF_AFPP|GPIO_MODE_50MHz|GPIO_PORTA|GPIO_PIN9)
#  define GPIO_TIM1_CH3IN   (GPIO_INPUT|GPIO_CNF_INFLOAT|GPIO_MODE_INPUT|GPIO_PORTA|GPIO_PIN10)
#  define GPIO_TIM1_CH3OUT  (GPIO_ALT|GPIO_CNF_AFPP|GPIO_MODE_50MHz|GPIO_PORTA|GPIO_PIN10)
#  define GPIO_TIM1_CH4IN   (GPIO_INPUT|GPIO_CNF_INFLOAT|GPIO_MODE_INPUT|GPIO_PORTA|GPIO_PIN11)
#  define GPIO_TIM1_CH4OUT  (GPIO_ALT|GPIO_CNF_AFPP|GPIO_MODE_50MHz|GPIO_PORTA|GPIO_PIN11)
#  define GPIO_TIM1_BKIN    (GPIO_INPUT|GPIO_CNF_INFLOAT|GPIO_MODE_INPUT|GPIO_PORTB|GPIO_PIN12)
#  define GPIO_TIM1_CH1NOUT (GPIO_ALT|GPIO_CNF_AFPP|GPIO_MODE_50MHz|GPIO_PORTB|GPIO_PIN13)
#  define GPIO_TIM1_CH2NOUT (GPIO_ALT|GPIO_CNF_AFPP|GPIO_MODE_50MHz|GPIO_PORTB|GPIO_PIN14)
#  define GPIO_TIM1_CH3NOUT (GPIO_ALT|GPIO_CNF_AFPP|GPIO_MODE_50MHz|GPIO_PORTB|GPIO_PIN15)
#endif

#if defined(CONFIG_STM32_TIM2_FULL_REMAP)
#  define GPIO_TIM2_ETR     (GPIO_INPUT|GPIO_CNF_INFLOAT|GPIO_MODE_INPUT|GPIO_PORTA|GPIO_PIN15)
#  define GPIO_TIM2_CH1IN   (GPIO_INPUT|GPIO_CNF_INFLOAT|GPIO_MODE_INPUT|GPIO_PORTA|GPIO_PIN15)
#  define GPIO_TIM2_CH1OUT  (GPIO_ALT|GPIO_CNF_AFPP|GPIO_MODE_50MHz|GPIO_PORTA|GPIO_PIN15)
#  define GPIO_TIM2_CH2IN   (GPIO_INPUT|GPIO_CNF_INFLOAT|GPIO_MODE_INPUT|GPIO_PORTB|GPIO_PIN3)
#  define GPIO_TIM2_CH2OUT  (GPIO_ALT|GPIO_CNF_AFPP|GPIO_MODE_50MHz|GPIO_PORTB|GPIO_PIN3)
#  define GPIO_TIM2_CH3IN   (GPIO_INPUT|GPIO_CNF_INFLOAT|GPIO_MODE_INPUT|GPIO_PORTB|GPIO_PIN10)
#  define GPIO_TIM2_CH3OUT  (GPIO_ALT|GPIO_CNF_AFPP|GPIO_MODE_50MHz|GPIO_PORTB|GPIO_PIN10)
#  define GPIO_TIM2_CH4IN   (GPIO_INPUT|GPIO_CNF_INFLOAT|GPIO_MODE_INPUT|GPIO_PORTB|GPIO_PIN11)
#  define GPIO_TIM2_CH4OUT  (GPIO_ALT|GPIO_CNF_AFPP|GPIO_MODE_50MHz|GPIO_PORTB|GPIO_PIN11)
#elif defined(CONFIG_STM32_TIM2_PARTIAL_REMAP_1)
#  define GPIO_TIM2_ETR     (GPIO_INPUT|GPIO_CNF_INFLOAT|GPIO_MODE_INPUT|GPIO_PORTA|GPIO_PIN15)
#  define GPIO_TIM2_CH1IN   (GPIO_INPUT|GPIO_CNF_INFLOAT|GPIO_MODE_INPUT|GPIO_PORTA|GPIO_PIN15)
#  define GPIO_TIM2_CH1OUT  (GPIO_ALT|GPIO_CNF_AFPP|GPIO_MODE_50MHz|GPIO_PORTA|GPIO_PIN15)
#  define GPIO_TIM2_CH2IN   (GPIO_INPUT|GPIO_CNF_INFLOAT|GPIO_MODE_INPUT|GPIO_PORTB|GPIO_PIN3)
#  define GPIO_TIM2_CH2OUT  (GPIO_ALT|GPIO_CNF_AFPP|GPIO_MODE_50MHz|GPIO_PORTB|GPIO_PIN3)
#  define GPIO_TIM2_CH3IN   (GPIO_INPUT|GPIO_CNF_INFLOAT|GPIO_MODE_INPUT|GPIO_PORTA|GPIO_PIN2)
#  define GPIO_TIM2_CH3OUT  (GPIO_ALT|GPIO_CNF_AFPP|GPIO_MODE_50MHz|GPIO_PORTA|GPIO_PIN2)
#  define GPIO_TIM2_CH4IN   (GPIO_INPUT|GPIO_CNF_INFLOAT|GPIO_MODE_INPUT|GPIO_PORTA|GPIO_PIN3)
#  define GPIO_TIM2_CH4OUT  (GPIO_ALT|GPIO_CNF_AFPP|GPIO_MODE_50MHz|GPIO_PORTA|GPIO_PIN3)
#elif defined(CONFIG_STM32_TIM2_PARTIAL_REMAP_2)
#  define GPIO_TIM2_ETR     (GPIO_INPUT|GPIO_CNF_INFLOAT|GPIO_MODE_INPUT|GPIO_PORTA|GPIO_PIN0)
#  define GPIO_TIM2_CH1IN   (GPIO_INPUT|GPIO_CNF_INFLOAT|GPIO_MODE_INPUT|GPIO_PORTA|GPIO_PIN0)
#  define GPIO_TIM2_CH1OUT  (GPIO_ALT|GPIO_CNF_AFPP|GPIO_MODE_50MHz|GPIO_PORTA|GPIO_PIN0)
#  define GPIO_TIM2_CH2IN   (GPIO_INPUT|GPIO_CNF_INFLOAT|GPIO_MODE_INPUT|GPIO_PORTA|GPIO_PIN1)
#  define GPIO_TIM2_CH2OUT  (GPIO_ALT|GPIO_CNF_AFPP|GPIO_MODE_50MHz|GPIO_PORTA|GPIO_PIN1)
#  define GPIO_TIM2_CH3IN   (GPIO_INPUT|GPIO_CNF_INFLOAT|GPIO_MODE_INPUT|GPIO_PORTB|GPIO_PIN10)
#  define GPIO_TIM2_CH3OUT  (GPIO_ALT|GPIO_CNF_AFPP|GPIO_MODE_50MHz|GPIO_PORTB|GPIO_PIN10)
#  define GPIO_TIM2_CH4IN   (GPIO_INPUT|GPIO_CNF_INFLOAT|GPIO_MODE_INPUT|GPIO_PORTB|GPIO_PIN11)
#  define GPIO_TIM2_CH4OUT  (GPIO_ALT|GPIO_CNF_AFPP|GPIO_MODE_50MHz|GPIO_PORTB|GPIO_PIN11)
#else
#  define GPIO_TIM2_ETR     (GPIO_INPUT|GPIO_CNF_INFLOAT|GPIO_MODE_INPUT|GPIO_PORTA|GPIO_PIN0)
#  define GPIO_TIM2_CH1IN   (GPIO_INPUT|GPIO_CNF_INFLOAT|GPIO_MODE_INPUT|GPIO_PORTA|GPIO_PIN0)
#  define GPIO_TIM2_CH1OUT  (GPIO_ALT|GPIO_CNF_AFPP|GPIO_MODE_50MHz|GPIO_PORTA|GPIO_PIN0)
#  define GPIO_TIM2_CH2IN   (GPIO_INPUT|GPIO_CNF_INFLOAT|GPIO_MODE_INPUT|GPIO_PORTA|GPIO_PIN1)
#  define GPIO_TIM2_CH2OUT  (GPIO_ALT|GPIO_CNF_AFPP|GPIO_MODE_50MHz|GPIO_PORTA|GPIO_PIN1)
#  define GPIO_TIM2_CH3IN   (GPIO_INPUT|GPIO_CNF_INFLOAT|GPIO_MODE_INPUT|GPIO_PORTA|GPIO_PIN2)
#  define GPIO_TIM2_CH3OUT  (GPIO_ALT|GPIO_CNF_AFPP|GPIO_MODE_50MHz|GPIO_PORTA|GPIO_PIN2)
#  define GPIO_TIM2_CH4IN   (GPIO_INPUT|GPIO_CNF_INFLOAT|GPIO_MODE_INPUT|GPIO_PORTA|GPIO_PIN3)
#  define GPIO_TIM2_CH4OUT  (GPIO_ALT|GPIO_CNF_AFPP|GPIO_MODE_50MHz|GPIO_PORTA|GPIO_PIN3)
#endif

#if defined(CONFIG_STM32_TIM3_FULL_REMAP)
#  define GPIO_TIM3_CH1IN   (GPIO_INPUT|GPIO_CNF_INFLOAT|GPIO_MODE_INPUT|GPIO_PORTC|GPIO_PIN6)
#  define GPIO_TIM3_CH1OUT  (GPIO_ALT|GPIO_CNF_AFPP|GPIO_MODE_50MHz|GPIO_PORTC|GPIO_PIN6)
#  define GPIO_TIM3_CH2IN   (GPIO_INPUT|GPIO_CNF_INFLOAT|GPIO_MODE_INPUT|GPIO_PORTC|GPIO_PIN7)
#  define GPIO_TIM3_CH2OUT  (GPIO_ALT|GPIO_CNF_AFPP|GPIO_MODE_50MHz|GPIO_PORTC|GPIO_PIN7)
#  define GPIO_TIM3_CH3IN   (GPIO_INPUT|GPIO_CNF_INFLOAT|GPIO_MODE_INPUT|GPIO_PORTC|GPIO_PIN8)
#  define GPIO_TIM3_CH3OUT  (GPIO_ALT|GPIO_CNF_AFPP|GPIO_MODE_50MHz|GPIO_PORTC|GPIO_PIN8)
#  define GPIO_TIM3_CH4IN   (GPIO_INPUT|GPIO_CNF_INFLOAT|GPIO_MODE_INPUT|GPIO_PORTC|GPIO_PIN9)
#  define GPIO_TIM3_CH4OUT  (GPIO_ALT|GPIO_CNF_AFPP|GPIO_MODE_50MHz|GPIO_PORTC|GPIO_PIN9)
#elif defined(CONFIG_STM32_TIM3_PARTIAL_REMAP)
#  define GPIO_TIM3_CH1IN   (GPIO_INPUT|GPIO_CNF_INFLOAT|GPIO_MODE_INPUT|GPIO_PORTB|GPIO_PIN4)
#  define GPIO_TIM3_CH1OUT  (GPIO_ALT|GPIO_CNF_AFPP|GPIO_MODE_50MHz|GPIO_PORTB|GPIO_PIN4)
#  define GPIO_TIM3_CH2IN   (GPIO_INPUT|GPIO_CNF_INFLOAT|GPIO_MODE_INPUT|GPIO_PORTB|GPIO_PIN5)
#  define GPIO_TIM3_CH2OUT  (GPIO_ALT|GPIO_CNF_AFPP|GPIO_MODE_50MHz|GPIO_PORTB|GPIO_PIN5)
#  define GPIO_TIM3_CH3IN   (GPIO_INPUT|GPIO_CNF_INFLOAT|GPIO_MODE_INPUT|GPIO_PORTB|GPIO_PIN0)
#  define GPIO_TIM3_CH3OUT  (GPIO_ALT|GPIO_CNF_AFPP|GPIO_MODE_50MHz|GPIO_PORTB|GPIO_PIN0)
#  define GPIO_TIM3_CH4IN   (GPIO_INPUT|GPIO_CNF_INFLOAT|GPIO_MODE_INPUT|GPIO_PORTB|GPIO_PIN1)
#  define GPIO_TIM3_CH4OUT  (GPIO_ALT|GPIO_CNF_AFPP|GPIO_MODE_50MHz|GPIO_PORTB|GPIO_PIN1)
#else
#  define GPIO_TIM3_CH1IN   (GPIO_INPUT|GPIO_CNF_INFLOAT|GPIO_MODE_INPUT|GPIO_PORTA|GPIO_PIN6)
#  define GPIO_TIM3_CH1OUT  (GPIO_ALT|GPIO_CNF_AFPP|GPIO_MODE_50MHz|GPIO_PORTA|GPIO_PIN6)
#  define GPIO_TIM3_CH2IN   (GPIO_INPUT|GPIO_CNF_INFLOAT|GPIO_MODE_INPUT|GPIO_PORTA|GPIO_PIN7)
#  define GPIO_TIM3_CH2OUT  (GPIO_ALT|GPIO_CNF_AFPP|GPIO_MODE_50MHz|GPIO_PORTA|GPIO_PIN7)
#  define GPIO_TIM3_CH3IN   (GPIO_INPUT|GPIO_CNF_INFLOAT|GPIO_MODE_INPUT|GPIO_PORTB|GPIO_PIN0)
#  define GPIO_TIM3_CH3OUT  (GPIO_ALT|GPIO_CNF_AFPP|GPIO_MODE_50MHz|GPIO_PORTB|GPIO_PIN0)
#  define GPIO_TIM3_CH4IN   (GPIO_INPUT|GPIO_CNF_INFLOAT|GPIO_MODE_INPUT|GPIO_PORTB|GPIO_PIN1)
#  define GPIO_TIM3_CH4OUT  (GPIO_ALT|GPIO_CNF_AFPP|GPIO_MODE_50MHz|GPIO_PORTB|GPIO_PIN1)
#endif

#if defined(CONFIG_STM32_TIM4_REMAP)
#  define GPIO_TIM4_CH1IN   (GPIO_INPUT|GPIO_CNF_INFLOAT|GPIO_MODE_INPUT|GPIO_PORTD|GPIO_PIN12)
#  define GPIO_TIM4_CH1OUT  (GPIO_ALT|GPIO_CNF_AFPP|GPIO_MODE_50MHz|GPIO_PORTD|GPIO_PIN12)
#  define GPIO_TIM4_CH2IN   (GPIO_INPUT|GPIO_CNF_INFLOAT|GPIO_MODE_INPUT|GPIO_PORTD|GPIO_PIN13)
#  define GPIO_TIM4_CH2OUT  (GPIO_ALT|GPIO_CNF_AFPP|GPIO_MODE_50MHz|GPIO_PORTD|GPIO_PIN13)
#  define GPIO_TIM4_CH3IN   (GPIO_INPUT|GPIO_CNF_INFLOAT|GPIO_MODE_INPUT|GPIO_PORTD|GPIO_PIN14)
#  define GPIO_TIM4_CH3OUT  (GPIO_ALT|GPIO_CNF_AFPP|GPIO_MODE_50MHz|GPIO_PORTD|GPIO_PIN14)
#  define GPIO_TIM4_CH4IN   (GPIO_INPUT|GPIO_CNF_INFLOAT|GPIO_MODE_INPUT|GPIO_PORTD|GPIO_PIN15)
#  define GPIO_TIM4_CH4OUT  (GPIO_ALT|GPIO_CNF_AFPP|GPIO_MODE_50MHz|GPIO_PORTD|GPIO_PIN15)
#else
#  define GPIO_TIM4_CH1IN   (GPIO_INPUT|GPIO_CNF_INFLOAT|GPIO_MODE_INPUT|GPIO_PORTB|GPIO_PIN6)
#  define GPIO_TIM4_CH1OUT  (GPIO_ALT|GPIO_CNF_AFPP|GPIO_MODE_50MHz|GPIO_PORTB|GPIO_PIN6)
#  define GPIO_TIM4_CH2IN   (GPIO_INPUT|GPIO_CNF_INFLOAT|GPIO_MODE_INPUT|GPIO_PORTB|GPIO_PIN7)
#  define GPIO_TIM4_CH2OUT  (GPIO_ALT|GPIO_CNF_AFPP|GPIO_MODE_50MHz|GPIO_PORTB|GPIO_PIN7)
#  define GPIO_TIM4_CH3IN   (GPIO_INPUT|GPIO_CNF_INFLOAT|GPIO_MODE_INPUT|GPIO_PORTB|GPIO_PIN8)
#  define GPIO_TIM4_CH3OUT  (GPIO_ALT|GPIO_CNF_AFPP|GPIO_MODE_50MHz|GPIO_PORTB|GPIO_PIN8)
#  define GPIO_TIM4_CH4IN   (GPIO_INPUT|GPIO_CNF_INFLOAT|GPIO_MODE_INPUT|GPIO_PORTB|GPIO_PIN9)
#  define GPIO_TIM4_CH4OUT  (GPIO_ALT|GPIO_CNF_AFPP|GPIO_MODE_50MHz|GPIO_PORTB|GPIO_PIN9)
#endif

#define GPIO_TIM5_CH1IN     (GPIO_INPUT|GPIO_CNF_INFLOAT|GPIO_MODE_INPUT|GPIO_PORTA|GPIO_PIN0)
#define GPIO_TIM5_CH1OUT    (GPIO_ALT|GPIO_CNF_AFPP|GPIO_MODE_50MHz|GPIO_PORTA|GPIO_PIN0)
#define GPIO_TIM5_CH2IN     (GPIO_INPUT|GPIO_CNF_INFLOAT|GPIO_MODE_INPUT|GPIO_PORTA|GPIO_PIN1)
#define GPIO_TIM5_CH2OUT    (GPIO_ALT|GPIO_CNF_AFPP|GPIO_MODE_50MHz|GPIO_PORTA|GPIO_PIN1)
#define GPIO_TIM5_CH3IN     (GPIO_INPUT|GPIO_CNF_INFLOAT|GPIO_MODE_INPUT|GPIO_PORTA|GPIO_PIN2)
#define GPIO_TIM5_CH3OUT    (GPIO_ALT|GPIO_CNF_AFPP|GPIO_MODE_50MHz|GPIO_PORTA|GPIO_PIN2)
#define GPIO_TIM5_CH4IN     (GPIO_INPUT|GPIO_CNF_INFLOAT|GPIO_MODE_INPUT|GPIO_PORTA|GPIO_PIN3)
#define GPIO_TIM5_CH4OUT    (GPIO_ALT|GPIO_CNF_AFPP|GPIO_MODE_50MHz|GPIO_PORTA|GPIO_PIN3)

#define GPIO_TIM8_ETR       (GPIO_INPUT|GPIO_CNF_INFLOAT|GPIO_MODE_INPUT|GPIO_PORTA|GPIO_PIN0)
#define GPIO_TIM8_CH1IN     (GPIO_INPUT|GPIO_CNF_INFLOAT|GPIO_MODE_INPUT|GPIO_PORTC|GPIO_PIN6)
#define GPIO_TIM8_CH1OUT    (GPIO_ALT|GPIO_CNF_AFPP|GPIO_MODE_50MHz|GPIO_PORTC|GPIO_PIN6)
#define GPIO_TIM8_CH2IN     (GPIO_INPUT|GPIO_CNF_INFLOAT|GPIO_MODE_INPUT|GPIO_PORTC|GPIO_PIN7)
#define GPIO_TIM8_CH2OUT    (GPIO_ALT|GPIO_CNF_AFPP|GPIO_MODE_50MHz|GPIO_PORTC|GPIO_PIN7)
#define GPIO_TIM8_CH3IN     (GPIO_INPUT|GPIO_CNF_INFLOAT|GPIO_MODE_INPUT|GPIO_PORTC|GPIO_PIN8)
#define GPIO_TIM8_CH3OUT    (GPIO_ALT|GPIO_CNF_AFPP|GPIO_MODE_50MHz|GPIO_PORTC|GPIO_PIN8)
#define GPIO_TIM8_CH4IN     (GPIO_INPUT|GPIO_CNF_INFLOAT|GPIO_MODE_INPUT|GPIO_PORTC|GPIO_PIN9)
#define GPIO_TIM8_CH4OUT    (GPIO_ALT|GPIO_CNF_AFPP|GPIO_MODE_50MHz|GPIO_PORTC|GPIO_PIN9)
#define GPIO_TIM8_BKIN      (GPIO_INPUT|GPIO_CNF_INFLOAT|GPIO_MODE_INPUT|GPIO_PORTA|GPIO_PIN6)
#define GPIO_TIM8_CH1NOUT   (GPIO_ALT|GPIO_CNF_AFPP|GPIO_MODE_50MHz|GPIO_PORTA|GPIO_PIN7)
#define GPIO_TIM8_CH2NOUT   (GPIO_ALT|GPIO_CNF_AFPP|GPIO_MODE_50MHz|GPIO_PORTB|GPIO_PIN0)
#define GPIO_TIM8_CH3NOUT   (GPIO_ALT|GPIO_CNF_AFPP|GPIO_MODE_50MHz|GPIO_PORTB|GPIO_PIN1)

/* USART */

#if defined(CONFIG_STM32_USART1_REMAP)
#  define GPIO_USART1_TX    (GPIO_ALT|GPIO_CNF_AFPP|GPIO_MODE_50MHz|GPIO_PORTB|GPIO_PIN6)
#  define GPIO_USART1_RX    (GPIO_INPUT|GPIO_CNF_INFLOAT|GPIO_MODE_INPUT|GPIO_PORTB|GPIO_PIN7)
#else
#  define GPIO_USART1_TX    (GPIO_ALT|GPIO_CNF_AFPP|GPIO_MODE_50MHz|GPIO_PORTA|GPIO_PIN9)
#  define GPIO_USART1_RX    (GPIO_INPUT|GPIO_CNF_INFLOAT|GPIO_MODE_INPUT|GPIO_PORTA|GPIO_PIN10)
#endif

#if defined(CONFIG_STM32_USART2_REMAP)
#  define GPIO_USART2_CTS   (GPIO_INPUT|GPIO_CNF_INFLOAT|GPIO_MODE_INPUT|GPIO_PORTD|GPIO_PIN3)
#  define GPIO_USART2_RTS   (GPIO_ALT|GPIO_CNF_AFPP|GPIO_MODE_50MHz|GPIO_PORTD|GPIO_PIN4)
#  define GPIO_USART2_TX    (GPIO_ALT|GPIO_CNF_AFPP|GPIO_MODE_50MHz|GPIO_PORTD|GPIO_PIN5)
#  define GPIO_USART2_RX    (GPIO_INPUT|GPIO_CNF_INFLOAT|GPIO_MODE_INPUT|GPIO_PORTD|GPIO_PIN6)
#  define GPIO_USART2_CK    (GPIO_ALT|GPIO_CNF_AFPP|GPIO_MODE_50MHz|GPIO_PORTD|GPIO_PIN7)
#else
#  define GPIO_USART2_CTS   (GPIO_INPUT|GPIO_CNF_INFLOAT|GPIO_MODE_INPUT|GPIO_PORTA|GPIO_PIN0)
#  define GPIO_USART2_RTS   (GPIO_ALT|GPIO_CNF_AFPP|GPIO_MODE_50MHz|GPIO_PORTA|GPIO_PIN1)
#  define GPIO_USART2_TX    (GPIO_ALT|GPIO_CNF_AFPP|GPIO_MODE_50MHz|GPIO_PORTA|GPIO_PIN2)
#  define GPIO_USART2_RX    (GPIO_INPUT|GPIO_CNF_INFLOAT|GPIO_MODE_INPUT|GPIO_PORTA|GPIO_PIN3)
#  define GPIO_USART2_CK    (GPIO_ALT|GPIO_CNF_AFPP|GPIO_MODE_50MHz|GPIO_PORTA|GPIO_PIN4)
#endif

#if defined(CONFIG_STM32_USART3_FULL_REMAP)
#  define GPIO_USART3_TX    (GPIO_ALT|GPIO_CNF_AFPP|GPIO_MODE_50MHz|GPIO_PORTD|GPIO_PIN8)
#  define GPIO_USART3_RX    (GPIO_INPUT|GPIO_CNF_INFLOAT|GPIO_MODE_INPUT|GPIO_PORTD|GPIO_PIN9)
#  define GPIO_USART3_CK    (GPIO_ALT|GPIO_CNF_AFPP|GPIO_MODE_50MHz|GPIO_PORTD|GPIO_PIN10)
#  define GPIO_USART3_CTS   (GPIO_INPUT|GPIO_CNF_INFLOAT|GPIO_MODE_INPUT|GPIO_PORTD|GPIO_PIN11)
#  define GPIO_USART3_RTS   (GPIO_ALT|GPIO_CNF_AFPP|GPIO_MODE_50MHz|GPIO_PORTD|GPIO_PIN12)
#elif defined(CONFIG_STM32_USART3_PARTIAL_REMAP)
#  define GPIO_USART3_TX    (GPIO_ALT|GPIO_CNF_AFPP|GPIO_MODE_50MHz|GPIO_PORTC|GPIO_PIN10)
#  define GPIO_USART3_RX    (GPIO_INPUT|GPIO_CNF_INFLOAT|GPIO_MODE_INPUT|GPIO_PORTC|GPIO_PIN11)
#  define GPIO_USART3_CK    (GPIO_ALT|GPIO_CNF_AFPP|GPIO_MODE_50MHz|GPIO_PORTC|GPIO_PIN12)
#  define GPIO_USART3_CTS   (GPIO_INPUT|GPIO_CNF_INFLOAT|GPIO_MODE_INPUT|GPIO_PORTB|GPIO_PIN13)
#  define GPIO_USART3_RTS   (GPIO_ALT|GPIO_CNF_AFPP|GPIO_MODE_50MHz|GPIO_PORTB|GPIO_PIN14)
#else
#  define GPIO_USART3_TX    (GPIO_ALT|GPIO_CNF_AFPP|GPIO_MODE_50MHz|GPIO_PORTB|GPIO_PIN10)
#  define GPIO_USART3_RX    (GPIO_INPUT|GPIO_CNF_INFLOAT|GPIO_MODE_INPUT|GPIO_PORTB|GPIO_PIN11)
#  define GPIO_USART3_CK    (GPIO_ALT|GPIO_CNF_AFPP|GPIO_MODE_50MHz|GPIO_PORTB|GPIO_PIN12)
#  define GPIO_USART3_CTS   (GPIO_INPUT|GPIO_CNF_INFLOAT|GPIO_MODE_INPUT|GPIO_PORTB|GPIO_PIN13)
#  define GPIO_USART3_RTS   (GPIO_ALT|GPIO_CNF_AFPP|GPIO_MODE_50MHz|GPIO_PORTB|GPIO_PIN14)
#endif

#define GPIO_UART4_RX       (GPIO_INPUT|GPIO_CNF_INFLOAT|GPIO_MODE_INPUT|GPIO_PORTC|GPIO_PIN11)
#define GPIO_UART4_TX       (GPIO_ALT|GPIO_CNF_AFPP|GPIO_MODE_50MHz|GPIO_PORTC|GPIO_PIN10)

#define GPIO_UART5_RX       (GPIO_INPUT|GPIO_CNF_INFLOAT|GPIO_MODE_INPUT|GPIO_PORTD|GPIO_PIN2)
#define GPIO_UART5_TX       (GPIO_ALT|GPIO_CNF_AFPP|GPIO_MODE_50MHz|GPIO_PORTC|GPIO_PIN12)

/* SPI */

#if defined(CONFIG_STM32_SPI1_REMAP)
#  define GPIO_SPI1_NSS     (GPIO_INPUT|GPIO_CNF_INFLOAT|GPIO_MODE_INPUT|GPIO_PORTA|GPIO_PIN15)
#  define GPIO_SPI1_SCK     (GPIO_ALT|GPIO_CNF_AFPP|GPIO_MODE_50MHz|GPIO_PORTB|GPIO_PIN3)
#  define GPIO_SPI1_MISO    (GPIO_INPUT|GPIO_CNF_INFLOAT|GPIO_MODE_INPUT|GPIO_PORTB|GPIO_PIN4)
#  define GPIO_SPI1_MOSI    (GPIO_ALT|GPIO_CNF_AFPP|GPIO_MODE_50MHz|GPIO_PORTB|GPIO_PIN5)
#else
#  define GPIO_SPI1_NSS     (GPIO_INPUT|GPIO_CNF_AFPP|GPIO_MODE_50MHz|GPIO_PORTA|GPIO_PIN4)
#  define GPIO_SPI1_SCK     (GPIO_ALT|GPIO_CNF_AFPP|GPIO_MODE_50MHz|GPIO_PORTA|GPIO_PIN5)
#  define GPIO_SPI1_MISO    (GPIO_INPUT|GPIO_CNF_AFPP|GPIO_MODE_50MHz|GPIO_PORTA|GPIO_PIN6)
#  define GPIO_SPI1_MOSI    (GPIO_ALT|GPIO_CNF_AFPP|GPIO_MODE_50MHz|GPIO_PORTA|GPIO_PIN7)
#endif

#define GPIO_SPI2_NSS       (GPIO_INPUT|GPIO_CNF_AFPP|GPIO_MODE_50MHz|GPIO_PORTB|GPIO_PIN12)
#define GPIO_SPI2_SCK       (GPIO_ALT|GPIO_CNF_AFPP|GPIO_MODE_50MHz|GPIO_PORTB|GPIO_PIN13)
#define GPIO_SPI2_MISO      (GPIO_INPUT|GPIO_CNF_AFPP|GPIO_MODE_50MHz|GPIO_PORTB|GPIO_PIN14)
#define GPIO_SPI2_MOSI      (GPIO_ALT|GPIO_CNF_AFPP|GPIO_MODE_50MHz|GPIO_PORTB|GPIO_PIN15)

#if defined(CONFIG_STM32_SPI3_REMAP)
#  define GPIO_SPI3_NSS     (GPIO_INPUT|GPIO_CNF_INFLOAT|GPIO_MODE_INPUT|GPIO_PORTA|GPIO_PIN4)
#  define GPIO_SPI3_SCK     (GPIO_ALT|GPIO_CNF_AFPP|GPIO_MODE_50MHz|GPIO_PORTC|GPIO_PIN10)
#  define GPIO_SPI3_MISO    (GPIO_INPUT|GPIO_CNF_INFLOAT|GPIO_MODE_INPUT|GPIO_PORTC|GPIO_PIN11)
#  define GPIO_SPI3_MOSI    (GPIO_ALT|GPIO_CNF_AFPP|GPIO_MODE_50MHz|GPIO_PORTC|GPIO_PIN12)
#else
#  define GPIO_SPI3_NSS     (GPIO_INPUT|GPIO_CNF_INFLOAT|GPIO_MODE_INPUT|GPIO_PORTA|GPIO_PIN15)
#  define GPIO_SPI3_SCK     (GPIO_ALT|GPIO_CNF_AFPP|GPIO_MODE_50MHz|GPIO_PORTB|GPIO_PIN3)
#  define GPIO_SPI3_MISO    (GPIO_INPUT|GPIO_CNF_INFLOAT|GPIO_MODE_INPUT|GPIO_PORTB|GPIO_PIN4)
#  define GPIO_SPI3_MOSI    (GPIO_ALT|GPIO_CNF_AFPP|GPIO_MODE_50MHz|GPIO_PORTB|GPIO_PIN5)
#endif

/* I2C */

#if defined(CONFIG_STM32_I2C1_REMAP)
#  define GPIO_I2C1_SCL     (GPIO_ALT|GPIO_CNF_AFOD|GPIO_MODE_50MHz|GPIO_PORTB|GPIO_PIN8)
#  define GPIO_I2C1_SDA     (GPIO_ALT|GPIO_CNF_AFOD|GPIO_MODE_50MHz|GPIO_PORTB|GPIO_PIN9)
#else
#  define GPIO_I2C1_SCL     (GPIO_ALT|GPIO_CNF_AFOD|GPIO_MODE_50MHz|GPIO_PORTB|GPIO_PIN6)
#  define GPIO_I2C1_SDA     (GPIO_ALT|GPIO_CNF_AFOD|GPIO_MODE_50MHz|GPIO_PORTB|GPIO_PIN7)
#endif
#define GPIO_I2C1_SMBA    (GPIO_ALT|GPIO_CNF_INFLOAT|GPIO_MODE_50MHz|GPIO_PORTB|GPIO_PIN5)

#define GPIO_I2C2_SCL     (GPIO_ALT|GPIO_CNF_AFOD|GPIO_MODE_50MHz|GPIO_PORTB|GPIO_PIN10)
#define GPIO_I2C2_SDA     (GPIO_ALT|GPIO_CNF_AFOD|GPIO_MODE_50MHz|GPIO_PORTB|GPIO_PIN11)
#define GPIO_I2C2_SMBA    (GPIO_ALT|GPIO_CNF_INFLOAT|GPIO_MODE_50MHz|GPIO_PORTB|GPIO_PIN12)

/* CAN */

#if defined(CONFIG_STM32_CAN1_REMAP1)
#  define GPIO_CAN1_RX      (GPIO_INPUT|GPIO_CNF_INFLOAT|GPIO_MODE_INPUT|GPIO_PORTB|GPIO_PIN8)
#  define GPIO_CAN1_TX      (GPIO_ALT|GPIO_CNF_AFPP|GPIO_MODE_50MHz|GPIO_PORTB|GPIO_PIN9)
#elif defined(CONFIG_STM32_CAN1_REMAP2)
#  define GPIO_CAN1_RX      (GPIO_INPUT|GPIO_CNF_INFLOAT|GPIO_MODE_INPUT|GPIO_PORTD|GPIO_PIN0)
#  define GPIO_CAN1_TX      (GPIO_ALT|GPIO_CNF_AFPP|GPIO_MODE_50MHz|GPIO_PORTD|GPIO_PIN1)
#else
#  define GPIO_CAN1_RX      (GPIO_INPUT|GPIO_CNF_INFLOAT|GPIO_MODE_INPUT|GPIO_PORTA|GPIO_PIN11)
#  define GPIO_CAN1_TX      (GPIO_ALT|GPIO_CNF_AFPP|GPIO_MODE_50MHz|GPIO_PORTA|GPIO_PIN12)
#endif

/* SDIO */

#define GPIO_SDIO_D0      (GPIO_ALT|GPIO_CNF_AFPP|GPIO_MODE_50MHz|GPIO_PORTC|GPIO_PIN8)

#ifndef CONFIG_STM32_SDIO_WIDTH_D1_ONLY
#  define GPIO_SDIO_D1    (GPIO_ALT|GPIO_CNF_AFPP|GPIO_MODE_50MHz|GPIO_PORTC|GPIO_PIN9)
#  define GPIO_SDIO_D2    (GPIO_ALT|GPIO_CNF_AFPP|GPIO_MODE_50MHz|GPIO_PORTC|GPIO_PIN10)
#  define GPIO_SDIO_D3    (GPIO_ALT|GPIO_CNF_AFPP|GPIO_MODE_50MHz|GPIO_PORTC|GPIO_PIN11)
#  define GPIO_SDIO_D4    (GPIO_ALT|GPIO_CNF_AFPP|GPIO_MODE_50MHz|GPIO_PORTB|GPIO_PIN8)
#  define GPIO_SDIO_D5    (GPIO_ALT|GPIO_CNF_AFPP|GPIO_MODE_50MHz|GPIO_PORTB|GPIO_PIN9)
#  define GPIO_SDIO_D6    (GPIO_ALT|GPIO_CNF_AFPP|GPIO_MODE_50MHz|GPIO_PORTC|GPIO_PIN6)
#  define GPIO_SDIO_D7    (GPIO_ALT|GPIO_CNF_AFPP|GPIO_MODE_50MHz|GPIO_PORTC|GPIO_PIN7)
#endif

#define GPIO_SDIO_CK      (GPIO_ALT|GPIO_CNF_AFPP|GPIO_MODE_50MHz|GPIO_PORTC|GPIO_PIN12)
#define GPIO_SDIO_CMD     (GPIO_ALT|GPIO_CNF_AFPP|GPIO_MODE_50MHz|GPIO_PORTD|GPIO_PIN2)

#endif /* __ARCH_ARM_SRC_STM32_HARDWARE_STM32F103R_PINMAP_LEGACY_H */
