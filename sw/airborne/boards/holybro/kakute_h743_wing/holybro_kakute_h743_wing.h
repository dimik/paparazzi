/*
 * Copyright (C) 2026 Paparazzi team
 *
 * This file is part of Paparazzi.
 *
 * Paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * Paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Paparazzi; see the file COPYING.  If not, write to
 * the Free Software Foundation, 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 */

/**
 * @file boards/holybro/kakute_h743_wing/holybro_kakute_h743_wing.h
 * @brief Holybro Kakute H743-Wing board configuration.
 *
 * Everything (LEDs, ADCs, UARTs, I2C, SPI buses, SPI slave selects, the
 * first 8 PWM outputs, CAN pins) is derived automatically from the pin
 * names in board.h by the generic arch/chibios/common_board.h.
 *
 * This wrapper only adds what common_board.h does not cover: the 6 extra
 * PWM outputs M9-M14 (common_board.h stops at SERVO8). The Kakute H743-Wing
 * exposes 14 PWM outputs (M1-M14).
 *
 * Pin map (Kakute H743-Wing silkscreen -> STM32H743):
 *   SPI1  : SCK PA5  MISO PA6  MOSI PA7         -> IMU1 BMI088
 *             SLAVE0 PC8  = BMI088 accel CS
 *             SLAVE1 PC9  = BMI088 gyro  CS
 *   SPI3  : SCK PC10 MISO PC11 MOSI PC12        -> IMU2 ICM42688-P
 *             SLAVE2 PE12 = ICM42688 CS
 *   SPI2  : SCK PD3  MISO PC2  MOSI PC3         -> OSD AT7456E
 *             SLAVE3 PB12 = MAX7456/AT7456E CS
 *   CAN1  : RX PD0   TX PD1                     -> DroneCAN/UAVCAN (Radxa Q6A)
 *   SD    : SDMMC2 (PB14/PB15/PB3/PB4 data, PC1 CK, PD7 CMD) -> ChibiOS SDCD2
 *   UART1 : TX PB6  RX PA10
 *   UART2 : TX PD5  RX PD6   (GPS)
 *   UART3 : TX PD8  RX PD9
 *   UART5 : TX PB13 RX PD2  (TX AF14 / RX AF8 differ - single-AF UART
 *           driver cannot drive both pins; use UART5 TX-only or avoid)
 *   UART6 : TX PC6  RX PC7   (SBUS/RC)
 *   UART7 : TX PE8  RX PE7  (+RTS PE9 / CTS PE10)
 *   UART8 : TX PE1  RX PE0
 *   I2C4  : SCL PD12 SDA PD13  -> onboard baro BMP280/SPL06 (internal bus)
 *   I2C1  : SCL PB8  SDA PB7    I2C2 : SCL PB10 SDA PB11   (external connectors)
 *   ADC   : Vbat PC5  Curr PC4  Vbat2 PA3  Curr2 PA2  RSSI PC0  5V-sens PA4
 *   LED   : LED1 PC15 (blue/ACT)  LED2 PC14 (green/B-E)   (active low)
 *   GPIO  : CAM_SELECT PC13  VTX_9V_EN PE3 (9V VTX BEC, default ON)
 *           USER1 PD4  USER2 PE4   VDD_3V3_SENSORS_EN PB2 (sensor rail, ON)
 *   PWM   : M1 PA8  M2 PE11 M3 PE13 M4 PE14 M5 PD14 M6 PD15 M7 PA0
 *           M8 PA1  M9 PE5  M10 PE6 M11 PB5 M12 PB0 M13 PB1 M14 PA15
 */

#ifndef CONFIG_HOLYBRO_KAKUTE_H743_WING_H
#define CONFIG_HOLYBRO_KAKUTE_H743_WING_H

/* Generic ChibiOS board wiring (LEDs/ADC/UART/SPI/CAN/SERVO0-8). */
#include "arch/chibios/common_board.h"

/*
 * Extra PWM outputs M9-M14 (common_board.h only defines up to SERVO8).
 */
#if defined(LINE_SERVO9)
#ifndef USE_PWM9
#define USE_PWM9 1
#endif
#if USE_PWM9
#define PWM_SERVO_9 9
#define PWM_SERVO_9_GPIO    PAL_PORT(LINE_SERVO9)
#define PWM_SERVO_9_PIN     PAL_PAD(LINE_SERVO9)
#define PWM_SERVO_9_AF      AF_LINE_SERVO9
#define PWM_SERVO_9_DRIVER  CONCAT_BOARD_PARAM(PWMD, SERVO9_TIM)
#define PWM_SERVO_9_CHANNEL (SERVO9_TIM_CH-1)
#define PWM_SERVO_9_CONF    CONCAT_BOARD_PARAM(pwmcfg, SERVO9_TIM)
#endif
#endif

#if defined(LINE_SERVO10)
#ifndef USE_PWM10
#define USE_PWM10 1
#endif
#if USE_PWM10
#define PWM_SERVO_10 10
#define PWM_SERVO_10_GPIO    PAL_PORT(LINE_SERVO10)
#define PWM_SERVO_10_PIN     PAL_PAD(LINE_SERVO10)
#define PWM_SERVO_10_AF      AF_LINE_SERVO10
#define PWM_SERVO_10_DRIVER  CONCAT_BOARD_PARAM(PWMD, SERVO10_TIM)
#define PWM_SERVO_10_CHANNEL (SERVO10_TIM_CH-1)
#define PWM_SERVO_10_CONF    CONCAT_BOARD_PARAM(pwmcfg, SERVO10_TIM)
#endif
#endif

#if defined(LINE_SERVO11)
#ifndef USE_PWM11
#define USE_PWM11 1
#endif
#if USE_PWM11
#define PWM_SERVO_11 11
#define PWM_SERVO_11_GPIO    PAL_PORT(LINE_SERVO11)
#define PWM_SERVO_11_PIN     PAL_PAD(LINE_SERVO11)
#define PWM_SERVO_11_AF      AF_LINE_SERVO11
#define PWM_SERVO_11_DRIVER  CONCAT_BOARD_PARAM(PWMD, SERVO11_TIM)
#define PWM_SERVO_11_CHANNEL (SERVO11_TIM_CH-1)
#define PWM_SERVO_11_CONF    CONCAT_BOARD_PARAM(pwmcfg, SERVO11_TIM)
#endif
#endif

#if defined(LINE_SERVO12)
#ifndef USE_PWM12
#define USE_PWM12 1
#endif
#if USE_PWM12
#define PWM_SERVO_12 12
#define PWM_SERVO_12_GPIO    PAL_PORT(LINE_SERVO12)
#define PWM_SERVO_12_PIN     PAL_PAD(LINE_SERVO12)
#define PWM_SERVO_12_AF      AF_LINE_SERVO12
#define PWM_SERVO_12_DRIVER  CONCAT_BOARD_PARAM(PWMD, SERVO12_TIM)
#define PWM_SERVO_12_CHANNEL (SERVO12_TIM_CH-1)
#define PWM_SERVO_12_CONF    CONCAT_BOARD_PARAM(pwmcfg, SERVO12_TIM)
#endif
#endif

#if defined(LINE_SERVO13)
#ifndef USE_PWM13
#define USE_PWM13 1
#endif
#if USE_PWM13
#define PWM_SERVO_13 13
#define PWM_SERVO_13_GPIO    PAL_PORT(LINE_SERVO13)
#define PWM_SERVO_13_PIN     PAL_PAD(LINE_SERVO13)
#define PWM_SERVO_13_AF      AF_LINE_SERVO13
#define PWM_SERVO_13_DRIVER  CONCAT_BOARD_PARAM(PWMD, SERVO13_TIM)
#define PWM_SERVO_13_CHANNEL (SERVO13_TIM_CH-1)
#define PWM_SERVO_13_CONF    CONCAT_BOARD_PARAM(pwmcfg, SERVO13_TIM)
#endif
#endif

#if defined(LINE_SERVO14)
#ifndef USE_PWM14
#define USE_PWM14 1
#endif
#if USE_PWM14
#define PWM_SERVO_14 14
#define PWM_SERVO_14_GPIO    PAL_PORT(LINE_SERVO14)
#define PWM_SERVO_14_PIN     PAL_PAD(LINE_SERVO14)
#define PWM_SERVO_14_AF      AF_LINE_SERVO14
#define PWM_SERVO_14_DRIVER  CONCAT_BOARD_PARAM(PWMD, SERVO14_TIM)
#define PWM_SERVO_14_CHANNEL (SERVO14_TIM_CH-1)
#define PWM_SERVO_14_CONF    CONCAT_BOARD_PARAM(pwmcfg, SERVO14_TIM)
#endif
#endif

#endif /* CONFIG_HOLYBRO_KAKUTE_H743_WING_H */
