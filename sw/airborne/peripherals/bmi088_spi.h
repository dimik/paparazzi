/*
 * Copyright (C) 2026 Paparazzi team
 *
 * This file is part of paparazzi.
 *
 * paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 */

/**
 * @file peripherals/bmi088_spi.h
 *
 * Driver for the BMI088 using SPI.
 *
 * The BMI088 exposes the accelerometer and gyroscope as two separate SPI
 * slaves (two chip-select lines) sharing one SPI bus. SPI specifics handled
 * here (vs the I2C transport):
 *   - read transfers set bit 7 of the register address; writes clear it;
 *   - the ACCELEROMETER returns one DUMMY byte before real data on every SPI
 *     read (the gyro does not), so accel reads carry one extra leading byte;
 *   - the accelerometer powers up in I2C mode and only switches to SPI after
 *     the first rising edge on its CSB - we issue one dummy accel read before
 *     configuration to trigger that switch.
 *
 * !!! UNTESTED: written from the datasheet, not yet validated on hardware. !!!
 */

#ifndef BMI088_SPI_H
#define BMI088_SPI_H

#include "std.h"
#include "math/pprz_algebra_int.h"
#include "mcu_periph/spi.h"

/* Include common BMI088 options and definitions */
#include "peripherals/bmi088.h"

/* SPI read/write address bit */
#define BMI088_SPI_READ_FLAG  0x80

/* enough for address + dummy + 6 data bytes, rounded up */
#define BMI088_SPI_BUFFER_LEN 16

struct Bmi088_Spi {
  struct spi_periph *spi_p;
  struct spi_transaction gyro_trans;   ///< SPI transaction for the gyro slave
  struct spi_transaction accel_trans;  ///< SPI transaction for the accel slave
  volatile uint8_t gyro_tx_buf[2];
  volatile uint8_t gyro_rx_buf[BMI088_SPI_BUFFER_LEN];
  volatile uint8_t accel_tx_buf[2];
  volatile uint8_t accel_rx_buf[BMI088_SPI_BUFFER_LEN];
  volatile bool gyro_available;        ///< gyro data ready flag
  volatile bool accel_available;       ///< accel data ready flag
  bool accel_spi_ready;                ///< accel switched from I2C to SPI mode
  union {
    struct Int16Vect3 vect;            ///< accel data vector in accel coordinate system
    int16_t value[3];                  ///< accel data values accessible by channel index
  } data_accel;
  union {
    struct Int16Rates rates;           ///< rates data as angular rates in gyro coordinate system
    int16_t value[3];                  ///< rates data values accessible by channel index
  } data_rates;
  struct Bmi088Config config;
};

// Functions
extern void bmi088_spi_init(struct Bmi088_Spi *bmi, struct spi_periph *spi_p,
                            uint8_t gyro_slave_idx, uint8_t accel_slave_idx);
extern void bmi088_spi_start_configure(struct Bmi088_Spi *bmi);
extern void bmi088_spi_read(struct Bmi088_Spi *bmi);
extern void bmi088_spi_event(struct Bmi088_Spi *bmi);

/// convenience function: read or start configuration if not already initialized
static inline void bmi088_spi_periodic(struct Bmi088_Spi *bmi)
{
  if (bmi->config.initialized) {
    bmi088_spi_read(bmi);
  } else {
    bmi088_spi_start_configure(bmi);
  }
}

#endif // BMI088_SPI_H
