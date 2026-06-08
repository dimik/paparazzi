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
 * @file modules/imu/imu_bmi088_spi.c
 *
 * IMU driver for the BMI088 using SPI.
 *
 * !!! UNTESTED SCAFFOLD - validate on hardware before flight use. !!!
 * Mirrors modules/imu/imu_bmi088_i2c.c; only the transport differs.
 */

#include "modules/imu/imu_bmi088_spi.h"
#include "modules/imu/imu.h"
#include "mcu_periph/spi.h"
#include "mcu_periph/sys_time.h"
#include "modules/core/abi.h"

PRINT_CONFIG_VAR(IMU_BMI088_GYRO_RANGE)
PRINT_CONFIG_VAR(IMU_BMI088_ACCEL_RANGE)

#ifndef IMU_BMI088_GYRO_ODR
#define IMU_BMI088_GYRO_ODR BMI088_GYRO_ODR_1000_BW_116
#endif
PRINT_CONFIG_VAR(IMU_BMI088_GYRO_ODR)

#ifndef IMU_BMI088_ACCEL_ODR
#define IMU_BMI088_ACCEL_ODR BMI088_ACCEL_ODR_1600
#endif
PRINT_CONFIG_VAR(IMU_BMI088_ACCEL_ODR)

#ifndef IMU_BMI088_ACCEL_BW
#define IMU_BMI088_ACCEL_BW BMI088_ACCEL_BW_OSR4
#endif
PRINT_CONFIG_VAR(IMU_BMI088_ACCEL_BW)

/* SPI slave indices for the two chip selects */
#ifndef IMU_BMI088_GYRO_SPI_SLAVE_IDX
#error "IMU_BMI088_GYRO_SPI_SLAVE_IDX must be defined (gyro CS)"
#endif
PRINT_CONFIG_VAR(IMU_BMI088_GYRO_SPI_SLAVE_IDX)

#ifndef IMU_BMI088_ACCEL_SPI_SLAVE_IDX
#error "IMU_BMI088_ACCEL_SPI_SLAVE_IDX must be defined (accel CS)"
#endif
PRINT_CONFIG_VAR(IMU_BMI088_ACCEL_SPI_SLAVE_IDX)

// Default channels order
#ifndef IMU_BMI088_CHAN_X
#define IMU_BMI088_CHAN_X 0
#endif
PRINT_CONFIG_VAR(IMU_BMI088_CHAN_X)
#ifndef IMU_BMI088_CHAN_Y
#define IMU_BMI088_CHAN_Y 1
#endif
PRINT_CONFIG_VAR(IMU_BMI088_CHAN_Y)
#ifndef IMU_BMI088_CHAN_Z
#define IMU_BMI088_CHAN_Z 2
#endif
PRINT_CONFIG_VAR(IMU_BMI088_CHAN_Z)

#ifndef IMU_BMI088_X_SIGN
#define IMU_BMI088_X_SIGN 1
#endif
PRINT_CONFIG_VAR(IMU_BMI088_X_SIGN)
#ifndef IMU_BMI088_Y_SIGN
#define IMU_BMI088_Y_SIGN 1
#endif
PRINT_CONFIG_VAR(IMU_BMI088_Y_SIGN)
#ifndef IMU_BMI088_Z_SIGN
#define IMU_BMI088_Z_SIGN 1
#endif
PRINT_CONFIG_VAR(IMU_BMI088_Z_SIGN)


struct ImuBmi088Spi imu_bmi088;

void imu_bmi088_init(void)
{
  /* BMI088 over SPI: gyro and accel are separate chip selects on the same bus */
  bmi088_spi_init(&imu_bmi088.bmi, &(IMU_BMI088_SPI_DEV),
                  IMU_BMI088_GYRO_SPI_SLAVE_IDX, IMU_BMI088_ACCEL_SPI_SLAVE_IDX);
  // change the default configuration
  imu_bmi088.bmi.config.gyro_range = IMU_BMI088_GYRO_RANGE;
  imu_bmi088.bmi.config.gyro_odr = IMU_BMI088_GYRO_ODR;
  imu_bmi088.bmi.config.accel_range = IMU_BMI088_ACCEL_RANGE;
  imu_bmi088.bmi.config.accel_odr = IMU_BMI088_ACCEL_ODR;
  imu_bmi088.bmi.config.accel_bw = IMU_BMI088_ACCEL_BW;

  // Set the default scaling
  imu_set_defaults_gyro(IMU_BMI088_ID, NULL, NULL, &BMI088_GYRO_SENS_F[IMU_BMI088_GYRO_RANGE]);
  imu_set_defaults_accel(IMU_BMI088_ID, NULL, NULL, &BMI088_ACCEL_SENS_F[IMU_BMI088_ACCEL_RANGE]);
}

void imu_bmi088_periodic(void)
{
  bmi088_spi_periodic(&imu_bmi088.bmi);
}

void imu_bmi088_event(void)
{
  uint32_t now_ts = get_sys_time_usec();

  bmi088_spi_event(&imu_bmi088.bmi);

  if (imu_bmi088.bmi.gyro_available) {
    struct Int32Rates rates = {
      IMU_BMI088_X_SIGN *(int32_t)(imu_bmi088.bmi.data_rates.value[IMU_BMI088_CHAN_X]),
      IMU_BMI088_Y_SIGN *(int32_t)(imu_bmi088.bmi.data_rates.value[IMU_BMI088_CHAN_Y]),
      IMU_BMI088_Z_SIGN *(int32_t)(imu_bmi088.bmi.data_rates.value[IMU_BMI088_CHAN_Z])
    };

    imu_bmi088.bmi.gyro_available = false;
    AbiSendMsgIMU_GYRO_RAW(IMU_BMI088_ID, now_ts, &rates, 1, imu_bmi088.bmi.config.gyro_samplerate, NAN);
  }
  if (imu_bmi088.bmi.accel_available) {
    struct Int32Vect3 accel = {
      IMU_BMI088_X_SIGN *(int32_t)(imu_bmi088.bmi.data_accel.value[IMU_BMI088_CHAN_X]),
      IMU_BMI088_Y_SIGN *(int32_t)(imu_bmi088.bmi.data_accel.value[IMU_BMI088_CHAN_Y]),
      IMU_BMI088_Z_SIGN *(int32_t)(imu_bmi088.bmi.data_accel.value[IMU_BMI088_CHAN_Z])
    };

    imu_bmi088.bmi.accel_available = false;
    AbiSendMsgIMU_ACCEL_RAW(IMU_BMI088_ID, now_ts, &accel, 1, imu_bmi088.bmi.config.accel_samplerate, NAN);
  }
}
