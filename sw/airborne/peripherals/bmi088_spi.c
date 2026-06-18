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
 * @file peripherals/bmi088_spi.c
 *
 * Driver for the BMI088 using SPI.
 *
 * !!! UNTESTED SCAFFOLD - validate on hardware before flight use. !!!
 * Mirrors peripherals/bmi088_i2c.c, adapted for the SPI specifics described
 * in bmi088_spi.h (read flag, accel dummy byte, two chip selects, accel SPI
 * mode-switch). The shared configuration sequence in peripherals/bmi088.c is
 * reused unchanged.
 */

#include "peripherals/bmi088_spi.h"

/* little-endian: BMI088 outputs LSB first */
#define Int16FromBuf(_buf,_idx) ((int16_t)((_buf[_idx+1]<<8) | _buf[_idx]))

static void bmi088_spi_trans_init(struct spi_transaction *t, uint8_t slave_idx,
                                  volatile uint8_t *txb, volatile uint8_t *rxb)
{
  /* BMI088 SPI: mode 3 (CPOL=1, CPHA=1), up to 10 MHz, MSB first */
  t->cpol = SPICpolIdleHigh;
  t->cpha = SPICphaEdge2;
  t->dss = SPIDss8bit;
  t->bitorder = SPIMSBFirst;
  t->cdiv = SPIDiv32;
  t->select = SPISelectUnselect;
  t->slave_idx = slave_idx;
  t->output_length = 0;
  t->input_length = 0;
  t->before_cb = NULL;
  t->after_cb = NULL;
  t->output_buf = (uint8_t *)txb;
  t->input_buf = (uint8_t *)rxb;
  t->status = SPITransDone;
}

void bmi088_spi_init(struct Bmi088_Spi *bmi, struct spi_periph *spi_p,
                     uint8_t gyro_slave_idx, uint8_t accel_slave_idx)
{
  bmi->spi_p = spi_p;

  bmi088_spi_trans_init(&bmi->gyro_trans, gyro_slave_idx, bmi->gyro_tx_buf, bmi->gyro_rx_buf);
  bmi088_spi_trans_init(&bmi->accel_trans, accel_slave_idx, bmi->accel_tx_buf, bmi->accel_rx_buf);

  /* set default BMI088 config options */
  bmi088_set_default_config(&(bmi->config));

  bmi->gyro_available = false;
  bmi->accel_available = false;
  bmi->accel_spi_ready = false;
  bmi->config.initialized = false;
  bmi->config.init_status = BMI088_CONF_UNINIT;
}

static void bmi088_spi_write_to_reg(void *bmi, uint8_t _reg, uint8_t _val, uint8_t _type)
{
  struct Bmi088_Spi *bmi_spi = (struct Bmi088_Spi *)(bmi);
  if (_type == BMI088_CONFIG_ACCEL) {
    bmi_spi->accel_trans.output_length = 2;
    bmi_spi->accel_trans.input_length = 0;
    bmi_spi->accel_tx_buf[0] = _reg & ~BMI088_SPI_READ_FLAG; // write: MSB cleared
    bmi_spi->accel_tx_buf[1] = _val;
    spi_submit(bmi_spi->spi_p, &(bmi_spi->accel_trans));
  } else if (_type == BMI088_CONFIG_GYRO) {
    bmi_spi->gyro_trans.output_length = 2;
    bmi_spi->gyro_trans.input_length = 0;
    bmi_spi->gyro_tx_buf[0] = _reg & ~BMI088_SPI_READ_FLAG;
    bmi_spi->gyro_tx_buf[1] = _val;
    spi_submit(bmi_spi->spi_p, &(bmi_spi->gyro_trans));
  }
}

/* issue a dummy accel read; its only purpose before config is to generate the
 * CSB rising edge that switches the accelerometer from I2C to SPI mode */
static void bmi088_spi_accel_dummy_read(struct Bmi088_Spi *bmi)
{
  bmi->accel_trans.output_length = 1;
  bmi->accel_trans.input_length = 3; // addr phase + dummy + 1 byte
  bmi->accel_tx_buf[0] = BMI088_ACCEL_CHIP_ID | BMI088_SPI_READ_FLAG;
  spi_submit(bmi->spi_p, &(bmi->accel_trans));
}

// Configuration function called once before normal use
void bmi088_spi_start_configure(struct Bmi088_Spi *bmi)
{
  /* first, make sure the accelerometer is switched to SPI mode */
  if (!bmi->accel_spi_ready) {
    if (bmi->accel_trans.status == SPITransSuccess || bmi->accel_trans.status == SPITransDone) {
      bmi088_spi_accel_dummy_read(bmi);
    }
    return;
  }

  if (bmi->config.init_status == BMI088_CONF_UNINIT) {
    bmi->config.init_status++;
    if (bmi->accel_trans.status == SPITransSuccess || bmi->accel_trans.status == SPITransDone) {
      bmi088_send_config(bmi088_spi_write_to_reg, (void *)bmi, &(bmi->config));
    }
  }
}

void bmi088_spi_read(struct Bmi088_Spi *bmi)
{
  if (bmi->config.initialized &&
      bmi->gyro_trans.status == SPITransDone &&
      bmi->accel_trans.status == SPITransDone) {
    /* read gyro: addr phase + 6 data bytes (no dummy on the gyro) */
    bmi->gyro_trans.output_length = 1;
    bmi->gyro_trans.input_length = 1 + 6;
    bmi->gyro_tx_buf[0] = BMI088_GYRO_RATE_X_LSB | BMI088_SPI_READ_FLAG;
    spi_submit(bmi->spi_p, &(bmi->gyro_trans));
    /* read accel: addr phase + 1 dummy + 6 data bytes */
    bmi->accel_trans.output_length = 1;
    bmi->accel_trans.input_length = 1 + 1 + 6;
    bmi->accel_tx_buf[0] = BMI088_ACCEL_X_LSB | BMI088_SPI_READ_FLAG;
    spi_submit(bmi->spi_p, &(bmi->accel_trans));
  }
}

void bmi088_spi_event(struct Bmi088_Spi *bmi)
{
  /* phase 0: wait for the accel SPI-mode switch dummy read to complete */
  if (!bmi->accel_spi_ready) {
    if (bmi->accel_trans.status == SPITransSuccess) {
      bmi->accel_spi_ready = true;
      bmi->accel_trans.status = SPITransDone;
    } else if (bmi->accel_trans.status == SPITransFailed) {
      bmi->accel_trans.status = SPITransDone; // retry on next periodic
    }
    return;
  }

  if (bmi->config.initialized) {
    // gyro: data starts at rx[1] (rx[0] clocked while the address byte was sent)
    if (bmi->gyro_trans.status == SPITransFailed) {
      bmi->gyro_trans.status = SPITransDone;
    } else if (bmi->gyro_trans.status == SPITransSuccess) {
      bmi->data_rates.rates.p = Int16FromBuf(bmi->gyro_rx_buf, 1);
      bmi->data_rates.rates.q = Int16FromBuf(bmi->gyro_rx_buf, 3);
      bmi->data_rates.rates.r = Int16FromBuf(bmi->gyro_rx_buf, 5);
      bmi->gyro_available = true;
      bmi->gyro_trans.status = SPITransDone;
    }
    // accel: data starts at rx[2] (rx[0] address phase, rx[1] dummy byte)
    if (bmi->accel_trans.status == SPITransFailed) {
      bmi->accel_trans.status = SPITransDone;
    } else if (bmi->accel_trans.status == SPITransSuccess) {
      bmi->data_accel.vect.x = Int16FromBuf(bmi->accel_rx_buf, 2);
      bmi->data_accel.vect.y = Int16FromBuf(bmi->accel_rx_buf, 4);
      bmi->data_accel.vect.z = Int16FromBuf(bmi->accel_rx_buf, 6);
      bmi->accel_available = true;
      bmi->accel_trans.status = SPITransDone;
    }
  } else if (bmi->config.init_status != BMI088_CONF_UNINIT) { // configuring
    if (bmi->config.init_status <= BMI088_CONF_ACCEL_PWR_CTRL) {
      // accel config phase
      switch (bmi->accel_trans.status) {
        case SPITransFailed:
          bmi->config.init_status--; // retry (TODO max retry)
          /* Falls through. */
        case SPITransSuccess:
        case SPITransDone:
          bmi088_send_config(bmi088_spi_write_to_reg, (void *)bmi, &(bmi->config));
          break;
        default:
          break;
      }
    } else {
      // gyro config phase
      switch (bmi->gyro_trans.status) {
        case SPITransFailed:
          bmi->config.init_status--; // retry (TODO max retry)
          /* Falls through. */
        case SPITransSuccess:
        case SPITransDone:
          bmi088_send_config(bmi088_spi_write_to_reg, (void *)bmi, &(bmi->config));
          if (bmi->config.initialized) {
            bmi->gyro_trans.status = SPITransDone;
          }
          break;
        default:
          break;
      }
    }
  }
}
