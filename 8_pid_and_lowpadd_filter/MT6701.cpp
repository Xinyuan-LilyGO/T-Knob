#include "MT6701.h"
#include "esp32-hal-gpio.h"
#include "Arduino.h"

static const float ALPHA = 0.4;

static uint8_t tableCRC6[64] = {
  0x00, 0x03, 0x06, 0x05, 0x0C, 0x0F, 0x0A, 0x09,
  0x18, 0x1B, 0x1E, 0x1D, 0x14, 0x17, 0x12, 0x11,
  0x30, 0x33, 0x36, 0x35, 0x3C, 0x3F, 0x3A, 0x39,
  0x28, 0x2B, 0x2E, 0x2D, 0x24, 0x27, 0x22, 0x21,
  0x23, 0x20, 0x25, 0x26, 0x2F, 0x2C, 0x29, 0x2A,
  0x3B, 0x38, 0x3D, 0x3E, 0x37, 0x34, 0x31, 0x32,
  0x13, 0x10, 0x15, 0x16, 0x1F, 0x1C, 0x19, 0x1A,
  0x0B, 0x08, 0x0D, 0x0E, 0x07, 0x04, 0x01, 0x02
};

/*32-bit input data, right alignment, Calculation over 18 bits (mult. of 6) */
static uint8_t CRC6_43_18bit(uint32_t w_InputData) {
  uint8_t b_Index = 0;
  uint8_t b_CRC = 0;

  b_Index = (uint8_t)(((uint32_t)w_InputData >> 12u) & 0x0000003Fu);

  b_CRC = (uint8_t)(((uint32_t)w_InputData >> 6u) & 0x0000003Fu);
  b_Index = b_CRC ^ tableCRC6[b_Index];

  b_CRC = (uint8_t)((uint32_t)w_InputData & 0x0000003Fu);
  b_Index = b_CRC ^ tableCRC6[b_Index];

  b_CRC = tableCRC6[b_Index];

  return b_CRC;
}

MT6701Sensor::MT6701Sensor() {}

void MT6701Sensor::init() {
  pinMode(MAG_CS, OUTPUT);
  digitalWrite(MAG_CS, HIGH);

  spi_bus_config_t tx_bus_config = {
    .mosi_io_num = -1,
    .miso_io_num = MAG_DIO,
    .sclk_io_num = MAG_CLK,
    .quadwp_io_num = -1,
    .quadhd_io_num = -1,
    .max_transfer_sz = 1000,
  };
  esp_err_t ret = spi_bus_initialize(SPI2_HOST, &tx_bus_config, SPI_DMA_CH_AUTO);
  ESP_ERROR_CHECK(ret);

  spi_device_interface_config_t tx_device_config = {
    .command_bits = 0,
    .address_bits = 0,
    .dummy_bits = 0,
    .mode = 1,
    .duty_cycle_pos = 0,
    .cs_ena_pretrans = 4,
    .cs_ena_posttrans = 0,
    .clock_speed_hz = 4000000,
    .input_delay_ns = 0,
    .spics_io_num = MAG_CS,
    .flags = 0,
    .queue_size = 1,
    .pre_cb = NULL,
    .post_cb = NULL,
  };
  ret = spi_bus_add_device(SPI2_HOST, &tx_device_config, &spi_device_);
  ESP_ERROR_CHECK(ret);

  spi_transaction_.flags = SPI_TRANS_USE_RXDATA;
  spi_transaction_.length = 24;
  spi_transaction_.rxlength = 24;
  spi_transaction_.tx_buffer = NULL;
  spi_transaction_.rx_buffer = NULL;

  // initialize all the internal variables of Sensor to ensure a "smooth" startup (without a 'jump' from zero)
  getSensorAngle();
  delayMicroseconds(1);
  vel_angle_prev = getSensorAngle();  // call again
  vel_angle_prev_ts = micros();
  delay(1);
  getSensorAngle();  // call once
  delayMicroseconds(1);
  angle_prev = getSensorAngle();  // call again
  angle_prev_ts = micros();
}

float MT6701Sensor::getSensorAngle() {
  uint32_t now = micros();
  if (now - last_update_ > 100) {

    esp_err_t ret = spi_device_polling_transmit(spi_device_, &spi_transaction_);
    assert(ret == ESP_OK);

    uint32_t spi_32 = (spi_transaction_.rx_data[0] << 16) | (spi_transaction_.rx_data[1] << 8) | spi_transaction_.rx_data[2];
    uint32_t angle_spi = spi_32 >> 10;

    // uint8_t field_status = (spi_32 >> 6) & 0x3; // 磁场强弱。0：正常；1：磁场过强；2：磁场太弱
    // uint8_t push_status = (spi_32 >> 8) & 0x1;  // 旋钮按压。0：正常；1：旋钮被按压
    // uint8_t loss_status = (spi_32 >> 9) & 0x1;  // 磁场变化速度。0：正常；1：超速

    uint8_t received_crc = spi_32 & 0x3F;
    uint8_t calculated_crc = CRC6_43_18bit(spi_32 >> 6);

    if (received_crc == calculated_crc) {
      float new_angle = (float)angle_spi * 2 * PI / 16384;
      float new_x = cosf(new_angle);
      float new_y = sinf(new_angle);
      x_ = new_x * ALPHA + x_ * (1 - ALPHA);
      y_ = new_y * ALPHA + y_ * (1 - ALPHA);
    } else {
      error_ = {
        .error = true,
        .received_crc = received_crc,
        .calculated_crc = calculated_crc,
      };
    }

    last_update_ = now;
  }
  float rad = -atan2f(y_, x_);
  if (rad < 0) {
    rad += 2 * PI;
  }
  return rad;
}

void MT6701Sensor::update() {
  float val = getSensorAngle();
  angle_prev_ts = micros();
  float d_angle = val - angle_prev;
  if (abs(d_angle) > (0.8f * 6.28318530718f)) full_rotations += (d_angle > 0) ? -1 : 1;
  angle_prev = val;
}

float MT6701Sensor::getMechanicalAngle() {
  update();
  return angle_prev;
}

float MT6701Sensor::getAngle() {
  update();
  return (float)full_rotations * 6.28318530718f + angle_prev;
}

double MT6701Sensor::getPreciseAngle() {
  update();
  return (double)full_rotations * (double)6.28318530718f + (double)angle_prev;
}

int32_t MT6701Sensor::getFullRotations() {
  update();
  return full_rotations;
}

/** get current angular velocity (rad/s) */
float MT6701Sensor::getVelocity() {
  update();
  // calculate sample time
  float Ts = (angle_prev_ts - vel_angle_prev_ts) * 1e-6;
  // quick fix for strange cases (micros overflow)
  if (Ts <= 0) Ts = 1e-3f;
  // velocity calculation
  float vel = ((float)(full_rotations - vel_full_rotations) * 6.28318530718f + (angle_prev - vel_angle_prev)) / Ts;
  // save variables for future pass
  vel_angle_prev = angle_prev;
  vel_full_rotations = full_rotations;
  vel_angle_prev_ts = angle_prev_ts;
  return vel;
}

MT6701Error MT6701Sensor::getAndClearError() {
  MT6701Error out = error_;
  error_ = {};
  return out;
}