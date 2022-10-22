// © Kay Sievers <kay@versioduo.com>, 2020-2022
// SPDX-License-Identifier: Apache-2.0

#include "vl53lx_api.h"
#include "vl53lx_platform.h"
#include <Arduino.h>
#include <Wire.h>

static int32_t write_multi(VL53LX_Dev_t *dev, uint16_t index, uint8_t *pdata, uint32_t count) {
  TwoWire *i2c = (TwoWire *)dev->userdata;
  i2c->beginTransmission(dev->address);
  i2c->write(index >> 8);
  i2c->write(index & 0xff);

  while (count--) {
    i2c->write((uint8_t)pdata[0]);
    pdata++;
  }

  int32_t status = i2c->endTransmission();
  if (status != 0)
    return VL53LX_ERROR_CONTROL_INTERFACE;

  return VL53LX_ERROR_NONE;
}

static int32_t read_multi(VL53LX_Dev_t *dev, uint16_t index, uint8_t *pdata, uint32_t count) {
  TwoWire *i2c = (TwoWire *)dev->userdata;
  i2c->beginTransmission(dev->address);
  i2c->write(index >> 8);
  i2c->write(index & 0xff);

  int32_t status = i2c->endTransmission();
  if (status != 0)
    return VL53LX_ERROR_CONTROL_INTERFACE;

  i2c->requestFrom(dev->address, (uint8_t)count);

  while (count--) {
    pdata[0] = i2c->read();
    pdata++;
  }

  return VL53LX_ERROR_NONE;
}

static int32_t write_byte(VL53LX_Dev_t *dev, uint16_t index, uint8_t data) {
  return write_multi(dev, index, &data, 1);
}

static int32_t write_word(VL53LX_Dev_t *dev, uint16_t index, uint16_t data) {
  uint8_t buffer[2];
  buffer[1] = data & 0xFF;
  buffer[0] = data >> 8;
  return write_multi(dev, index, buffer, 2);
}

static int32_t write_dword(VL53LX_Dev_t *dev, uint16_t index, uint32_t data) {
  uint8_t buffer[4];
  buffer[3] = data & 0xFF;
  buffer[2] = data >> 8;
  buffer[1] = data >> 16;
  buffer[0] = data >> 24;
  return write_multi(dev, index, buffer, 4);
}

static int32_t read_byte(VL53LX_Dev_t *dev, uint16_t index, uint8_t *data) {
  return read_multi(dev, index, data, 1);
}

static int32_t read_word(VL53LX_Dev_t *dev, uint16_t index, uint16_t *data) {
  uint8_t buffer[2];
  int32_t r = read_multi(dev, index, buffer, 2);

  uint16_t v;
  v = buffer[0];
  v <<= 8;
  v |= buffer[1];
  *data = v;

  return r;
}

static int32_t read_dword(VL53LX_Dev_t *dev, uint16_t index, uint32_t *data) {
  uint8_t buffer[4];
  int32_t r = read_multi(dev, index, buffer, 4);

  uint32_t v;
  v = buffer[0];
  v <<= 8;
  v |= buffer[1];
  v <<= 8;
  v |= buffer[2];
  v <<= 8;
  v |= buffer[3];
  *data = v;

  return r;
}

VL53LX_Error VL53LX_WriteMulti(VL53LX_Dev_t *dev, uint16_t index, uint8_t *pdata, uint32_t count) {
  int32_t status = write_multi(dev, index, pdata, count);
  if (status != 0)
    return VL53LX_ERROR_CONTROL_INTERFACE;

  return VL53LX_ERROR_NONE;
}

VL53LX_Error VL53LX_ReadMulti(VL53LX_Dev_t *dev, uint16_t index, uint8_t *pdata, uint32_t count) {
  int32_t status = read_multi(dev, index, pdata, count);
  if (status != 0)
    return VL53LX_ERROR_CONTROL_INTERFACE;

  return VL53LX_ERROR_NONE;
}

VL53LX_Error VL53LX_WrByte(VL53LX_Dev_t *dev, uint16_t index, uint8_t data) {
  int32_t status = write_byte(dev, index, data);
  if (status != 0)
    return VL53LX_ERROR_CONTROL_INTERFACE;

  return VL53LX_ERROR_NONE;
}

VL53LX_Error VL53LX_WrWord(VL53LX_Dev_t *dev, uint16_t index, uint16_t data) {
  int32_t status = write_word(dev, index, data);
  if (status != 0)
    return VL53LX_ERROR_CONTROL_INTERFACE;

  return VL53LX_ERROR_NONE;
}

VL53LX_Error VL53LX_WrDWord(VL53LX_Dev_t *dev, uint16_t index, uint32_t data) {
  int32_t status = write_dword(dev, index, data);
  if (status != 0)
    return VL53LX_ERROR_CONTROL_INTERFACE;

  return VL53LX_ERROR_NONE;
}

VL53LX_Error VL53LX_RdByte(VL53LX_Dev_t *dev, uint16_t index, uint8_t *data) {
  int32_t status = read_byte(dev, index, data);
  if (status != 0)
    return VL53LX_ERROR_CONTROL_INTERFACE;

  return VL53LX_ERROR_NONE;
}

VL53LX_Error VL53LX_RdWord(VL53LX_Dev_t *dev, uint16_t index, uint16_t *data) {
  int32_t status = read_word(dev, index, data);
  if (status != 0)
    return VL53LX_ERROR_CONTROL_INTERFACE;

  return VL53LX_ERROR_NONE;
}

VL53LX_Error VL53LX_RdDWord(VL53LX_Dev_t *dev, uint16_t index, uint32_t *data) {
  int32_t status = read_dword(dev, index, data);
  if (status != 0)
    return VL53LX_ERROR_CONTROL_INTERFACE;

  return VL53LX_ERROR_NONE;
}

VL53LX_Error VL53LX_WaitUs(VL53LX_Dev_t *pdev, int32_t wait_us) {
  delayMicroseconds(wait_us);
  return VL53LX_ERROR_NONE;
}

VL53LX_Error VL53LX_WaitMs(VL53LX_Dev_t *pdev, int32_t wait_ms) {
  delay(wait_ms);
  return VL53LX_ERROR_NONE;
}

VL53LX_Error VL53LX_WaitValueMaskEx(VL53LX_Dev_t *pdev,
                                    uint32_t timeout_ms,
                                    uint16_t index,
                                    uint8_t value,
                                    uint8_t mask,
                                    uint32_t poll_delay_ms) {
  unsigned long msec = millis();

  for (;;) {
    uint8_t b;
    int32_t status = VL53LX_RdByte(pdev, index, &b);

    if ((b & mask) == value)
      return VL53LX_ERROR_NONE;

    if ((unsigned long)(millis() - msec) < timeout_ms)
      break;

    delay(poll_delay_ms);
  }

  return VL53LX_ERROR_CONTROL_INTERFACE;
}
