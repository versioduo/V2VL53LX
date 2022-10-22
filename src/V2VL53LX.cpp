// © Kay Sievers <kay@versioduo.com>, 2020-2022
// SPDX-License-Identifier: Apache-2.0

#include "V2VL53LX.h"
#include "vl53lx_api.h"

#ifndef V2VL53LX_MAX_DEVICES
#define V2VL53LX_MAX_DEVICES 4
#endif

// Assign every device an index.
static uint8_t n_devices = 0;

// Jumping through hoops here to work-around the over-simplistic Arduino interrupt API;
// it has no arguments and requires a separate ISR for every interrupt that should be
// handled individually.
static volatile bool devices_ready[V2VL53LX_MAX_DEVICES]{};

static void device0_isr() {
  devices_ready[0] = true;
}

static void device1_isr() {
  devices_ready[1] = true;
}

static void device2_isr() {
  devices_ready[2] = true;
}

static void device3_isr() {
  devices_ready[3] = true;
}

static void (*devices_isr[V2VL53LX_MAX_DEVICES])(){device0_isr, device1_isr, device2_isr, device3_isr};

void V2VL53LX::Driver::begin() {
  if (n_devices == V2VL53LX_MAX_DEVICES)
    return;

  _index = n_devices;

  // Custom address to assign to the next device. Calling setAddress() before begin()
  // assigns a specific address.
  if (_address == 0)
    _address = 0x2a + n_devices;

  n_devices++;

  pinMode(_pin_reset, OUTPUT);
  pinMode(_pin_interrupt, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(_pin_interrupt), devices_isr[_index], FALLING);
  disable();
}

void V2VL53LX::Driver::disable() {
  digitalWrite(_pin_reset, LOW);

  // The default device address after a reset.
  _device.address = 0x29;
}

bool V2VL53LX::Driver::enable() {
  digitalWrite(_pin_reset, HIGH);
  uint8_t b;

  VL53LX_Error status = VL53LX_WaitDeviceBooted(&_device);
  if (status != VL53LX_ERROR_NONE)
    return false;

  if (_address > 0) {
    status = VL53LX_SetDeviceAddress(&_device, _address * 2);
    if (status != VL53LX_ERROR_NONE)
      return false;

    _device.address = _address;
  }

  status = VL53LX_DataInit(&_device);
  if (status != VL53LX_ERROR_NONE)
    return false;

  if (_config->detect <= 1000)
    status = VL53LX_SetDistanceMode(&_device, VL53LX_DISTANCEMODE_SHORT);
  else if (_config->detect <= 1800)
    status = VL53LX_SetDistanceMode(&_device, VL53LX_DISTANCEMODE_MEDIUM);
  if (status != VL53LX_ERROR_NONE)
    return false;

  status = VL53LX_SetMeasurementTimingBudgetMicroSeconds(&_device, 100 * 1000);
  if (status != VL53LX_ERROR_NONE)
    return false;

  status = VL53LX_StartMeasurement(&_device);
  if (status != VL53LX_ERROR_NONE)
    return false;

  return true;
}

void V2VL53LX::Driver::loop() {
  if (!devices_ready[_index])
    return;

  VL53LX_MultiRangingData_t parameters;
  VL53LX_Error status = VL53LX_GetMultiRangingData(&_device, &parameters);

  VL53LX_ClearInterruptAndStartMeasurement(&_device);
  devices_ready[_index] = false;

  if (status != VL53LX_ERROR_NONE)
    return;

  // No object detected.
  if (parameters.RangeData[0].RangeStatus == VL53LX_RANGESTATUS_WRAP_TARGET_FAIL) {
    _measurement = 0;
    _distance    = -1;
    _fraction    = 0;
    _lag         = 0;

    if (_step == 0)
      return;

    _step = 0;
    handleUpdate(_step, _fraction, _distance, 0);
    return;
  }

  if (parameters.RangeData[0].RangeStatus != VL53LX_RANGESTATUS_RANGE_VALID)
    return;

  // Suppress outliers; this happens if the object is at the outer edge of the field of view.
  if (parameters.RangeData[0].RangeMinMilliMeter < 0)
    return;
  if (parameters.RangeData[0].RangeMinMilliMeter + 10 < _config->min)
    return;
  if (parameters.RangeData[0].RangeMaxMilliMeter < 0)
    return;
  if (parameters.RangeData[0].RangeMaxMilliMeter + 10 > _config->detect)
    return;

  // Discard the first measurements to suppress spurious and very short-living events.
  _measurement++;
  if (_measurement <= 3)
    return;

  // Low-pass filter, smooth the measured value.
  float distance = parameters.RangeData[0].RangeMilliMeter;
  _distance *= 1 - _config->alpha;
  _distance += distance * _config->alpha;

  // Too far away, ignore it.
  if (_distance > _config->detect) {
    _fraction = 0;
    _lag      = 0;

    if (_step == 0)
      return;

    _step = 0;
    handleUpdate(_step, _fraction, -1, 0);
    return;
  }

  // Below the minimum distance.
  if (_distance < _config->min) {
    _fraction = 0;
    _lag      = 0 - _config->lag;

    if (_step == 0)
      return;

    _step = 0;
    handleUpdate(_step, _fraction, -1, 0);
    return;
  }

  // Above the maximum distance, but in detection range.
  if (_distance > _config->max) {
    _fraction = 1;
    _lag      = 1 + _config->lag;

    if (_step == _config->n_steps - 1)
      return;

    _step = _config->n_steps - 1;
    handleUpdate(_step, _fraction, _distance, 1);
    return;
  }

  // In range; normalized 0..1 fraction of the min..max range.
  _fraction = (_distance - _config->min) / (_config->max - _config->min);

  // If the new measurement is inside the lag, keep the current step value.
  if (fabs(_fraction - _lag) < _config->lag)
    return;

  uint32_t step = roundf(_fraction * (_config->n_steps - 1));
  if (_step == step)
    return;

  // Reposition the edge of the lag. We follow monotonic changes immediately,
  // but apply the lag if the direction changes.
  if (_fraction - _lag > 0)
    _lag = _fraction - _config->lag;

  else
    _lag = _fraction + _config->lag;

  _step = step;
  handleUpdate(_step, _fraction, _distance, parameters.NumberOfObjectsFound);
}
