// © Kay Sievers <kay@versioduo.com>, 2020-2022
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include "Arduino.h"
#include "Wire.h"
#include "vl53lx_platform.h"

class V2VL53LX {
public:
  struct Configuration {
    // The number of steps to map the measurement to.
    // 128 steps will emit values from 0 to 127.
    uint32_t n_steps;

    // The measurement range in millimeter.
    int16_t min;
    int16_t max;

    // The maximum distance in millimeter to recognize an object. It will be reported as 'max'.
    int16_t detect;

    // The exponential smoothing constant.
    float alpha;

    // Hysteresis lag; the amount of jitter we accept without changing the step.
    // The unit is a fraction of the normalized 0..1 value of the min..max range.
    float lag;
  };

  class Driver {
  public:
    constexpr Driver(const struct Configuration *config, TwoWire *i2c, uint8_t pin_reset, uint8_t pin_interrupt) :
      _config(config),
      _pin_reset(pin_reset),
      _pin_interrupt(pin_interrupt),
      _device({.userdata = i2c}) {}

    //  Specific address; if not called, a new one will be assigned automatically.
    void setAddress(uint8_t address) {
      _address = address;
    }

    // Initialize the Driver; it is still disabled, not visible on the bus.
    void begin();

    // Disable the Driver.
    void disable();

    // The Driver will appear on the bus with its default address, and get its
    // unique address assigned.
    bool enable();

    // If the Driver signifies by interrupt that a measurement is ready,
    // retrieve it; otherwise do nothing.
    void loop();

  protected:
    // Called when a value changes from the previous mesurement.
    virtual void handleUpdate(uint32_t step, float fraction, float distance, uint8_t count);

  private:
    const struct Configuration *_config = NULL;
    uint8_t _index                      = 0;
    uint8_t _address                    = 0;
    uint8_t _pin_reset                  = 0;
    uint8_t _pin_interrupt              = 0;
    VL53LX_Dev_t _device{};

    // The number of successful measurements after no object was detected;
    // used to discard the very first measurements to avoid spurious events.
    uint32_t _measurement = 0;

    // Distance in millimeter; constantly updated, triggered by Driver interrupt, smoothed-out.
    float _distance = -1;

    // Normalized 0..1 value of the min..max range.
    float _fraction = 0;

    // Current step value.
    uint32_t _step = 0;

    // The edge of the lag range, set by the previous value change.
    float _lag = 0;
  };
};
