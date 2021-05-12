#pragma once

#include "vl53lx_def.h"

typedef struct {
  VL53LX_DevData_t Data;
  uint8_t address;

  // Platform I2C device. Use 'void' to avoid leaking C++ headers into the C driver codebase.
  void *userdata;
} VL53LX_Dev_t;

typedef VL53LX_Dev_t *VL53LX_DEV;

#define VL53LXDevDataGet(Dev, field) (Dev->Data.field)

#define VL53LXDevDataSet(Dev, field, VL53LX_p_003) ((Dev->Data.field) = (VL53LX_p_003))

#define VL53LXDevStructGetLLDriverHandle(Dev) (&Dev->Data.LLData)

#define VL53LXDevStructGetLLResultsHandle(Dev) (&Dev->Data.llresults)
