/**
 * @file vl53l0x-esp32-port.h
 * @author Obediah Klopfenstein (obe711@gmail.com)
 * @brief VL53L0X STM driver port API header
 * @version 1
 * @date 2024-10-23
 *
 * @copyright Copyright (c) 2024
 *
 */

#pragma once

#include <stdint.h>
#include <string.h>

#include "vl53l0x_api.h"
#include "vl53l0x_platform.h"

#define VL53l0X_DEFAULT_I2C_ADDRESS 0x29

typedef VL53L0X_Version_t *VL53L0X_VERSION;
typedef VL53L0X_DeviceInfo_t *VL53L0X_DEVICE_INFO;

typedef struct {
  uint32_t refSpadCount;
  uint8_t isApertureSpads;
  uint8_t VhvSettings;
  uint8_t PhaseCal;
} VL53L0X_Calibration_t;

VL53L0X_DEV vl53l0x__CreateDevice(i2c_port_t i2c_port);

VL53L0X_Error vl53l0x__WaitMeasurementDataReady(VL53L0X_DEV Dev);

VL53L0X_Error vl53l0x__WaitStopCompleted(VL53L0X_DEV Dev);

VL53L0X_Error vl53l0x__InitializeDriver(VL53L0X_DEV Dev,
                                        VL53L0X_VERSION Version,
                                        VL53L0X_DEVICE_INFO DeviceInfo);

VL53L0X_Error vl53l0x__Start(VL53L0X_DEV Dev);
VL53L0X_Error vl53l0x__Stop(VL53L0X_DEV Dev);
VL53L0X_Error vl53l0x__Clear(VL53L0X_DEV Dev);

VL53L0X_Error
vl53l0x__GetDistance(VL53L0X_DEV Dev,
                     VL53L0X_RangingMeasurementData_t *pRangingMeasurementData);
