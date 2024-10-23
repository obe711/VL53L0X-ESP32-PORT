/**
 * @file vl53l0x-esp32-port.c
 * @author Obediah Klopfenstein (obe711@gmail.com)
 * @brief VL53L0X STM driver port API header
 * @version 1
 * @date 2024-10-23
 *
 * @copyright Copyright (c) 2024
 *
 */

#include "vl53l0x-esp32-port.h"

#define TAG "vl53l0x-esp32-port"

static VL53L0X_Dev_t VL53L0X_Dev;
static VL53L0X_Calibration_t VL53L0X_Calibration;

VL53L0X_Error print_pal_error(VL53L0X_Error status, const char *method) {
  char buf[VL53L0X_MAX_STRING_LENGTH];
  VL53L0X_GetPalErrorString(status, buf);
  ESP_LOGE(TAG, "%s API status: %i : %s\n", method, status, buf);
  return status;
}

VL53L0X_Error init_vl53l0x(VL53L0X_Dev_t *pDevice) {
  VL53L0X_Error status;
  uint8_t isApertureSpads;
  uint8_t PhaseCal;
  uint32_t refSpadCount;
  uint8_t VhvSettings;
  // Device Initialization (~40ms)
  status = VL53L0X_DataInit(pDevice);
  if (status != VL53L0X_ERROR_NONE)
    return print_pal_error(status, "VL53L0X_DataInit");
  status = VL53L0X_StaticInit(pDevice);
  if (status != VL53L0X_ERROR_NONE)
    return print_pal_error(status, "VL53L0X_StaticInit");
  // SPADs calibration (~10ms)
  status = VL53L0X_PerformRefSpadManagement(pDevice, &refSpadCount,
                                            &isApertureSpads);
  ESP_LOGD(TAG, "refSpadCount = %" PRIu32 ", isApertureSpads = %" PRIu8 "\n",
           refSpadCount, isApertureSpads);
  if (status != VL53L0X_ERROR_NONE)
    return print_pal_error(status, "VL53L0X_PerformRefSpadManagement");
  // Temperature calibration (~40ms)
  status = VL53L0X_PerformRefCalibration(pDevice, &VhvSettings, &PhaseCal);
  if (status != VL53L0X_ERROR_NONE)
    return print_pal_error(status, "VL53L0X_PerformRefCalibration");
  // Setup in single ranging mode
  status = VL53L0X_SetDeviceMode(pDevice, VL53L0X_DEVICEMODE_SINGLE_RANGING);
  if (status != VL53L0X_ERROR_NONE)
    return print_pal_error(status, "VL53L0X_SetDeviceMode");
  // end
  return status;
}

VL53L0X_DEV vl53l0x__CreateDevice(i2c_port_t i2c_port) {
  VL53L0X_Dev.i2c_address = VL53l0X_DEFAULT_I2C_ADDRESS;
  VL53L0X_Dev.i2c_port_num = i2c_port;
  return &VL53L0X_Dev;
};

VL53L0X_Error vl53l0x__WaitMeasurementDataReady(VL53L0X_DEV Dev) {
  VL53L0X_Error Status = VL53L0X_ERROR_NONE;
  uint8_t NewDatReady = 0;
  uint32_t LoopNb;

  // Wait until it finished
  // use timeout to avoid deadlock
  if (Status == VL53L0X_ERROR_NONE) {
    LoopNb = 0;
    do {
      Status = VL53L0X_GetMeasurementDataReady(Dev, &NewDatReady);
      if ((NewDatReady == 0x01) || Status != VL53L0X_ERROR_NONE) {
        break;
      }
      LoopNb = LoopNb + 1;
      VL53L0X_PollingDelay(Dev);
    } while (LoopNb < VL53L0X_DEFAULT_MAX_LOOP);

    if (LoopNb >= VL53L0X_DEFAULT_MAX_LOOP) {
      Status = VL53L0X_ERROR_TIME_OUT;
      return print_pal_error(Status, "vl53l0x__WaitMeasurementDataReady");
    }
  }

  return Status;
}

VL53L0X_Error vl53l0x__WaitStopCompleted(VL53L0X_DEV Dev) {

  VL53L0X_Error Status = VL53L0X_ERROR_NONE;
  uint32_t StopCompleted = 0;
  uint32_t LoopNb;

  // Wait until it finished
  // use timeout to avoid deadlock
  if (Status == VL53L0X_ERROR_NONE) {
    LoopNb = 0;
    do {
      Status = VL53L0X_GetStopCompletedStatus(Dev, &StopCompleted);
      if ((StopCompleted == 0x00) || Status != VL53L0X_ERROR_NONE) {
        break;
      }
      LoopNb = LoopNb + 1;
      VL53L0X_PollingDelay(Dev);
    } while (LoopNb < VL53L0X_DEFAULT_MAX_LOOP);

    if (LoopNb >= VL53L0X_DEFAULT_MAX_LOOP) {
      Status = VL53L0X_ERROR_TIME_OUT;
      return print_pal_error(Status, "vl53l0x__WaitStopCompleted");
    }
  }

  return Status;
}

VL53L0X_Error vl53l0x__Calibrate(VL53L0X_DEV Dev) {
  VL53L0X_Error Status = VL53L0X_ERROR_NONE;
  Status = VL53L0X_PerformRefCalibration(
      Dev, &VL53L0X_Calibration.VhvSettings,
      &VL53L0X_Calibration.PhaseCal); // Device Initialization

  if (Status != VL53L0X_ERROR_NONE) {
    return print_pal_error(Status, "VL53L0X_PerformRefCalibration");
  }

  Status = VL53L0X_PerformRefSpadManagement(
      Dev, &VL53L0X_Calibration.refSpadCount,
      &VL53L0X_Calibration.isApertureSpads); // Device Initialization
  if (Status != VL53L0X_ERROR_NONE) {
    return print_pal_error(Status, "VL53L0X_PerformRefSpadManagement");
  }

  return Status;
}

VL53L0X_Error vl53l0x__InitializeDriver(VL53L0X_DEV Dev,
                                        VL53L0X_VERSION Version,
                                        VL53L0X_DEVICE_INFO DeviceInfo) {

  VL53L0X_Error Status = init_vl53l0x(Dev);
  if (Status != VL53L0X_ERROR_NONE) {
    return print_pal_error(Status, "init_vl53l0x");
  }

  Status = VL53L0X_GetVersion(Version);
  if (Status != VL53L0X_ERROR_NONE) {
    return print_pal_error(Status, "VL53L0X_GetVersion");
  }

  Status = VL53L0X_GetDeviceInfo(Dev, DeviceInfo);
  if (Status != VL53L0X_ERROR_NONE) {
    return print_pal_error(Status, "VL53L0X_GetDeviceInfo");
  }

  Status = VL53L0X_StaticInit(Dev);
  if (Status != VL53L0X_ERROR_NONE) {
    return print_pal_error(Status, "VL53L0X_StaticInit");
  }

  Status = vl53l0x__Calibrate(Dev);
  if (Status != VL53L0X_ERROR_NONE) {
    return print_pal_error(Status, "vl53l0x__Calibrate");
  }

  Status = VL53L0X_SetDeviceMode(
      Dev,
      VL53L0X_DEVICEMODE_CONTINUOUS_RANGING); // Setup in single ranging mode
  if (Status != VL53L0X_ERROR_NONE) {
    return print_pal_error(Status, "VL53L0X_SetDeviceMode");
  }
  return Status;
};

VL53L0X_Error vl53l0x__Start(VL53L0X_DEV Dev) {
  VL53L0X_Error Status = VL53L0X_StartMeasurement(Dev);
  if (Status != VL53L0X_ERROR_NONE) {
    return print_pal_error(Status, "VL53L0X_StartMeasurement");
  }
  return Status;
}

VL53L0X_Error vl53l0x__GetDistance(
    VL53L0X_DEV Dev,
    VL53L0X_RangingMeasurementData_t *pRangingMeasurementData) {
  VL53L0X_Error Status = vl53l0x__WaitMeasurementDataReady(Dev);
  if (Status != VL53L0X_ERROR_NONE) {
    return print_pal_error(Status, "vl53l0x__WaitMeasurementDataReady");
  }

  return VL53L0X_GetRangingMeasurementData(Dev, pRangingMeasurementData);
};

VL53L0X_Error vl53l0x__Stop(VL53L0X_DEV Dev) {
  VL53L0X_Error Status = VL53L0X_StopMeasurement(Dev);
  if (Status != VL53L0X_ERROR_NONE) {
    return print_pal_error(Status, "VL53L0X_StopMeasurement");
  }

  Status = vl53l0x__WaitStopCompleted(Dev);
  if (Status != VL53L0X_ERROR_NONE) {
    return print_pal_error(Status, "vl53l0x__WaitStopCompleted");
  }

  Status = VL53L0X_ClearInterruptMask(
      Dev, VL53L0X_REG_SYSTEM_INTERRUPT_GPIO_NEW_SAMPLE_READY);
  if (Status != VL53L0X_ERROR_NONE) {
    return print_pal_error(Status, "VL53L0X_ClearInterruptMask");
  }

  return Status;
}

VL53L0X_Error vl53l0x__Clear(VL53L0X_DEV Dev) {
  VL53L0X_Error Status = VL53L0X_ClearInterruptMask(
      Dev, VL53L0X_REG_SYSTEM_INTERRUPT_GPIO_NEW_SAMPLE_READY);

  if (Status != VL53L0X_ERROR_NONE) {
    return print_pal_error(Status, "VL53L0X_ClearInterruptMask");
  }
  Status = VL53L0X_PollingDelay(Dev);
  if (Status != VL53L0X_ERROR_NONE) {
    return print_pal_error(Status, "VL53L0X_PollingDelay");
  }
  return Status;
};