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

#define LOG_LEVEL_LOCAL ESP_LOG_DEBUG

static VL53L0X_Dev_t VL53L0X_Dev;
static VL53L0X_Calibration_t VL53L0X_Calibration;

VL53L0X_Error print_pal_error(VL53L0X_Error status, const char *method) {
  char buf[VL53L0X_MAX_STRING_LENGTH];
  VL53L0X_GetPalErrorString(status, buf);
  ESP_LOGE(TAG, "%s API status: %i : %s\n", method, status, buf);
  return status;
}

VL53L0X_Error vl53l0x__SetContinuousMode(VL53L0X_DEV Dev) {
  VL53L0X_Error Status = VL53L0X_SetDeviceMode(
      Dev,
      VL53L0X_DEVICEMODE_CONTINUOUS_RANGING); // Setup in single ranging mode
  if (Status != VL53L0X_ERROR_NONE) {
    return print_pal_error(Status, "VL53L0X_SetDeviceMode");
  }
  return Status;
}

VL53L0X_Error vl53l0x__SetSingleRangeMode(VL53L0X_DEV Dev) {
  VL53L0X_Error Status = VL53L0X_SetDeviceMode(
      Dev,
      VL53L0X_DEVICEMODE_SINGLE_RANGING); // Setup in single ranging mode
  if (Status != VL53L0X_ERROR_NONE) {
    return print_pal_error(Status, "VL53L0X_SetDeviceMode");
  }
  return Status;
}

VL53L0X_Error
vl53l0x__SetCalibrationData(VL53L0X_DEV Dev,
                            VL53L0X_Calibration_t *calibration_data) {
  VL53L0X_Error status;
  status = VL53L0X_SetReferenceSpads(Dev, calibration_data->refSpadCount,
                                     calibration_data->isApertureSpads);
  if (status != VL53L0X_ERROR_NONE)
    return print_pal_error(status, "VL53L0X_SetReferenceSpads");

  status = VL53L0X_SetRefCalibration(Dev, calibration_data->pVhvSettings,
                                     calibration_data->pPhaseCal);
  if (status != VL53L0X_ERROR_NONE)
    return print_pal_error(status, "VL53L0X_SetRefCalibration");

  status = VL53L0X_SetOffsetCalibrationDataMicroMeter(
      Dev, calibration_data->pOffsetMicroMeter);
  if (status != VL53L0X_ERROR_NONE)
    return print_pal_error(status,
                           "VL53L0X_SetOffsetCalibrationDataMicroMeter");

  status = VL53L0X_SetXTalkCompensationRateMegaCps(
      Dev, calibration_data->pXTalkCompensationRateMegaCps);
  if (status != VL53L0X_ERROR_NONE)
    return print_pal_error(status, "VL53L0X_SetXTalkCompensationRateMegaCps");

  status = VL53L0X_SetXTalkCompensationEnable(Dev, 1);
  if (status != VL53L0X_ERROR_NONE)
    return print_pal_error(status, "VL53L0X_SetXTalkCompensationEnable");

  return status;
};

VL53L0X_Error vl53l0x__OffsetCalibrate(VL53L0X_DEV Dev,
                                       VL53L0X_Calibration_t *calibration_data,
                                       uint32_t CalDistanceMilliMeter) {
  VL53L0X_Error status;
  // Offset calibration (~300ms) // After Set white target
  // Recommendation is to use a white (88%reflectance) target at 100mm, in a
  // dark environment.
  status = VL53L0X_PerformOffsetCalibration(
      Dev, (FixPoint1616_t)CalDistanceMilliMeter,
      &calibration_data->pOffsetMicroMeter);
  if (status != VL53L0X_ERROR_NONE)
    return print_pal_error(status, "VL53L0X_PerformOffsetCalibration");
  ESP_LOGI(TAG, "Offset calibration data: pOffsetMicroMeter = %" PRIi32 "\n",
           calibration_data->pOffsetMicroMeter);

  return status;
};

VL53L0X_Error vl53l0x__XTalkCalibrate(VL53L0X_DEV Dev,
                                      VL53L0X_Calibration_t *calibration_data,
                                      uint32_t XTalkCalDistance) {
  VL53L0X_Error status;
  // CrossTalk calibration (~1sec) // After Set grey target
  // Use a grey 17% reflectance target.  - Recomended
  status = VL53L0X_PerformXTalkCalibration(
      Dev, (FixPoint1616_t)XTalkCalDistance,
      &calibration_data->pXTalkCompensationRateMegaCps);
  if (status != VL53L0X_ERROR_NONE)
    return print_pal_error(status, "VL53L0X_PerformXTalkCalibration");
  ESP_LOGI(
      TAG,
      "CrossTalk calibration data: pXTalkCompensationRateMegaCps = %" PRIu32
      "\n",
      calibration_data->pXTalkCompensationRateMegaCps);

  return status;
};

VL53L0X_Error
vl53l0x__ManufacturingCalibrate(VL53L0X_DEV Dev,
                                VL53L0X_Calibration_t *calibration_data) {
  VL53L0X_Error status;

  // SPADs calibration (~10ms)
  status = VL53L0X_PerformRefSpadManagement(
      Dev, &calibration_data->refSpadCount, &calibration_data->isApertureSpads);
  if (status != VL53L0X_ERROR_NONE)
    return print_pal_error(status, "VL53L0X_PerformRefSpadManagement");
  ESP_LOGI(TAG,
           "SPADs calibration data: refSpadCount = %" PRIu32
           ", isApertureSpads = %" PRIu8 "\n",
           calibration_data->refSpadCount, calibration_data->isApertureSpads);

  // Temperature calibration (~40ms)
  status = VL53L0X_PerformRefCalibration(Dev, &calibration_data->pVhvSettings,
                                         &calibration_data->pPhaseCal);
  if (status != VL53L0X_ERROR_NONE)
    return print_pal_error(status, "VL53L0X_PerformRefSpadManagement");
  ESP_LOGI(TAG,
           "Temperature calibration data: pVhvSettings = %" PRIu8
           ", pPhaseCal = %" PRIu8 "\n",
           calibration_data->pVhvSettings, calibration_data->pPhaseCal);

  return status;
}

VL53L0X_Error init_vl53l0x(VL53L0X_Dev_t *pDevice) {
  VL53L0X_Error status;

  // Device Initialization (~40ms)
  status = VL53L0X_DataInit(pDevice);
  if (status != VL53L0X_ERROR_NONE)
    return print_pal_error(status, "VL53L0X_DataInit");

  // Static Initialization
  status = VL53L0X_StaticInit(pDevice);
  if (status != VL53L0X_ERROR_NONE)
    return print_pal_error(status, "VL53L0X_StaticInit");

  return status;
}

VL53L0X_DEV vl53l0x__CreateDevice(i2c_port_t i2c_port) {
  esp_log_level_set(TAG, LOG_LEVEL_LOCAL);
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

// // // // <------ DO not use vl53l0x__Calibrate
VL53L0X_Error vl53l0x__Calibrate(VL53L0X_DEV Dev) {
  VL53L0X_Error Status = VL53L0X_ERROR_NONE;
  Status = VL53L0X_PerformRefCalibration(
      Dev, &VL53L0X_Calibration.pVhvSettings,
      &VL53L0X_Calibration.pPhaseCal); // Device Initialization

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

  ESP_LOGI(TAG, "vl53l0x__InitializeDriver");

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

///////////////////////////
VL53L0X_Error
vl53l0x__read_calibration(VL53L0X_DEV Dev,
                          VL53L0X_Calibration_t *calibration_data) {
  // ReferenceSpads
  VL53L0X_Error Status = VL53L0X_GetReferenceSpads(
      Dev, &calibration_data->refSpadCount, &calibration_data->isApertureSpads);
  if (Status != VL53L0X_ERROR_NONE) {
    return print_pal_error(Status, "VL53L0X_GetReferenceSpads");
  }

  // Temp
  Status = VL53L0X_GetRefCalibration(Dev, &calibration_data->pVhvSettings,
                                     &calibration_data->pPhaseCal);
  if (Status != VL53L0X_ERROR_NONE) {
    return print_pal_error(Status, "VL53L0X_GetRefCalibration");
  }

  // Offset
  Status = VL53L0X_GetOffsetCalibrationDataMicroMeter(
      Dev, &calibration_data->pOffsetMicroMeter);
  if (Status != VL53L0X_ERROR_NONE) {
    return print_pal_error(Status,
                           "VL53L0X_GetOffsetCalibrationDataMicroMeter");
  }

  // XTalk
  Status = VL53L0X_GetXTalkCompensationRateMegaCps(
      Dev, &calibration_data->pXTalkCompensationRateMegaCps);
  if (Status != VL53L0X_ERROR_NONE) {
    return print_pal_error(Status, "VL53L0X_GetXTalkCompensationRateMegaCps");
  }

  return Status;
}