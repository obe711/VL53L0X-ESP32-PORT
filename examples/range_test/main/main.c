
#include "driver/i2c.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "vl53l0x-esp32-port.h"
#include <stdio.h>
#include <string.h>

#define TAG "range_test"

#define RANGE_TEST_SDA_GPIO GPIO_NUM_10
#define RANGE_TEST_SCL_GPIO GPIO_NUM_9

#define WRITE_BIT I2C_MASTER_WRITE /*!< I2C master write */
#define READ_BIT I2C_MASTER_READ   /*!< I2C master read */
#define ACK_CHECK_EN 0x1           /*!< I2C master will check ack from slave*/
#define ACK_CHECK_DIS 0x0 /*!< I2C master will not check ack from slave */
#define ACK_VAL 0x0       /*!< I2C ack value */
#define NACK_VAL 0x1      /*!< I2C nack value */
#define I2C_MASTER_TX_BUF_DISABLE 0 /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE 0 /*!< I2C master doesn't need buffer */

#define ESP_SLAVE_ADDR 0x40 /*!< Slave address, you can set any 7bit value */

static int i2c_master_port = 1;

esp_err_t init_i2c(void) {

  i2c_config_t conf = {
      .mode = I2C_MODE_MASTER,
      .sda_io_num = RANGE_TEST_SDA_GPIO,
      .sda_pullup_en = GPIO_PULLUP_ENABLE,
      .scl_io_num = RANGE_TEST_SCL_GPIO,
      .scl_pullup_en = GPIO_PULLUP_ENABLE,
      .master.clk_speed = 400000,
  };
  esp_err_t err = i2c_param_config(i2c_master_port, &conf);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "I2C Init FAIL");
    return err;
  }
  return i2c_driver_install(i2c_master_port, conf.mode,
                            I2C_MASTER_RX_BUF_DISABLE,
                            I2C_MASTER_TX_BUF_DISABLE, 0);
}

VL53L0X_Error rangingTest(VL53L0X_Dev_t *pMyDevice) {
  VL53L0X_RangingMeasurementData_t RangingMeasurementData;
  VL53L0X_Error Status = vl53l0x__Start(pMyDevice);
  if (Status != VL53L0X_ERROR_NONE)
    return Status;

  do {
    if (Status == VL53L0X_ERROR_NONE) {
      Status = vl53l0x__GetDistance(pMyDevice, &RangingMeasurementData);
      if (Status == VL53L0X_ERROR_NONE) {
        ESP_LOGI(TAG, "TOF Reading: %d",
                 RangingMeasurementData.RangeMilliMeter);

        // Clear the interrupt
        vl53l0x__Clear(pMyDevice);
      } else {
        ESP_LOGE(TAG, "Range Test Failed");
        break;
      }
    }
    vTaskDelay(pdMS_TO_TICKS(50));
  } while (1);

  return vl53l0x__Stop(pMyDevice);
}

void auto_detect_task(void *args) {
  VL53L0X_DEV Dev = vl53l0x__CreateDevice(i2c_master_port);

  VL53L0X_Error Status = VL53L0X_ERROR_NONE;
  VL53L0X_Version_t Version;
  VL53L0X_DeviceInfo_t DeviceInfo;

  if (vl53l0x__InitializeDriver(Dev, &Version, &DeviceInfo) !=
      VL53L0X_ERROR_NONE) {
    ESP_LOGE(TAG, "VL53l0X Failed to initialize");
    vTaskDelete(NULL);
  }

  ESP_LOGI(TAG, "VL53L0X_GetDeviceInfo:");
  ESP_LOGI(TAG, "Device Name : %s", DeviceInfo.Name);
  ESP_LOGI(TAG, "Device Type : %s", DeviceInfo.Type);
  ESP_LOGI(TAG, "Device ID : %s", DeviceInfo.ProductId);
  ESP_LOGI(TAG, "ProductRevisionMajor : %d", DeviceInfo.ProductRevisionMajor);
  ESP_LOGI(TAG, "ProductRevisionMinor : %d", DeviceInfo.ProductRevisionMinor);

  if ((DeviceInfo.ProductRevisionMinor != 1) &&
      (DeviceInfo.ProductRevisionMinor != 1)) {
    ESP_LOGE(TAG, "Error expected cut 1.1 but found cut %d.%d",
             DeviceInfo.ProductRevisionMajor, DeviceInfo.ProductRevisionMinor);
    vTaskDelete(NULL);
  }

  while (1) {
    if (Status == VL53L0X_ERROR_NONE) {
      Status = rangingTest(Dev);
    } else {
      ESP_LOGE(TAG, "Range Test Failed");
      vTaskDelete(NULL);
    }
    vTaskDelay(pdMS_TO_TICKS(250));
  }
}

void app_main(void) {
  init_i2c();

  xTaskCreate(auto_detect_task, "tof_task", 4095, NULL, 5, NULL);
}