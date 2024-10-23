# VL53L0X-ESP32-PORT

VL53L0X STM driver port as an ESP-IDF component

## Dependency

- [ESP-IDF](https://github.com/espressif/esp-idf)

## Install

Create a `components` directory in your ESP32 project

```sh
mkdir components
cd components
git clone https://github.com/obe711/VL53L0X-ESP32-PORT.git
```

Add this line to the `CMakeLists.txt` in your ESP32 project directory

```
set(EXTRA_COMPONENT_DIRS ./components/VL53L0X-ESP32-PORT)
```

## API Reference

[STMicroelectronics official VL53L0X API](./stsw-img005.pdf)

[VL53L0X - Datasheet](./vl53l0x.pdf)
