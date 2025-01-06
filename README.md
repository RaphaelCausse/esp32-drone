# ESP32 DRONE

Developping and building a quadcopter drone, based on the ESP32-S3 SoC.

Components used:
- ESP32-S3-devKitC-1 N16R8
- MPU 6050 GY-521
- VL53L0X

Using Visual Studio Code on Ubuntu Linux 24.04 LTS.


# TABLE OF CONTENT

* [Setup](#setup)
* [Installation](#installation)
* [Links](#links)


# SETUP

Install prerequisites for Ubuntu :
```
sudo apt-get install git wget flex bison gperf python3 python3-pip python3-venv cmake ninja-build ccache libffi-dev libssl-dev dfu-util libusb-1.0-0
```


# INSTALLATION

Open Visual Studio Code.

Install extension "ESP-IDF" from Espressif Systems.

Select "EXPRESS" setup mode.

Select the ESP-IDF version.

Click "Install" button and wait for the installation to finish.

__Note :__

For Linux users, a message is shown to add OpenOCD rules in /etc/udev/rules.d which you need to run with sudo privileges.


## LINKS

[ESP-IDF Programming Guide](https://docs.espressif.com/projects/esp-idf/en/stable/esp32s3/index.html)

[ESP32-S3-devKitC-1 Guide](https://docs.espressif.com/projects/esp-idf/en/v5.3.1/esp32s3/hw-reference/esp32s3/user-guide-devkitc-1.html)