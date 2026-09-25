# SmartPod Firmware

This folder contains the SmartPod ESP32 firmware source code.

## Firmware

- `SmartPod-v1.0.0.ino` — Main firmware source code.

## Board

- ESP32-WROOM-32
- Arduino IDE: **ESP32 Dev Module**

## Required Libraries

- ESP32-A2DP
- AudioTools
- arduino-libhelix
- U8g2

## Configuration

Before uploading, set the Bluetooth device name in the firmware:

```cpp
const char EXACT_BT_NAME[] = "Your Device Name";
