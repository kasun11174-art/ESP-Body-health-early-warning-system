# IoT-Based Continuous Vital Monitoring System

This project implements a wearable health monitoring system using an ESP32 microcontroller, MAX30105 optical sensor, and OLED display. It continuously measures heart rate (HR), body temperature, and displays readings locally, while supporting real-time monitoring with adjustable limits.

## Features

* **Heart Rate Measurement:** Uses MAX30105 sensor with moving average filtering and peak detection.
* **Temperature Measurement:** Currently simulated; can be replaced with real sensor data.
* **OLED Display:** Displays readings and configurable thresholds, navigable via buttons.
* **User Interaction:** Buttons allow changing display modes and adjusting HR/temperature limits.
* **Real-time Processing:** Tasks run on FreeRTOS for multitasking and I²C mutex protection.
* **Limits & Alerts:** Configurable HR and temperature thresholds.

## Hardware

* ESP32 microcontroller
* MAX30105 optical sensor
* SSD1306 128x64 OLED display
* Pushbuttons (D14, D27, D26, D25, D33, D32)

## Software & Libraries

* Arduino IDE / PlatformIO
* `Wire.h` for I2C communication
* `Adafruit_GFX` and `Adafruit_SSD1306` for OLED
* `MAX30105.h` for sensor interaction
* FreeRTOS tasks for multitasking

## Installation

1. Connect the MAX30105 sensor and OLED display to ESP32 using I2C.
2. Connect pushbuttons to D14, D27, D26, D25, D33, D32.
3. Install required libraries: `Adafruit_GFX`, `Adafruit_SSD1306`, `MAX30105`.
4. Upload the code to ESP32.
5. Open Serial Monitor at 115200 baud to observe debug messages.

## Usage

* **Display Modes:** Navigate between sensor readings, set HR limits, set temperature limits, and view all limits using buttons.
* **Adjust Limits:** Increase/decrease HR or temperature limits using designated buttons.
* **Monitoring:** The system continuously measures HR and temperature, displaying values on OLED.

## Future Improvements

* Integrate a real temperature sensor.
* Add cloud connectivity for remote monitoring.
* Implement emergency alerts based on thresholds.
* Integrate SpO₂ and ML-based health risk assessment.

## License

This project can be released under the MIT License for open-source sharing and contributions.
