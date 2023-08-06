# AWS
Creating a Automatic Weather Station - DIY
IISER Weather Station workshop 5-6 Aug 2023

Steps to configure

1.	Install Arduino software
2.	Install ESP32 Board, ESP32 by Espressif Systems (This depends on device we are using. This device is ESP32 Dev Kit)
3.	Select Board Manager: ESP32S2 Dev Module
4.	Check if the code blink.ino in examples is compiling. If yes, then use one port to upload this compiled binary file to the ESP32 device.
5.	If it doesn’t work, Install Port Silicon Labs CP210x (https://www.silabs.com/developers/usb-to-uart-bridge-vcp-drivers?tab=downloads). This helps in communication of your OS port with the ESP32 port.
6.	Run Blink code
7.	Run Buzzer code. Buzzer: https://esp32io.com/tutorials/esp32-piezo-buzzer
8.	Connect Temperature Sensor and run the code. Load libraries needed. 


https://www.instructables.com/Solar-Powered-WiFi-Weather-Station-V40/
To understand: 
https://lastminuteengineers.com/bme280-arduino-tutorial/

9. Publish the information to device: Thingsboard or Arduino board platform
10. check steps here: https://thingsboard.io/docs/devices-library/esp32-dev-kit-v1/
