# Airthings Wave Local Monitor (ESP32)

### 📥 [Download the Source Code, Firmware, and Build Guide Here](https://ilnixadmin.gumroad.com/l/kcuaym)

---

![20200302_060545980_iOS](https://github.com/user-attachments/assets/50ea0840-b2ee-4925-b53c-92591b04109b)

**Description:**
A standalone, local-only monitor for Airthings Wave devices.
* **Hardware:** ESP32 + 2.4" TFT
* **Connectivity:** MQTT (SSL) + Home Assistant
* **Enclosure:** Custom clamshell (STLs included)

[Get the files here](https://ilnixadmin.gumroad.com/l/kcuaym)

## DEVELOPER NOTES - PLEASE READ BEFORE COMPILING

This is a technical project for intermediate/advanced makers.
The firmware is functional but requires specific configuration environment setup.

CRITICAL SETUP STEPS:

1. LIBRARY DEPENDENCIES (TFT_eSPI)
   - This project uses the Bodmer/TFT_eSPI library.
   - You MUST edit the "User_Setup_Select.h" file inside your Arduino libraries folder.
   - Uncomment the line corresponding to your display driver (e.g., ILI9341 or ILI9488).
   - If you skip this, the screen will remain white.

2. FIRST TIME BOOT (EEPROM RESET)
   - The device stores settings in EEPROM. On a fresh ESP32, this memory is empty.
   - Open "main.cpp", find "#define EEPROM_INIT false".
   - Set it to TRUE and flash the device once to format the memory.
   - Set it back to FALSE and flash again for normal operation.

3. SSL CERTIFICATES (MQTT)
   - The code uses secure MQTT (SSL/TLS).
   - The Root CA certificate is hardcoded in "main.cpp" (variable: digicert).
   - You MUST update this certificate if your MQTT broker uses a different authority (e.g., Let's Encrypt).
   - To disable SSL (not recommended), modify "wifi_SetSecure" to use a standard WiFiClient.

4. WIFI CONFIGURATION
   - Do not hardcode WiFi credentials.
   - On first boot, connect to the "SYBRASET-XXXX" hotspot.
   - Use the captive portal to enter your WiFi and MQTT details.

5. SPLASH SCREEN
   - The files "splash-ili9341.h" and "splash-ili9488.h" contain the boot logos for their respective displays.
   - Ensure the included header matches your screen resolution, or generate a new array using an "Image2Cpp" converter.

DISCLAIMER:
This code is provided "as is" for educational purposes. It works on the hardware specified in the BOM.

### A NOTE ON THE BACKEND ARCHITECTURE

You may notice references to "mqtt_cmd" and OTA updates in the code.
This firmware is designed to run standalone (Home Assistant), but it was
originally built to pair with a custom "Headless Fleet Manager" written
in Go.

That backend handles:
 - Logging of device data
 - Centralized OTA firmware dispatch.
 - Remote fleet command-and-control.
 - Sending and receiving MQTT data to and from the device

I am considering cleaning up and releasing that Server Stack as an
"Advanced Architecture Package" for developers interested in Go/IoT
systems.

If you want to see this released, please leave a rating on Gumroad or
drop me a message. If there's enough interest, I'll ship it.

Happy Building.
