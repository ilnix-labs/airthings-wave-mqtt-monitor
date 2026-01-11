/**
   A sketch that searches for a compatible Airthings device and
   publishes the sensor data to an MQTT server.

   To use:
   (1) Set up your Airthings following the manufacter's instructions.
   (2) Install the PunSubClient library (https://pubsubclient.knolleary.net/).

   * The library runs once an hour to take a reading and deep sleeps in
     between, so feasibly this could run on a battery for a very long time.
   * The library will attempt to find any airthings device to read from,
     picking the first it finds. The Airthings BLE API is unauthenticated
     so no device configuration or pairing is necessary on the Airthings.
   * The library will not interfere with your Airthings' normal upload to a
     phone/cloud.
   * If it fails to read, it will attempt again after 30 seconds instead.
   * The ESP32's bluetooth stack is a little unstable IMHO so expect this to
     hang for a few minutes, restart prematurely, and report errors often.
*/

#define SSD1306 1 // SSD1306 or SH1106
#define SH1106 2
#define ILI9341 3
#define ILI9488 4
#define DISPLAY ILI9341 // ILI9341 (2.4") or ILI9488 (3.5")

#include <WiFiClientSecure.h>
#include <PubSubClient.h>
#include <BLEDevice.h>
#include <time.h>
#include <math.h>
#include <SPI.h>
#include <Wire.h>
#include <WiFiManager.h> // WiFi Configuration Magic (  https://github.com/zhouhan0126/DNSServer---esp32  ) >> https://github.com/zhouhan0126/DNSServer---esp32  (ORIGINAL)
#include <EEPROM.h>
#include <HTTPUpdate.h>
#include <Ticker.h>

#include "splash-ili9341.h"

#if DISPLAY == ILI9488 || DISPLAY == ILI9341
// NOTE: Make sure User_Setup_Select.h contains #if DISPLAY xxx if overwrite.
// TODO: #include correct file in User_Setup_Select.h before compiling. Figure out how to have to set all the time.
#include "TFT_eSPI.h"
#elif DISPLAY == SSD1306
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#elif DISPLAY == SH1106
#include <Adafruit_GFX.h>
#include <Adafruit_SH1106.h>
#endif

// -----------------------------------------------------------------------------
// Debug.
// -----------------------------------------------------------------------------

#define DEBUG true

#if DEBUG == true
#define D(x) do { x; } while(0)
#else
#define D(x) do { } while(0)
#endif

// -----------------------------------------------------------------------------
// Defines and constants.
// -----------------------------------------------------------------------------

#define MODEL "SYBRASET-AW01A" // AW01A 2.4" or AW01B 3.5"
#define FW_VERSION "2021.04.05" // yy.mm.dd

#define EEPROM_INIT false
#define EEPROM_CONFIG 0x10

#define WIFIMANAGER_RESET false

#define SSD1306_I2C_ADDRESS (0x3C)

// Some useful constants.
#define uS_TO_S_FACTOR 1000000
#define SECONDS_TO_MILLIS 1000
#define BECQUERELS_M2_IN_PICOCURIES_L 37.0F // 1 pCi/L = 37 Bq/m3.

// If taking a reading fails for any reason (BLE is pretty flaky...) then the ESP will sleep for this long before retrying.
#define READ_WAIT_RETRY_SECONDS 30

// How long the ESP will wait to connect to WiFi, MQTT, scan for Airthings devices, etc.
#define CONNECT_WAIT_SECONDS 30

#define BLUETOOTH_LED 2

// Language.
enum Localization {
    en,
    fr
};
// Measurement system.
enum MSystem {
    metric,
    imperial
};
// Airthing Wave model.
enum AWModel {
    unknown,
    wave,
    wave2,
    waveplus,
    wavemini
};
// Airthing Wave main sensor status.
enum AWStatus {
    all,
    temperature,
    humidity,
    radon,
    co2,
    tvoc
};

// Localization string id's.
#define STR_AIR_QUALITY_IS_GOOD 1
#define STR_AIR_QUALITY_IS_OK 2
#define STR_AIR_QUALITY_IS_POOR 3
#define STR_SENSORS_CALIBRATING 4
#define STR_PLIFEP 5
#define STR_HUMIDITY 6
#define STR_PRESSURE 7
#define STR_SEARCHING_AIRTHINGS 8
#define STR_SEARCHING_AIRTHINGS_SN 9
#define STR_WIFI_CONNECT 10
#define STR_FW_UPDATE 11

// -----------------------------------------------------------------------------
// Global variables.
// -----------------------------------------------------------------------------

// Device config (stored in eeprom).
// NOTE: Don't use pointers as they won't be copied.
struct Eeprom {
    bool fw_updated; // was fw updated?
    char mqtt_ci[33]; // auth token.
    char aw_sn[11]; // serial number of airthings wave device.
    char user_email[80]; // email specified by user on wifimanager screen.
    char custom_msg[22]; // custom message displayed bottom of screen (max 21 chars). e.g. location, etc.
    int8_t t_offset; // temperature offset (in degrees). e.g. 11
    int8_t h_offset; // humidity offset (in %). e.g. -2
    unsigned int time_to_sleep; // how long to sleep (in seconds).
    Localization lang; // Language.
    MSystem msystem; // Measurement system. e.g. metric, imperial systems.
    // Sensor thresholds.
    AWStatus aw_status;
    int16_t thld_t1, thld_t2;
    uint8_t thld_h1, thld_h2, thld_h3, thld_h4;
    uint16_t thld_r1, thld_r2;
    uint16_t thld_co21, thld_co22;
    uint16_t thld_tvoc1, thld_tvoc2;
    // Display.
    unsigned char display_rotation; // Display orientation.
    unsigned char display_brightness; // Display brightness (0-255).
    unsigned char display_brightness_ontime; // Display brightness at "display_ontime" (0-255)(if set).
    unsigned char display_brightness_offtime; // Display brightness at "display_offtime" (0-255)(if set).
    uint16_t display_ontime; // What time to turn the display on (0 = always on, 0-2459)(in hhmm, e.g. 1800 = 18hh*100+0).
    uint16_t display_offtime; // What time to turn the display off (0 = always on, 0-2459)(in hhmm, e.g. 1118 = 11h*100+18).
};
Eeprom default_config_eeprom;

// Documentation says radon lifetime will be the average since the batteries were removed.
struct SensorAirthingsWave {
    AWModel model;

    float r24; // radon 24 hours.
    float rlife; // radon life (since battery changed).
    float tvoc; // total volatile organic compound.
    float co2; // co2 (carbon dioxide).
    float t; // temperature.
    float h; // humidity.
    float p; // pressure.
};

#if DISPLAY == ILI9488 || DISPLAY == ILI9341
struct Display {
    TFT_eSPI tft = TFT_eSPI();
    short brightness = -1;
};
#else if DISPLAY == SSD1306 || DISPLAY == SH1106
struct Display {
    const uint8_t SCREEN_WIDTH = 128; // OLED display width, in pixels.
    const uint8_t SCREEN_HEIGHT = 64; // OLED display height, in pixels.
    const int8_t OLED_RESET = -1; // Reset pin # (or -1 if sharing Arduino reset pin).

    TwoWire screen;

    #if DISPLAY == SSD1306
    Adafruit_SSD1306 display;
    #elif DISPLAY == SH1106
    Adafruit_SH1106 display;
    #endif
};
#endif

struct Config {
    String mqtt_svr; // mqtt server.
    String mqtt_un; // mqtt username.
    String mqtt_pw; // mqtt password.
    String mqtt_rt; // mqtt receive topic (rx).
    String mqtt_tt; // mqtt transmit topic (tx).
    unsigned int mqtt_port; // mqtt server port.
    unsigned int fs_port; // file server port.

    bool wifi_save_config; // wifimanager flag for saving data.
    bool restart_device; // flag for restarting device.
    bool reset_wifi; // flag for resetting wifi manager settings.
    bool refresh_display; // flag for refreshing the display.
    String fw_update_binfile; // set with binfile when we want to update fw.
    unsigned int time_to_sleep; // how long to sleep (in seconds) in loop().

    // MQTT TX commands sent to server.
    const char *mqtt_cmd_initdevice;
    const char *mqtt_cmd_status;
    const char *mqtt_cmd_sensordata;

    struct tm current_time;

    Display display;
    SensorAirthingsWave aw;
    Eeprom eeprom;
};
Config config = {
    "iot.sybraset.com",
    "sybraset",
    "teScIALieNeFRO",
    "/rx",
    "/tx",
    8883,
    8100,
    false,
    false,
    false,
    false,
    "",
    0,
    "init-device",
    "status",
    "sensor-data",
    {},
    {},
    {},
    {}
};

WiFiClientSecure ssl_client;
PubSubClient *pub_client;

static const char *digicert PROGMEM = R"EOF(
-----BEGIN CERTIFICATE-----
MIIFFjCCAv6gAwIBAgIRAJErCErPDBinU/bWLiWnX1owDQYJKoZIhvcNAQELBQAw
TzELMAkGA1UEBhMCVVMxKTAnBgNVBAoTIEludGVybmV0IFNlY3VyaXR5IFJlc2Vh
cmNoIEdyb3VwMRUwEwYDVQQDEwxJU1JHIFJvb3QgWDEwHhcNMjAwOTA0MDAwMDAw
WhcNMjUwOTE1MTYwMDAwWjAyMQswCQYDVQQGEwJVUzEWMBQGA1UEChMNTGV0J3Mg
RW5jcnlwdDELMAkGA1UEAxMCUjMwggEiMA0GCSqGSIb3DQEBAQUAA4IBDwAwggEK
AoIBAQC7AhUozPaglNMPEuyNVZLD+ILxmaZ6QoinXSaqtSu5xUyxr45r+XXIo9cP
R5QUVTVXjJ6oojkZ9YI8QqlObvU7wy7bjcCwXPNZOOftz2nwWgsbvsCUJCWH+jdx
sxPnHKzhm+/b5DtFUkWWqcFTzjTIUu61ru2P3mBw4qVUq7ZtDpelQDRrK9O8Zutm
NHz6a4uPVymZ+DAXXbpyb/uBxa3Shlg9F8fnCbvxK/eG3MHacV3URuPMrSXBiLxg
Z3Vms/EY96Jc5lP/Ooi2R6X/ExjqmAl3P51T+c8B5fWmcBcUr2Ok/5mzk53cU6cG
/kiFHaFpriV1uxPMUgP17VGhi9sVAgMBAAGjggEIMIIBBDAOBgNVHQ8BAf8EBAMC
AYYwHQYDVR0lBBYwFAYIKwYBBQUHAwIGCCsGAQUFBwMBMBIGA1UdEwEB/wQIMAYB
Af8CAQAwHQYDVR0OBBYEFBQusxe3WFbLrlAJQOYfr52LFMLGMB8GA1UdIwQYMBaA
FHm0WeZ7tuXkAXOACIjIGlj26ZtuMDIGCCsGAQUFBwEBBCYwJDAiBggrBgEFBQcw
AoYWaHR0cDovL3gxLmkubGVuY3Iub3JnLzAnBgNVHR8EIDAeMBygGqAYhhZodHRw
Oi8veDEuYy5sZW5jci5vcmcvMCIGA1UdIAQbMBkwCAYGZ4EMAQIBMA0GCysGAQQB
gt8TAQEBMA0GCSqGSIb3DQEBCwUAA4ICAQCFyk5HPqP3hUSFvNVneLKYY611TR6W
PTNlclQtgaDqw+34IL9fzLdwALduO/ZelN7kIJ+m74uyA+eitRY8kc607TkC53wl
ikfmZW4/RvTZ8M6UK+5UzhK8jCdLuMGYL6KvzXGRSgi3yLgjewQtCPkIVz6D2QQz
CkcheAmCJ8MqyJu5zlzyZMjAvnnAT45tRAxekrsu94sQ4egdRCnbWSDtY7kh+BIm
lJNXoB1lBMEKIq4QDUOXoRgffuDghje1WrG9ML+Hbisq/yFOGwXD9RiX8F6sw6W4
avAuvDszue5L3sz85K+EC4Y/wFVDNvZo4TYXao6Z0f+lQKc0t8DQYzk1OXVu8rp2
yJMC6alLbBfODALZvYH7n7do1AZls4I9d1P4jnkDrQoxB3UqQ9hVl3LEKQ73xF1O
yK5GhDDX8oVfGKF5u+decIsH4YaTw7mP3GFxJSqv3+0lUFJoi5Lc5da149p90Ids
hCExroL1+7mryIkXPeFM5TgO9r0rvZaBFOvV2z0gp35Z0+L4WPlbuEjN/lxPFin+
HlUjr8gRsI3qfJOQFy/9rKIJR0Y/8Omwt/8oTWgy1mdeHmmjk7j1nYsvC9JSQ6Zv
MldlTTKB3zhThV1+XWYp6rjd5JW1zbVWEkLNxE7GJThEUG3szgBVGP7pSWTUTsqX
nLRbwHOoq7hHwg==
-----END CERTIFICATE-----
)EOF";

// TODO: Update these with your own device serial numbers.
// #define BASEMENT01_SN "2900129945"
// #define BASEMENT02_SN "2900153030"
// #define BASEMENT03_SN "2900123123"
// #define BASEMENT04_SN "2900263509"
// #define BASEMENT05_PLUS_SN "2930014818"
// #define BASEMENT06 "2950005958"

// The hard-coded uuid's airthings uses to advertise itself and its data.
static BLEUUID waveUUID("b42e1f6e-ade7-11e4-89d3-123b93f75cba");
static BLEUUID wave2UUID("b42e4a8e-ade7-11e4-89d3-123b93f75cba");
static BLEUUID wave2charUUID("b42e4dcc-ade7-11e4-89d3-123b93f75cba");
static BLEUUID waveplusUUID("b42e1c08-ade7-11e4-89d3-123b93f75cba");
static BLEUUID wavepluscharUUID("b42e2a68-ade7-11e4-89d3-123b93f75cba");

// -----------------------------------------------------------------------------
// Function prototypes.
// -----------------------------------------------------------------------------

void ble_ScanForAirthings(void);
bool ble_GetSensorData(BLEAddress pAddress);

void display_Init(Display *display);
void display_SetOnOff(Display *display, uint16_t ontime, uint16_t offtime);
void display_SetBrightness(Display *display, unsigned char brightness);
void printStatusMsg(Display *display, const char *msg);
void printSensorData(Display *display, const Eeprom *config, const SensorAirthingsWave *aw, MSystem msystem);

void tick(void);
bool wifi_Init(WiFiClientSecure *ssl_client);
bool wifi_Connect(WiFiClientSecure *ssl_client, bool show_connect_msg = true);
void wifi_Disconnect(WiFiClientSecure *ssl_client);
void wifi_SaveConfigCallback(void);
bool wifi_IsWifi(void);
void wifi_SetSecure(WiFiClientSecure *ssl_client, const char *cert);
String wifi_GetMAC(void);
int8_t wifi_GetQuality(void);

void eeprom_Init(Eeprom *default_config, Eeprom *config);
bool eeprom_IsDeviceInitialized(Eeprom *config);
void eeprom_InitDefaultConfig(Eeprom *default_config);
void eeprom_GetConfig(Eeprom *config);
void eeprom_StoreConfig(Eeprom *config);
void eeprom_Reset(void);
void eeprom_Print(void);
void eeprom_PrintConfig(Eeprom *config);

PubSubClient *mqtt_Init(WiFiClientSecure *ssl_client);
void mqtt_Connect(PubSubClient *client);
void mqtt_Processor(PubSubClient *client);
void mqtt_Callback(char *topic, byte *payload, unsigned int length);
bool mqtt_Send(PubSubClient *client, String payload);

uint http_Get(WiFiClientSecure *ssl_client, const char *server, int port, const char *resource, char *dst = NULL, size_t dst_length = 0, bool insecure = false);

void ota_UpdateFw(WiFiClientSecure *ssl_client, String binfile);

void util_SetClockWithNTP(struct tm *ctime);
void util_GetCurrentTime(struct tm *ctime);
void util_SplitString(String *src_str, String *arr, size_t arr_length, char *sep);
char *util_Trim(char *str);
int util_Char2Int(char input);
void util_Hex2Bin(const char *src, char *target);

// -----------------------------------------------------------------------------
// Main functions.
// -----------------------------------------------------------------------------
bool show_wifi_connect_msg = true;

void setup() {
    D(Serial.begin(115200));

    setCpuFrequencyMhz(80);

    eeprom_Init(&default_config_eeprom, &config.eeprom);

    display_Init(&config.display);

    if (eeprom_IsDeviceInitialized(&config.eeprom) == false) {
        printStatusMsg(&config.display, (String("Welcome to Sybraset Airthings Dashboard!\n\nTo setup:\n 1. Connect to Wifi hotspot \"SYBRASET-???\" using phone, tablet or laptop\n 2. Enter Wifi password: ") + wifi_GetMAC() + "\n 3. Enter your Wifi settings and email address\n 4. Device should restart").c_str());
        show_wifi_connect_msg = false;
    }

    wifi_Init(&ssl_client);

    pub_client = mqtt_Init(&ssl_client);

    // Turn off bluetooth led.
    pinMode(BLUETOOTH_LED, OUTPUT);
    digitalWrite(BLUETOOTH_LED, LOW);
}

void loop() {
    wifi_Connect(&ssl_client, show_wifi_connect_msg);
    // TODO: Why do I need to run this twice to pick up msgs I'm subscribed to?
    for (int i=0; i < 2; i++) {
        mqtt_Processor(pub_client);
        delay(100);
    };
    display_SetOnOff(&config.display, config.eeprom.display_ontime, config.eeprom.display_offtime);
    if (eeprom_IsDeviceInitialized(&config.eeprom) == true) {
        if (config.fw_update_binfile != "") {
            ota_UpdateFw(&ssl_client, config.fw_update_binfile);
        }
        if (config.restart_device == true) {
            if (config.reset_wifi == true) {
                WiFiManager wifi_manager;
                wifi_manager.resetSettings();
                delay(1000);
            }
            delay(1000);
            ESP.restart();
            delay(5000);
        }
        if (config.time_to_sleep > 0) {
            if (config.aw.model != unknown) {
                // Send data to server.
                mqtt_Send(pub_client, String(config.mqtt_cmd_sensordata) + String(",") +
                    String(config.aw.r24) + String(",") +
                    String(config.aw.rlife) + String(",") +
                    String(config.aw.tvoc) + String(",") +
                    String(config.aw.co2) + String(",") +
                    String(config.aw.h) + String(",") +
                    String(config.aw.t) + String(",") +
                    String(config.aw.p));

                // Display data on screen.
                printSensorData(&config.display, &config.eeprom, &config.aw, config.eeprom.msystem);
                config.refresh_display = false;
                yield();
            }
            D(Serial.printf("\nSleeping for %i seconds.\n", config.time_to_sleep));
            D(Serial.flush());
            // Sleep.
            // TODO: https://github.com/espressif/arduino-esp32/issues/1113 (check deep sleep power consumption)
            // https://www.savjee.be/2019/12/esp32-tips-to-increase-battery-life/#tip-3-pick-the-right-esp32-board-single-core
            // esp_sleep_enable_timer_wakeup(config.time_to_sleep * uS_TO_S_FACTOR);
            // esp_deep_sleep_start();

            // wifi_Disconnect(&ssl_client);
            unsigned long start_tts = millis();
            while (millis() < start_tts + config.time_to_sleep * SECONDS_TO_MILLIS) {
                // unsigned long start_tts2 = millis();
                // unsigned int sleep_tts2 = (config.time_to_sleep == READ_WAIT_RETRY_SECONDS) ? READ_WAIT_RETRY_SECONDS : 60*15;
                // while (millis() < start_tts2 + (sleep_tts2-5) * SECONDS_TO_MILLIS) {
                //     delay(1000);
                // }

                wifi_Connect(&ssl_client, show_wifi_connect_msg);
                // for (int i=0; i < 5; i++) {
                    mqtt_Processor(pub_client);
                    // delay(1000);
                // };
                display_SetOnOff(&config.display, config.eeprom.display_ontime, config.eeprom.display_offtime);
                if (config.fw_update_binfile != "") {
                    ota_UpdateFw(&ssl_client, config.fw_update_binfile);
                }
                if (config.restart_device == true) {
                    if (config.reset_wifi == true) {
                        WiFiManager wifi_manager;
                        wifi_manager.resetSettings();
                        delay(1000);
                    }
                    delay(1000);
                    ESP.restart();
                    delay(5000);
                }
                if (config.refresh_display == true) {
                    if (config.aw.model != unknown) {
                        printSensorData(&config.display, &config.eeprom, &config.aw, config.eeprom.msystem);
                    }
                    config.refresh_display = false;
                }
                D(Serial.print(F(".")));
                delay(1000);
            }
            ESP.restart();
            delay(5000);
        }
        delay(100);

        wifi_Disconnect(&ssl_client);
        ble_ScanForAirthings();
        btStop();
        show_wifi_connect_msg = false;
    } else {
        delay(10000);
        mqtt_Send(pub_client, String(config.mqtt_cmd_initdevice) + "," + String(config.eeprom.user_email));
    }

    delay(1000);
}

// -----------------------------------------------------------------------------
// Local functions.
// -----------------------------------------------------------------------------

// -----------------------------------------------------------------------------
// Bluetooth functions.
// -----------------------------------------------------------------------------

// The bluetooth stack takes a callback when scannign for devices. The first Airthings device it finds it will record in pServerAddress.
class FoundDeviceCallback: public BLEAdvertisedDeviceCallbacks {
    public:
        BLEAddress *address;
        bool found = false;
    bool foundAirthings() {
        return found;
    }
    BLEAddress getAddress() {
        return *address;
    }
    void onResult(BLEAdvertisedDevice device) {
        // We have found a device, see if it has the Airthings service UUID.
        if (device.haveServiceUUID() && (device.getServiceUUID().equals(waveUUID) || device.getServiceUUID().equals(wave2UUID) || device.getServiceUUID().equals(waveplusUUID))) {
            char *manu_data = BLEUtils::buildHexData(nullptr, (uint8_t*)device.getManufacturerData().data(), device.getManufacturerData().length());
            char manu_data_hex[strlen(manu_data)/2];
            util_Hex2Bin(manu_data, manu_data_hex);
            free(manu_data);
            char serial_number[10];
            sprintf(serial_number, "%u", 256*256*256*(0x0000FF&manu_data_hex[5]) + 256*256*(0x0000FF&manu_data_hex[4]) + 256*(0x0000FF&manu_data_hex[3]) + (0x0000FF&manu_data_hex[2]));

            if (strcmp(config.eeprom.aw_sn, "") != 0) {
                if (strcmp(serial_number, config.eeprom.aw_sn) == 0) {
                    D(Serial.print(F("Found our device(sn): ")));
                    D(Serial.println(device.toString().c_str()));
                    device.getScan()->stop();
                    address = new BLEAddress(device.getAddress());
                    found = true;
                } else {
                    D(Serial.print(F("Wrong airthings device: ")));
                    D(Serial.println(device.toString().c_str()));
                    D(Serial.print(F("Serial number: ")));
                    D(Serial.println(serial_number));
                }
            } else {
                D(Serial.print(F("Found our device (1st): ")));
                D(Serial.println(device.toString().c_str()));
                device.getScan()->stop();
                address = new BLEAddress(device.getAddress());
                found = true;
            }
        }
    }
};

// Scan for an Airthings device.
void ble_ScanForAirthings(void) {
    String status_msg;
    if (strcmp(config.eeprom.aw_sn, "") == 0) {
        status_msg = i10n_GetText(STR_SEARCHING_AIRTHINGS, config.eeprom.lang);
    } else {
        status_msg = i10n_GetText(STR_SEARCHING_AIRTHINGS_SN, config.eeprom.lang) + config.eeprom.aw_sn;
    }
    printStatusMsg(&config.display, (char *)status_msg.c_str());
    yield();

    BLEDevice::init("");
    BLEDevice::setPower(ESP_PWR_LVL_P7); // Set maximum bluetooth radio power level.
    BLEScan *pBLEScan = BLEDevice::getScan();
    FoundDeviceCallback *callback = new FoundDeviceCallback();
    pBLEScan->setAdvertisedDeviceCallbacks(callback);
    pBLEScan->setActiveScan(true);
    pBLEScan->start(30);

    unsigned int time_to_sleep = 0;
    if (callback->foundAirthings() == true && ble_GetSensorData(callback->getAddress())) {
        D(Serial.printf("\nReading complete. Sleeping for %i seconds before taking another reading.\n", config.eeprom.time_to_sleep));
        time_to_sleep = config.eeprom.time_to_sleep;
    } else if (callback->foundAirthings() == false) {
        // We timed out looking for an Airthings.
        D(Serial.printf("\nFAILED to find any Airthings devices. Sleeping for %i seconds before retrying.\n", READ_WAIT_RETRY_SECONDS));
        time_to_sleep = READ_WAIT_RETRY_SECONDS;
        config.aw.model = unknown;
    } else {
        D(Serial.printf("\nReading FAILED. Sleeping for %i seconds before retrying.\n", READ_WAIT_RETRY_SECONDS));
        time_to_sleep = READ_WAIT_RETRY_SECONDS;
        config.aw.model = unknown;
    }
    // time_to_sleep = 60; // debug.
    config.time_to_sleep = time_to_sleep;
}

bool ble_GetSensorData(BLEAddress pAddress) {
    D(Serial.println(F("\n[BLE] connecting...")));
    BLEClient *client = BLEDevice::createClient();
    yield();
    // Connect to the remote BLE Server.
    if (!client->connect(pAddress)) {
        D(Serial.println(F("[BLE] failed to connect.")));
        return false;
    }

    D(Serial.println(F("[BLE] connected.")));
    yield();
    // Obtain a reference to the service we are after in the remote BLE server.
    config.aw.model = wave;
    D(Serial.println(F("[BLE] retrieving service reference...")));
    BLERemoteService *pRemoteService = client->getService(waveUUID);
    D(Serial.println(F("[BLE] Is it Wave?")));
    if (pRemoteService == nullptr) {
        pRemoteService = client->getService(wave2UUID);
        D(Serial.println(F("[BLE] Is it Wave (v2)?")));
        config.aw.model = wave2;
        if (pRemoteService == nullptr) {
            pRemoteService = client->getService(waveplusUUID);
            D(Serial.println(F("[BLE] Is it Wave Plus?")));
            config.aw.model = waveplus;
            if (pRemoteService == nullptr) {
                D(Serial.println(F("[BLE] Airthings refused its service UUID.")));
                client->disconnect();
                return false;
            }
        }
    }
    // Get sensor data.
    if (config.aw.model == wave) {
        // The hard-coded uuid's airthings uses to advertise itself and its data.
        static BLEUUID radon24UUID("b42e01aa-ade7-11e4-89d3-123b93f75cba");
        static BLEUUID radonlifeUUID("b42e0a4c-ade7-11e4-89d3-123b93f75cba");
        static BLEUUID datetimeUUID((uint32_t)0x2A08);
        static BLEUUID temperatureUUID((uint32_t)0x2A6E);
        static BLEUUID humidityUUID((uint32_t)0x2A6F);
        // Get references to wave characteristics.
        D(Serial.println(F("[BLE] (wave)Reading radon/temperature/humidity...")));
        BLERemoteCharacteristic* temperatureCharacteristic = pRemoteService->getCharacteristic(temperatureUUID);
        BLERemoteCharacteristic* humidityCharacteristic = pRemoteService->getCharacteristic(humidityUUID);
        BLERemoteCharacteristic* radon24Characteristic = pRemoteService->getCharacteristic(radon24UUID);
        BLERemoteCharacteristic* radonLongTermCharacteristic = pRemoteService->getCharacteristic(radonlifeUUID);
        if (temperatureCharacteristic == nullptr ||
            humidityCharacteristic == nullptr ||
            radon24Characteristic == nullptr ||
            radonLongTermCharacteristic == nullptr) {
            D(Serial.print(F("Failed to read from the device!")));
            return false;
        }

        config.aw.t = ((short)temperatureCharacteristic->readUInt16()) / 100.0;
        config.aw.h = humidityCharacteristic->readUInt16() / 100.0;

        // The radon values are reported in terms of bqm3.
        // TODO: I think CAD values are in bqm3, whereas USA are in pcl?
        config.aw.r24 = radon24Characteristic->readUInt16() / BECQUERELS_M2_IN_PICOCURIES_L;
        config.aw.rlife = radonLongTermCharacteristic->readUInt16() / BECQUERELS_M2_IN_PICOCURIES_L;
    } else if (config.aw.model == wave2) {
        // Get references to wave2 characteristics.
        D(Serial.println(F("[BLE] (wave2)Reading radon/temperature/humidity...")));
        BLERemoteCharacteristic *allCharacteristic = pRemoteService->getCharacteristic(wave2charUUID);

        struct Wave2 {
            unsigned char version;
            unsigned char humidity;
            unsigned char padding[2];
            unsigned short radon24;
            unsigned short radonlife;
            unsigned short temperature;
        };

        if (allCharacteristic->canRead()) {
            std::string sstr = allCharacteristic->readValue();
            Wave2 *w2 = (Wave2 *)&sstr[0];

            D(Serial.println(w2->version));
            if (w2->version == 1) {
                D(Serial.println(w2->humidity));
                config.aw.h = w2->humidity / 2.0F;

                D(Serial.println(w2->radon24));
                config.aw.r24 = w2->radon24 / BECQUERELS_M2_IN_PICOCURIES_L;
                D(Serial.println(w2->radonlife));
                config.aw.rlife = w2->radonlife / BECQUERELS_M2_IN_PICOCURIES_L;

                D(Serial.println(w2->temperature));
                config.aw.t = w2->temperature / 100.0F;
            } else {
                D(Serial.println("[AW] error: unknown sensor version."));
            }
        } else {
            D(Serial.println(F("[BLE] can't read characteristics.")));
        }
    } else if (config.aw.model == waveplus) {
        // Get references to waveplus characteristics.
        D(Serial.println(F("[BLE] (waveplus)Reading radon/temperature/humidity/voc/co2/pressure...")));
        BLERemoteCharacteristic *allCharacteristic = pRemoteService->getCharacteristic(wavepluscharUUID);

        struct WavePlus {
            unsigned char version;
            unsigned char humidity;
            unsigned char padding[2];
            unsigned short radon24;
            unsigned short radonlife;
            unsigned short temperature;
            unsigned short pressure;
            unsigned short co2;
            unsigned short tvoc;
        };

        if (allCharacteristic->canRead()) {
            std::string sstr = allCharacteristic->readValue();
            WavePlus *wp = (WavePlus *)&sstr[0];

            D(Serial.println(wp->version));
            if (wp->version == 1) {
                D(Serial.println(wp->humidity));
                config.aw.h = wp->humidity / 2.0F;

                D(Serial.println(wp->radon24));
                config.aw.r24 = wp->radon24 / BECQUERELS_M2_IN_PICOCURIES_L;
                D(Serial.println(wp->radonlife));
                config.aw.rlife = wp->radonlife / BECQUERELS_M2_IN_PICOCURIES_L;

                D(Serial.println(wp->temperature));
                config.aw.t = wp->temperature / 100.0F;
                D(Serial.println(wp->pressure / 50));
                config.aw.p = wp->pressure / 50.0F;

                D(Serial.println(wp->co2));
                config.aw.co2 = wp->co2 * 1.0F;
                D(Serial.println(wp->tvoc));
                config.aw.tvoc = wp->tvoc * 1.0F;
            } else {
                D(Serial.println("[AW] error: unknown sensor version."));
            }
        } else {
            D(Serial.println(F("[BLE] can't read characteristics.")));
        }
    } else {
        D(Serial.println(F("[AW] unknown airthings device.")));
    }

    client->disconnect();

    // int a = (int) round(radon*BECQUERELS_M2_IN_PICOCURIES_L);
    // Serial.printf("Radon 24hr average: %d\n", a); // 1 pCi/L = 37 Bq/m3
    #if DEBUG == true
    Serial.printf("[AW] temperature: %.1f, (%f)\n", config.aw.t, config.aw.t);
    Serial.printf("[AW] humidity: %.1f, (%f)\n", round(config.aw.h), config.aw.h);
    Serial.printf("[AW] radon 24hr average: %d, (%f)\n", (int)(config.aw.r24*BECQUERELS_M2_IN_PICOCURIES_L), config.aw.r24);
    Serial.printf("[AW] radon lifetime average: %d, (%f)\n", (int)(config.aw.rlife*BECQUERELS_M2_IN_PICOCURIES_L), config.aw.rlife);
    #endif

    // De-init ble, otherwise not enough memory to run wifi.
    BLEDevice::deinit(true);

    return true;
}

// -----------------------------------------------------------------------------
// Display functions.
// -----------------------------------------------------------------------------

void display_Init(Display *display) {
    #if DISPLAY == ILI9488 || DISPLAY == ILI9341
    display->tft.init();
    display->tft.setRotation(config.eeprom.display_rotation);
    display->tft.fillScreen(TFT_BLACK);
    display->tft.setTextSize(2);

    display->tft.drawXBitmap(0, (TFT_HEIGHT - SPLASH_HEIGHT) / 2, splash, SPLASH_WIDTH, SPLASH_HEIGHT, TFT_WHITE);
    delay(5000);

    #elif DISPLAY == SSD1306
    display->display = Adafruit_SSD1306(display->SCREEN_WIDTH, display->SCREEN_HEIGHT, &display->screen, display->OLED_RESET);
    // SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally.
    if (!display->display.begin(SSD1306_SWITCHCAPVCC, SSD1306_I2C_ADDRESS)) {
        D(Serial.println(F("Display allocation failed.")));
        for (;;); // Don't proceed, loop forever.
    }
    display->display.setRotation(config.eeprom.display_rotation);
    display->display.setTextColor(WHITE, BLACK);
    // Show initial display buffer contents on the screen--the library initializes this with splash screen.
    display->display.display();
    #elif DISPLAY == SH1106
    display->display = Adafruit_SH1106(21, 22);
    display->begin(SH1106_SWITCHCAPVCC, SSD1306_I2C_ADDRESS);
    #endif
}

void display_SetOnOff(Display *display, uint16_t ontime, uint16_t offtime) {
    #if DISPLAY == ILI9488 || DISPLAY == ILI9341
    if (ontime != 0 && offtime != 0) {
        uint16_t current_time = config.current_time.tm_hour * 100 + config.current_time.tm_min; // hh * 100 + mm
        D(Serial.printf("[DISPLAY-current_time]: %d-%d-%d (%d)\n", config.current_time.tm_hour, config.current_time.tm_min, config.current_time.tm_sec, current_time));

        if (current_time >= config.eeprom.display_ontime && current_time < config.eeprom.display_offtime) {
            display_SetBrightness(&config.display, config.eeprom.display_brightness_ontime);
        } else {
            display_SetBrightness(&config.display, config.eeprom.display_brightness_offtime);
        }
    } else {
        display_SetBrightness(&config.display, config.eeprom.display_brightness);
    }
    // TODO: This code is to turn on/off power to the screen. Instead I use the brightness above. Is there a difference in power consumption when brightness = 0 which means screen is off?
    #if 0
    if (ontime != 0 && offtime != 0) {
        uint16_t current_time = config.current_time.tm_hour * 100 + config.current_time.tm_min; // hh * 100 + mm
        D(Serial.printf("[DISPLAY-current_time]: %d-%d-%d (%d)\n", config.current_time.tm_hour, config.current_time.tm_min, config.current_time.tm_sec, current_time));

        pinMode(GPIO_NUM_14, OUTPUT);
        if (current_time >= config.eeprom.display_ontime && current_time < config.eeprom.display_offtime) {
            digitalWrite(GPIO_NUM_14, HIGH);
            // rtc_clk_slow_freq_set(RTC_SLOW_FREQ_8MD256);
        } else {
            digitalWrite(GPIO_NUM_14, LOW);
        }
        gpio_hold_dis(GPIO_NUM_14);
        gpio_hold_en(GPIO_NUM_14);
        gpio_deep_sleep_hold_en();
    } else {
        pinMode(GPIO_NUM_14, INPUT_PULLUP);
        gpio_hold_dis(GPIO_NUM_14);
        gpio_deep_sleep_hold_dis();
        // display_SetBrightness(&config.display, config.eeprom.display_brightness);

        // pinMode(GPIO_NUM_14, OUTPUT);
        // digitalWrite(GPIO_NUM_14, HIGH);
        // gpio_hold_en(GPIO_NUM_14);
        // gpio_deep_sleep_hold_en();
    }
    #endif
    #endif
}

void display_SetBrightness(Display *display, unsigned char brightness) {
    #if DISPLAY == ILI9488 || DISPLAY == ILI9341
    if (display->brightness != brightness) {
        D(Serial.printf("[DISPLAY]: Set brightness to %d.\n", brightness));
        // NOTE: Does not hold in deep sleep.
        const int PWM_FREQ = 5000; // freq limits depend on resolution
        const byte PWM_CHANNEL = 0; // channels 0-15
        const byte PWM_RES = 8; // resolution 1-16 bits
        const byte PWM_PIN = GPIO_NUM_14;
        // Configure LED PWM functionalitites.
        ledcSetup(PWM_CHANNEL, PWM_FREQ, PWM_RES);

        // Attach the channel to the GPIO to be controlled.
        ledcAttachPin(PWM_PIN, PWM_CHANNEL);

        // Changing the LED brightness with PWM.
        ledcWrite(PWM_CHANNEL, brightness);

        display->brightness = brightness;
    }
    #endif
}

#if DISPLAY == ILI9488
void printStatusMsg(Display *display, const char *msg) {
    D(Serial.println(msg));
    display->tft.fillScreen(TFT_BLACK);
    display->tft.setTextFont(1);

    display->tft.setCursor(38, 0);
    display->tft.setTextColor(TFT_WHITE);
    display->tft.setTextSize(2);

    display->tft.drawCentreString("Airthings Wave Plus", 160, 0, 1);
    display->tft.setTextSize(1);
    display->tft.drawCentreString(config.eeprom.custom_msg, 160, 20, 1);

    display->tft.setCursor(0, 60);
    display->tft.setTextColor(TFT_WHITE);
    display->tft.setTextSize(2);
    display->tft.println(msg);
}

void printSensorData(Display *display, const Eeprom *config, const SensorAirthingsWave *aw, MSystem msystem) {
    int r24_bqm3 = aw->r24*BECQUERELS_M2_IN_PICOCURIES_L;
    int rlife_bqm3 = aw->rlife*BECQUERELS_M2_IN_PICOCURIES_L;
    uint16_t r24_ringcolor = (r24_bqm3 < config->thld_r1) ? TFT_GREEN : (r24_bqm3 >= config->thld_r2) ? TFT_RED : TFT_YELLOW;
    uint16_t rlife_ringcolor = (rlife_bqm3 < config->thld_r1) ? TFT_GREEN : (rlife_bqm3 >= config->thld_r2) ? TFT_RED : TFT_YELLOW;
    uint16_t t_ringcolor = (aw->t < config->thld_t1) ? TFT_BLUE : (aw->t > config->thld_t2) ? TFT_RED : TFT_GREEN;
    uint16_t h_ringcolor = (aw->h < config->thld_h1 || aw->h >= config->thld_h4) ? TFT_RED : (aw->h >= config->thld_h2 && aw->h < config->thld_h3) ? TFT_GREEN : TFT_YELLOW;
    uint16_t co2_ringcolor = (aw->co2 < config->thld_co21) ? TFT_GREEN : (aw->co2 >= config->thld_co22) ? TFT_RED : TFT_YELLOW;
    uint16_t tvoc_ringcolor = (aw->tvoc < config->thld_tvoc1) ? TFT_GREEN : (aw->tvoc >= config->thld_tvoc2) ? TFT_RED : TFT_YELLOW;
    uint16_t a_ringcolor = (r24_ringcolor == TFT_RED || t_ringcolor == TFT_RED || t_ringcolor == TFT_BLUE || h_ringcolor == TFT_RED) ? TFT_RED : (r24_ringcolor == TFT_YELLOW || t_ringcolor == TFT_YELLOW || h_ringcolor == TFT_YELLOW) ? TFT_YELLOW : TFT_GREEN;
    uint16_t ring_color = (config->aw_status == radon) ? r24_ringcolor : (config->aw_status == temperature) ? t_ringcolor : (config->aw_status == humidity) ? h_ringcolor : (config->aw_status == co2) ? co2_ringcolor : (config->aw_status == tvoc) ? tvoc_ringcolor : a_ringcolor;

    char r24str[4];
    char rlifestr[4];
    char tstr[6];
    if (msystem == metric) {
        dtostrf(r24_bqm3, 3, 0, r24str);
        dtostrf(rlife_bqm3, 3, 0, rlifestr);
        dtostrf(aw->t, 4, 1, tstr);
    } else {
        dtostrf(aw->r24, 2, 1, r24str);
        dtostrf(aw->rlife, 2, 1, rlifestr);
        dtostrf(9.0/5.0 * aw->t + 32.0, 2, 0, tstr);
    }
    char delim[] = ".";
    char *tmajorstr = strtok(tstr, delim);
    char *tminorstr = strtok(NULL, delim);

    display->tft.fillScreen(TFT_BLACK);

    display->tft.setCursor(48, 0);
    display->tft.setTextColor(TFT_WHITE);
    display->tft.setTextSize(2);

    display->tft.drawCentreString("Airthings Wave Plus", 160, 0, 1);
    display->tft.setTextSize(1);
    if (config->custom_msg[0] == '!') {
        display->tft.setTextSize(2);
        display->tft.setTextColor(TFT_BLACK, TFT_YELLOW);
    }
    display->tft.drawCentreString(config->custom_msg, 160, 20, 1);

    display->tft.fillCircle(160, 115, 75, ring_color);
    display->tft.fillCircle(160, 115, 63, TFT_BLACK);

    display->tft.setTextColor(TFT_WHITE);
    display->tft.setTextSize(3);
    display->tft.setCursor(133, 90);
    display->tft.print(util_Trim(r24str));
    display->tft.setTextColor(TFT_WHITE);
    display->tft.setTextSize(2);
    display->tft.setCursor(133, 130);
    display->tft.print("Radon");
    display->tft.setCursor(133, 150);
    display->tft.print("(24h)");

    display->tft.setTextSize(2);
    display->tft.setTextColor(ring_color);
    display->tft.setCursor(38, 220);
    switch (ring_color) {
    case TFT_GREEN:
        if (r24_bqm3 != 0) {
            display->tft.drawCentreString(i10n_GetText(STR_AIR_QUALITY_IS_GOOD, config->lang), 160, 220, 1);
        } else {
            display->tft.drawCentreString(i10n_GetText(STR_SENSORS_CALIBRATING, config->lang), 160, 220, 1);
        }
    break;
    case TFT_YELLOW:
        display->tft.drawCentreString(i10n_GetText(STR_AIR_QUALITY_IS_OK, config->lang), 160, 220, 1);
    break;
    case TFT_RED:
    case TFT_BLUE:
        display->tft.drawCentreString(i10n_GetText(STR_AIR_QUALITY_IS_POOR, config->lang), 160, 220, 1);
    break;
    }

    display->tft.setTextColor(TFT_WHITE);
    display->tft.setTextSize(3);
    display->tft.setCursor(27, 250);
    display->tft.print(util_Trim(rlifestr));
    display->tft.setCursor(135, 250);
    display->tft.print(String((int)(aw->tvoc)));
    display->tft.setCursor(240, 250);
    display->tft.print(String((int)aw->co2));

    display->tft.setTextColor(TFT_WHITE);
    display->tft.setTextSize(2);
    display->tft.setCursor(24, 290);
    display->tft.print("Radon");
    display->tft.setCursor(20, 310);
    display->tft.print(i10n_GetText(STR_PLIFEP, config->lang));
    if (rlife_ringcolor != TFT_GREEN) display->tft.fillCircle(52, 280, 4, rlife_ringcolor);
    display->tft.setCursor(135, 290);
    display->tft.print("TVOC");
    if (tvoc_ringcolor != TFT_GREEN) display->tft.fillCircle(156, 280, 4, tvoc_ringcolor);
    display->tft.setCursor(135, 310);
    display->tft.print("(ppb)");
    display->tft.setCursor(250, 290);
    display->tft.print("CO2");
    if (co2_ringcolor != TFT_GREEN) display->tft.fillCircle(265, 280, 4, co2_ringcolor);
    display->tft.setCursor(240, 310);
    display->tft.print("(ppm)");

    display->tft.setTextColor(TFT_WHITE);
    display->tft.setTextSize(3);
    display->tft.setCursor(27, 360);
    display->tft.print(String((int)aw->h) + "%");
    display->tft.setCursor(135, 360);
    display->tft.print(tmajorstr);
    if (msystem == metric) {
        display->tft.setCursor(168, 375);
        display->tft.setTextSize(1);
        display->tft.print(String(".") + tminorstr);
    }

    display->tft.setTextSize(3);
    display->tft.setCursor(230, 360);
    display->tft.print(String((int)aw->p));

    display->tft.setTextColor(TFT_WHITE);
    display->tft.setTextSize(2);
    display->tft.setCursor(5, 400);
    display->tft.print(i10n_GetText(STR_HUMIDITY, config->lang));
    if (h_ringcolor != TFT_GREEN) display->tft.fillCircle(52, 390, 4, h_ringcolor);
    display->tft.setCursor(135, 400);
    display->tft.print("Temp");
    if (t_ringcolor != TFT_GREEN) display->tft.fillCircle(156, 390, 4, t_ringcolor);
    display->tft.setCursor(220, 400);
    display->tft.print(i10n_GetText(STR_PRESSURE, config->lang));
}
#endif

#if DISPLAY == ILI9341
void printStatusMsg(Display *display, const char *msg) {
    D(Serial.println(msg));
    display->tft.fillScreen(TFT_BLACK);
    display->tft.setTextFont(1);

    display->tft.setCursor(38, 0);
    display->tft.setTextColor(TFT_WHITE);
    display->tft.setTextSize(2);

    display->tft.drawCentreString("Airthings Wave", 120, 0, 1);
    display->tft.setTextSize(1);
    display->tft.drawCentreString(config.eeprom.custom_msg, 120, 20, 1);

    display->tft.setCursor(0, 60);
    display->tft.setTextColor(TFT_WHITE);
    display->tft.setTextSize(2);
    display->tft.println(msg);
}

void printSensorData(Display *display, const Eeprom *config, const SensorAirthingsWave *aw, MSystem msystem) {
    int r24_bqm3 = aw->r24*BECQUERELS_M2_IN_PICOCURIES_L;
    int rlife_bqm3 = aw->rlife*BECQUERELS_M2_IN_PICOCURIES_L;
    uint16_t r24_ringcolor = (r24_bqm3 < config->thld_r1) ? TFT_GREEN : (r24_bqm3 >= config->thld_r2) ? TFT_RED : TFT_YELLOW;
    uint16_t rlife_ringcolor = (rlife_bqm3 < config->thld_r1) ? TFT_GREEN : (rlife_bqm3 >= config->thld_r2) ? TFT_RED : TFT_YELLOW;
    uint16_t t_ringcolor = (aw->t < config->thld_t1) ? TFT_BLUE : (aw->t > config->thld_t2) ? TFT_RED : TFT_GREEN;
    uint16_t h_ringcolor = (aw->h < config->thld_h1 || aw->h >= config->thld_h4) ? TFT_RED : (aw->h >= config->thld_h2 && aw->h < config->thld_h3) ? TFT_GREEN : TFT_YELLOW;
    uint16_t a_ringcolor = (r24_ringcolor == TFT_RED || t_ringcolor == TFT_RED || t_ringcolor == TFT_BLUE || h_ringcolor == TFT_RED) ? TFT_RED : (r24_ringcolor == TFT_YELLOW || t_ringcolor == TFT_YELLOW || h_ringcolor == TFT_YELLOW) ? TFT_YELLOW : TFT_GREEN;
    uint16_t ring_color = (config->aw_status == radon) ? r24_ringcolor : (config->aw_status == temperature) ? t_ringcolor : (config->aw_status == humidity) ? h_ringcolor : a_ringcolor;

    char r24str[4];
    char rlifestr[4];
    char tstr[6];
    if (msystem == metric) {
        dtostrf(r24_bqm3, 3, 0, r24str);
        dtostrf(rlife_bqm3, 3, 0, rlifestr);
        dtostrf(aw->t, 4, 1, tstr);
    } else {
        dtostrf(aw->r24, 2, 1, r24str);
        dtostrf(aw->rlife, 2, 1, rlifestr);
        dtostrf(9.0/5.0 * aw->t + 32.0, 2, 0, tstr);
    }
    char delim[] = ".";
    char *tmajorstr = strtok(tstr, delim);
    char *tminorstr = strtok(NULL, delim);

    display->tft.fillScreen(TFT_BLACK);

    display->tft.setCursor(38, 0);
    display->tft.setTextColor(TFT_WHITE);
    display->tft.setTextSize(2);

    display->tft.drawCentreString("Airthings Wave", 120, 0, 1);
    display->tft.setTextSize(1);
    if (config->custom_msg[0] == '!') {
        display->tft.setTextSize(2);
        display->tft.setTextColor(TFT_BLACK, TFT_YELLOW);
    }
    display->tft.drawCentreString(config->custom_msg, 120, 20, 1);

    display->tft.fillCircle(120, 115, 75, ring_color);
    display->tft.fillCircle(120, 115, 63, TFT_BLACK);

    display->tft.setTextColor(TFT_WHITE);
    display->tft.setTextSize(3);
    display->tft.setCursor(95, 90);
    display->tft.print(util_Trim(r24str));
    display->tft.setTextColor(TFT_WHITE);
    display->tft.setTextSize(2);
    display->tft.setCursor(93, 130);
    display->tft.print("Radon");
    display->tft.setCursor(93, 150);
    display->tft.print("(24h)");

    display->tft.setTextSize(2);
    display->tft.setTextColor(ring_color);
    display->tft.setCursor(0, 210);
    switch (ring_color) {
    case TFT_GREEN:
        if (r24_bqm3 != 0) {
            display->tft.drawCentreString(i10n_GetText(STR_AIR_QUALITY_IS_GOOD, config->lang), 120, 210, 1);
        } else {
            display->tft.drawCentreString(i10n_GetText(STR_SENSORS_CALIBRATING, config->lang), 120, 210, 1);
        }
    break;
    case TFT_YELLOW:
        display->tft.drawCentreString(i10n_GetText(STR_AIR_QUALITY_IS_OK, config->lang), 120, 210, 1);
    break;
    case TFT_RED:
    case TFT_BLUE:
        display->tft.drawCentreString(i10n_GetText(STR_AIR_QUALITY_IS_POOR, config->lang), 120, 210, 1);
    break;
    }

    display->tft.setTextColor(TFT_WHITE);
    display->tft.setTextSize(3);
    display->tft.setCursor(7, 240);
    display->tft.print(util_Trim(rlifestr));
    display->tft.setCursor(101, 240);
    display->tft.print(String((int)aw->h) + "%");
    display->tft.setCursor(194, 240);
    display->tft.print(tmajorstr);
    if (msystem == metric) {
        display->tft.setCursor(227, 255);
        display->tft.setTextSize(1);
        display->tft.print(String(".") + tminorstr);
    }

    display->tft.setTextColor(TFT_WHITE);
    display->tft.setTextSize(2);
    display->tft.setCursor(4, 280);
    display->tft.print("Radon");
    display->tft.setCursor(0, 300);
    display->tft.print(i10n_GetText(STR_PLIFEP, config->lang));
    if (rlife_ringcolor != TFT_GREEN) display->tft.fillCircle(32, 271, 4, rlife_ringcolor);
    display->tft.setCursor(79, 280);
    display->tft.print(i10n_GetText(STR_HUMIDITY, config->lang));
    if (h_ringcolor != TFT_GREEN) display->tft.fillCircle(122, 271, 4, h_ringcolor);
    display->tft.setCursor(189, 280);
    display->tft.print("Temp");
    if (t_ringcolor != TFT_GREEN) display->tft.fillCircle(211, 271, 4, t_ringcolor);
}
#endif

#if DISPLAY == SSD1306 || DISPLAY == SH1106

void printStatusMsg(const char *msg) {
    D(Serial.println(msg));
    display.clearDisplay();

    display.drawRect(0, 0, SCREEN_WIDTH, SCREEN_HEIGHT, 0);

    display.setCursor(20, 1);
    display.print("Airthings Wave");

    display.setCursor(5, 30);
    display.print(msg);

    display.display();
}

void printSensorData(const Eeprom *config, const SensorAirthingsWave *aw, MSystem msystem) {
    display.clearDisplay();

    display.drawRect(0, 0, SCREEN_WIDTH, SCREEN_HEIGHT, 0);

    display.setCursor(20, 1);
    display.print("Airthings Wave");

    display.setCursor(5, 17);
    display.print("Radon(24h):" + String((int)(aw->r24*BECQUERELS_M2_IN_PICOCURIES_L)) + " bqm3");
    display.setCursor(5, 30);
    display.print("Radon(life):" + String((int)(aw->rlife*BECQUERELS_M2_IN_PICOCURIES_L)) + " bqm3");
    display.setCursor(5, 43);
    display.print("Temperature:" + String((int)aw->t) + "" + (char)247 + "C");
    display.setCursor(5, 56);
    display.print("Humidity:" + String((int)aw->h) + "%");

    display.display();
}

#endif

// -----------------------------------------------------------------------------
// Wifi functions.
// -----------------------------------------------------------------------------

void tick(void) {
    int state = digitalRead(BLUETOOTH_LED);
    digitalWrite(BLUETOOTH_LED, !state);
}

bool wifi_Init(WiFiClientSecure *ssl_client) {
    // Configure secure client connection.
    wifi_SetSecure(ssl_client, digicert);

    D(Serial.println(F("[WIFI] client initialized.")));

    return true;
}

bool wifi_Connect(WiFiClientSecure *ssl_client, bool show_connect_msg) {
    if (wifi_IsWifi() == false) {
        Ticker ticker;
        // WifiManager settings.
        WiFiManagerParameter custom_user_email("Email", "Email address (Sybraset account)", config.eeprom.user_email, sizeof(config.eeprom.user_email));

        // Blink led every .5 secs while in wifimanager setup mode.
        ticker.attach(0.5, tick);
        if (show_connect_msg == true) {
            printStatusMsg(&config.display, i10n_GetText(STR_WIFI_CONNECT, config.eeprom.lang).c_str());
        }

        WiFiManager wifi_manager;
        #if WIFIMANAGER_RESET == true
        wifi_manager.resetSettings();
        #endif
        #if DEBUG == false
        wifi_manager.setDebugOutput(false); // Uncomment for production.
        #endif

        wifi_manager.addParameter(&custom_user_email);
        wifi_manager.setSaveConfigCallback(wifi_SaveConfigCallback);

        // Print chip mac address (debug).
        D(Serial.print(F("\n[WIFI] MAC: ")));
        D(Serial.println(WiFi.macAddress()));
        D(Serial.printf("[ESP32] chip id: %08X\n", String((unsigned int)ESP.getEfuseMac(), HEX)));

        // Connect to wifi.
        wifi_manager.setTimeout(10*60);
        String hostname = String(MODEL) + "-" + String((unsigned int)ESP.getEfuseMac(), HEX);
        if (!wifi_manager.autoConnect((const char *)hostname.c_str(), wifi_GetMAC().c_str())) {
            printStatusMsg(&config.display, "Error: connecting to Wi-Fi network failed. Please try again.");
            delay(10000);
            WiFi.disconnect(true);
            ESP.restart();
            delay(10000);
        }
        util_SetClockWithNTP(&config.current_time);

        // WifiManager setup completed.
        ticker.detach();
        digitalWrite(BLUETOOTH_LED, LOW);

        if (config.wifi_save_config == true) {
            strcpy(config.eeprom.user_email, custom_user_email.getValue());
            eeprom_StoreConfig(&config.eeprom);
        }

        #if DEBUG == true
        Serial.println(F("\n[WIFI] connected."));
        // Print wifi signal strength (debug).
        Serial.print("[WIFI] Signal Strength (RSSI): ");
        Serial.print(wifi_GetQuality());
        Serial.println("%");
        // Print IP (debug).
        Serial.print(F("[WIFI] Got IP: "));
        Serial.println(WiFi.localIP());
        #endif
    } else {
        util_GetCurrentTime(&config.current_time);
    }

    return wifi_IsWifi();
}

void wifi_Disconnect(WiFiClientSecure *ssl_client) {
    ssl_client->stop();
    WiFi.mode(WIFI_OFF);
    WiFi.disconnect();
    delay(100);
}

// WifiManager callback notifying us of the need to save config.
void wifi_SaveConfigCallback(void) {
    D(Serial.println(F("[WIFI] wifi_SaveConfigCallback().")));
    config.wifi_save_config = true;
}

// Is wifi connected?
bool wifi_IsWifi(void) {
    return (WiFi.status() == WL_CONNECTED);
}

void wifi_SetSecure(WiFiClientSecure *ssl_client, const char *cert) {
    ssl_client->setCACert(cert);
}

// Get MAC address of device.
String wifi_GetMAC(void) {
    uint8_t mac[6];
    char result[14];

    WiFi.macAddress(mac);
    snprintf(result, sizeof(result), "%02x%02x%02x%02x%02x%02x", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

    return String(result);
}

// Converts the dBm to a range between 0 and 100%.
#if DEBUG == true
int8_t wifi_GetQuality(void) {
    int32_t dbm = WiFi.RSSI();
    if (dbm <= -100) {
        return 0;
    } else if (dbm >= -50) {
        return 100;
    } else {
        return 2 * (dbm + 100);
    }
}
#endif

// -----------------------------------------------------------------------------
// EEPROM functions.
// -----------------------------------------------------------------------------

// Variables associated with EEPROM are cached in RAM for use during execution so EEPROM does not need to be read excessively.

// NOTE: When adding new values in the EEPROM, don't copy config value to default_config value. Otherwise copies null value.
// OR is it better to update values after the update?
void eeprom_Init(Eeprom *default_config, Eeprom *config) {
    EEPROM.begin(512);
    // Init default config.
    eeprom_InitDefaultConfig(default_config);

    #if EEPROM_INIT == true
    // Init eeprom with default values.
    eeprom_Reset();
    eeprom_StoreConfig(default_config);
    #endif

    eeprom_GetConfig(config);
    if (config->fw_updated == true) {
        // Init eeprom with default values.
        eeprom_Reset();
        strcpy(default_config->mqtt_ci, config->mqtt_ci);
        strcpy(default_config->aw_sn, config->aw_sn);
        strcpy(default_config->user_email, config->user_email);
        strcpy(default_config->custom_msg, config->custom_msg);
        default_config->t_offset = config->t_offset;
        default_config->h_offset = config->h_offset;
        default_config->time_to_sleep = config->time_to_sleep;
        default_config->lang = config->lang;
        default_config->msystem = config->msystem;
        // Sensor thresholds.
        default_config->aw_status = config->aw_status;
        default_config->thld_t1 = config->thld_t1;
        default_config->thld_t2 = config->thld_t2;
        default_config->thld_h1 = config->thld_h1;
        default_config->thld_h2 = config->thld_h2;
        default_config->thld_h3 = config->thld_h3;
        default_config->thld_h4 = config->thld_h4;
        default_config->thld_r1 = config->thld_r1;
        default_config->thld_r2 = config->thld_r2;
        default_config->thld_co21 = config->thld_co21;
        default_config->thld_co22 = config->thld_co22;
        default_config->thld_tvoc1 = config->thld_tvoc1;
        default_config->thld_tvoc2 = config->thld_tvoc2;
        // Display.
        default_config->display_rotation = config->display_rotation;
        default_config->display_brightness = config->display_brightness;
        default_config->display_brightness_ontime = config->display_brightness_ontime;
        default_config->display_brightness_offtime = config->display_brightness_offtime;
        default_config->display_ontime = config->display_ontime;
        default_config->display_offtime = config->display_offtime;
        eeprom_StoreConfig(default_config);

        D(eeprom_PrintConfig(config));
        ESP.restart();
        delay(1000);
    }

    #if DEBUG == true
    Serial.println("");
    eeprom_PrintConfig(config);
    eeprom_Print();
    #endif
}

// Has the device been initialized and has a authtoken?
bool eeprom_IsDeviceInitialized(Eeprom *config) {
    return (strlen(config->mqtt_ci) == 32);
}

void eeprom_InitDefaultConfig(Eeprom *default_config) {
    default_config->fw_updated = false;
    strcpy(default_config->mqtt_ci, (wifi_GetMAC() + "/mac").c_str());
    strcpy(default_config->aw_sn, "");
    strcpy(default_config->user_email, "");
    strcpy(default_config->custom_msg, "");
    default_config->t_offset = 0;
    default_config->h_offset = 0;
    default_config->time_to_sleep = 60*60; // 1 hour
    default_config->lang = en;
    default_config->msystem = metric;
    // Sensor thresholds.
    default_config->aw_status = radon;
    default_config->thld_t1 = 18;
    default_config->thld_t2 = 25;
    default_config->thld_h1 = 25;
    default_config->thld_h2 = 30;
    default_config->thld_h3 = 60;
    default_config->thld_h4 = 70;
    default_config->thld_r1 = 100;
    default_config->thld_r2 = 150;
    default_config->thld_co21 = 800;
    default_config->thld_co22 = 1000;
    default_config->thld_tvoc1 = 250;
    default_config->thld_tvoc2 = 2000;
    // Display.
    default_config->display_rotation = 0;
    default_config->display_brightness = 100;
    default_config->display_brightness_ontime = 100;
    default_config->display_brightness_offtime = 30;
    default_config->display_ontime = 0;
    default_config->display_offtime = 0;
}

// Get config variables from eeprom.
void eeprom_GetConfig(Eeprom *config) {
    EEPROM.get(EEPROM_CONFIG, *config);
}

// Store config variables in eeprom.
void eeprom_StoreConfig(Eeprom *config) {
    EEPROM.put(EEPROM_CONFIG, *config);
    EEPROM.commit();
}

// Reset eeprom to all 0's.
void eeprom_Reset(void) {
    struct {
        char temp[512] = "";
    } data;

    EEPROM.put(0, data);

    EEPROM.commit();
}

// Prints a hex map of eeprom values (debug).
#if DEBUG == true
void eeprom_Print(void) {
    int i, j, v;
    for (j=0; j < 32; j++) {
        Serial.print(j*16, HEX);
        Serial.print(F(" "));
        for (i=0; i < 16; i++) {
            v = EEPROM.read(i+(16*j));
            Serial.print(v, HEX);
            Serial.print(F(" "));
        }
        Serial.println(F(""));
    }
}
#endif

// Prints config values (debug).
#if DEBUG == true
void eeprom_PrintConfig(Eeprom *config) {
    Serial.println("Config EEPROM: " +
        String(config->fw_updated) + "," +
        String(config->mqtt_ci) + "," +
        String(config->aw_sn) + "," +
        String(config->user_email) + "," +
        String(config->custom_msg) + "," +
        String(config->t_offset) + "," +
        String(config->h_offset) + "," +
        String(config->time_to_sleep) + "," +
        String(config->lang) + "," +
        String(config->msystem) + "," +
        // Sensor thresholds.
        String(config->aw_status) + "," +
        String(config->thld_t1) + "," +
        String(config->thld_t2) + "," +
        String(config->thld_h1) + "," +
        String(config->thld_h2) + "," +
        String(config->thld_h3) + "," +
        String(config->thld_h4) + "," +
        String(config->thld_r1) + "," +
        String(config->thld_r2) + "," +
        String(config->thld_co21) + "," +
        String(config->thld_co22) + "," +
        String(config->thld_tvoc1) + "," +
        String(config->thld_tvoc2) + "," +
        // Display.
        String(config->display_rotation) + "," +
        String(config->display_brightness) + "," +
        String(config->display_brightness_ontime) + "," +
        String(config->display_brightness_offtime) + "," +
        String(config->display_ontime) + "," +
        String(config->display_offtime)
    );
}
#endif

// -----------------------------------------------------------------------------
// MQTT functions.
// -----------------------------------------------------------------------------

// NOTE: The maximum message size, including header, is 128 bytes by default. This is configurable via MQTT_MAX_PACKET_SIZE in PubSubClient.h. Longer messages can also be sent with the publish_P() method.

// Initialize MQTT.
PubSubClient *mqtt_Init(WiFiClientSecure *ssl_client) {
    PubSubClient *client = new PubSubClient(*ssl_client);
    // Start mqtt server & set callback.
    // IPAddress addr(192,168,0,153);
    // client->setServer(addr, config.mqtt_port);
    client->setServer((char *)config.mqtt_svr.c_str(), config.mqtt_port);
    client->setCallback(mqtt_Callback);
    D(Serial.print(F("[MQTT] Rx Topic: ")));
    delay(10);
    D(Serial.println((char *)("[MQTT] " + String(config.eeprom.mqtt_ci) + config.mqtt_rt).c_str()));
    delay(10);
    D(Serial.print(F("[MQTT] Tx Topic: ")));
    delay(10);
    D(Serial.println((char *)("[MQTT] " + String(config.eeprom.mqtt_ci) + config.mqtt_tt).c_str()));
    delay(10);
    D(Serial.println(F("[MQTT] client initialized.")));
    delay(10);

    return client;
}

// Connect to MQTT broker - then subscribe to server topic.
void mqtt_Connect(PubSubClient *client) {
    int timeout = 10;
    bool connected = false;
    // Loop until we're reconnected (give up after 10 tries).
    while (!client->connected() && (timeout-- != 0)) {
        D(Serial.print(F("[MQTT] Attempting connection... ")));
        // Connect to MQTT Server.
        if (eeprom_IsDeviceInitialized(&config.eeprom) == true) {
            D(Serial.print(F("(device initialized)")));
            connected = client->connect(config.eeprom.mqtt_ci, config.mqtt_un.c_str(), config.mqtt_pw.c_str(), config.eeprom.mqtt_ci, 1, 0, "will_msg", false);
        } else {
            D(Serial.print(F("(device NOT initialized)")));
            // connected = client->connect(config.eeprom.mqtt_ci, config.mqtt_un.c_str(), config.mqtt_pw.c_str());
            connected = client->connect(config.eeprom.mqtt_ci, config.mqtt_un.c_str(), config.mqtt_pw.c_str(), config.eeprom.mqtt_ci, 1, 0, "will_msg", true);
        }
        if (connected == true) {
            D(Serial.println(F("\n[MQTT] connected")));
            client->subscribe((char *)(config.eeprom.mqtt_ci + config.mqtt_rt).c_str(), 1);
       } else {
            D(Serial.print(F("[MQTT] failed, rc=")));
            D(Serial.print(client->state()));
            D(Serial.println(F(" try again in 5 seconds")));
            delay(5000); // Wait 5 seconds before retrying.
        }
    }
}

 // MQTT loop() service.
void mqtt_Processor(PubSubClient *client) {
    if (wifi_IsWifi() == true) {
        yield();
        if (!client->connected()) {
            mqtt_Connect(client);
        }
        client->loop();
        yield();
    } else {
        D(Serial.println(F("[MQTT] processor: no wifi, cannot run.")));
    }
}

// MQTT server topic callback.
// Note: Do no restart/reset device from callnack. Otherwise mqtt msg won't be removed from queue and will repeat.
void mqtt_Callback(char *topic, byte *payload, unsigned int length) {
    String str_payload = "";
    // Extract payload.
    for (int i=0; i < length; i++) {
        str_payload = str_payload + String((char)payload[i]);
    }
    // Msg rx message.
    D(Serial.println(F("[MQTT] -------------------------------------")));
    D(Serial.print(F("[MQTT] Rx: topic[")));
    D(Serial.print("[MQTT] " + String(topic)));
    D(Serial.print("] \r\n[MQTT] Rx payload:"));
    D(Serial.println(str_payload));
    D(Serial.println(F("[MQTT] -------------------------------------")));
    // Ignore if not server request.
    if (String(topic) != String(config.eeprom.mqtt_ci) + config.mqtt_rt) {
        return;
    }

    // Parse payload for commands.
    size_t max_params = 12;
    String params[max_params];
    util_SplitString(&str_payload, params, max_params, ",");

    yield();
    bool eeprom_storeconfig = false;
    for (int i=0; params[i] != NULL; i++) {
        // Command.
        size_t cmd_max_params = 2;
        String cmd_params[cmd_max_params];
        util_SplitString(&params[i], cmd_params, cmd_max_params, ":");

        if (cmd_params[0] == "custom_msg") {
            if (cmd_params[1] != NULL) {
                D(Serial.println("custom_msg: " + cmd_params[1]));
                if (cmd_params[1].toInt() == -1) { // Reset to default setting.
                    strcpy(config.eeprom.custom_msg, "");
                } else {
                    strncpy(config.eeprom.custom_msg, cmd_params[1].c_str(), 21);
                }
                eeprom_storeconfig = true;
                config.refresh_display = true;
            }
        } else if (cmd_params[0] == "aw_sn") {
            if (cmd_params[1] != NULL) {
                D(Serial.println("aw_sn: " + cmd_params[1]));
                if (cmd_params[1].toInt() == -1) { // Reset to default setting.
                    strcpy(config.eeprom.aw_sn, "");
                } else {
                    strncpy(config.eeprom.aw_sn, cmd_params[1].c_str(), 10);
                }
                eeprom_storeconfig = true;
                config.restart_device = true;
            }
        } else if (cmd_params[0] == "t_offset") {
            if (cmd_params[1] != NULL) {
                D(Serial.println("t_offset: " + cmd_params[1]));
                if (config.eeprom.t_offset != cmd_params[1].toInt() && cmd_params[1].toInt() >= -20 && cmd_params[1].toInt() <= 20) {
                    config.eeprom.t_offset = cmd_params[1].toInt();
                    eeprom_storeconfig = true;
                }
            }
        } else if (cmd_params[0] == "h_offset") {
            if (cmd_params[1] != NULL) {
                D(Serial.println("h_offset: " + cmd_params[1]));
                if (config.eeprom.h_offset != cmd_params[1].toInt() && cmd_params[1].toInt() >= -20 && cmd_params[1].toInt() <= 20) {
                    config.eeprom.h_offset = cmd_params[1].toInt();
                    eeprom_storeconfig = true;
                }
            }
        } else if (cmd_params[0] == "time_to_sleep") {
            if (cmd_params[1] != NULL) {
                D(Serial.println("time_to_sleep: " + cmd_params[1]));
                if (config.eeprom.time_to_sleep != cmd_params[1].toInt() && cmd_params[1].toInt() > 0 && cmd_params[1].toInt() <= 60) {
                    config.eeprom.time_to_sleep = cmd_params[1].toInt() * 60*60; // Convert to hours. 1 hour minimum.
                    eeprom_storeconfig = true;
                }
            }
        } else if (cmd_params[0] == "lang") {
            if (cmd_params[1] != NULL) {
                D(Serial.println("lang: " + cmd_params[1]));
                if (config.eeprom.lang != (Localization)cmd_params[1].toInt() &&
                    ((Localization)cmd_params[1].toInt() == en || (Localization)cmd_params[1].toInt() == fr)) {
                    config.eeprom.lang = (Localization)cmd_params[1].toInt();
                    eeprom_storeconfig = true;
                    config.refresh_display = true;
                }
            }
        } else if (cmd_params[0] == "msystem") {
            if (cmd_params[1] != NULL) {
                D(Serial.println("msystem: " + cmd_params[1]));
                if (config.eeprom.msystem != (MSystem)cmd_params[1].toInt() &&
                    ((MSystem)cmd_params[1].toInt() == metric || (MSystem)cmd_params[1].toInt() == imperial)) {
                    config.eeprom.msystem = (MSystem)cmd_params[1].toInt();
                    eeprom_storeconfig = true;
                    config.refresh_display = true;
                }
            }
        } else if (cmd_params[0] == "aw_status") {
            if (cmd_params[1] != NULL) {
                D(Serial.println("aw_status: " + cmd_params[1]));
                if (config.eeprom.aw_status != (AWStatus)cmd_params[1].toInt() &&
                    ((AWStatus)cmd_params[1].toInt() == all || (AWStatus)cmd_params[1].toInt() == temperature ||
                    (AWStatus)cmd_params[1].toInt() == humidity || (AWStatus)cmd_params[1].toInt() == radon ||
                    (AWStatus)cmd_params[1].toInt() == co2 || (AWStatus)cmd_params[1].toInt() == tvoc)) {
                    config.eeprom.aw_status = (AWStatus)cmd_params[1].toInt();
                    eeprom_storeconfig = true;
                    config.refresh_display = true;
                }
            }
        } else if (cmd_params[0] == "thld_t1") {
            if (cmd_params[1] != NULL) {
                D(Serial.println("thld_t1: " + cmd_params[1]));
                if (config.eeprom.thld_t1 != cmd_params[1].toInt() && cmd_params[1].toInt() >= -500 && cmd_params[1].toInt() <= 500) {
                    config.eeprom.thld_t1 = cmd_params[1].toInt();
                    eeprom_storeconfig = true;
                    config.refresh_display = true;
                }
            }
        } else if (cmd_params[0] == "thld_t2") {
            if (cmd_params[1] != NULL) {
                D(Serial.println("thld_t2: " + cmd_params[1]));
                if (config.eeprom.thld_t2 != cmd_params[1].toInt() && cmd_params[1].toInt() >= -500 && cmd_params[1].toInt() <= 500) {
                    config.eeprom.thld_t2 = cmd_params[1].toInt();
                    eeprom_storeconfig = true;
                    config.refresh_display = true;
                }
            }
        } else if (cmd_params[0] == "thld_h1") {
            if (cmd_params[1] != NULL) {
                D(Serial.println("thld_h1: " + cmd_params[1]));
                if (config.eeprom.thld_h1 != cmd_params[1].toInt() && cmd_params[1].toInt() >= 0 && cmd_params[1].toInt() <= 100) {
                    config.eeprom.thld_h1 = cmd_params[1].toInt();
                    eeprom_storeconfig = true;
                    config.refresh_display = true;
                }
            }
        } else if (cmd_params[0] == "thld_h2") {
            if (cmd_params[1] != NULL) {
                D(Serial.println("thld_h2: " + cmd_params[1]));
                if (config.eeprom.thld_h2 != cmd_params[1].toInt() && cmd_params[1].toInt() >= 0 && cmd_params[1].toInt() <= 100) {
                    config.eeprom.thld_h2 = cmd_params[1].toInt();
                    eeprom_storeconfig = true;
                    config.refresh_display = true;
                }
            }
        } else if (cmd_params[0] == "thld_h3") {
            if (cmd_params[1] != NULL) {
                D(Serial.println("thld_h3: " + cmd_params[1]));
                if (config.eeprom.thld_h3 != cmd_params[1].toInt() && cmd_params[1].toInt() >= 0 && cmd_params[1].toInt() <= 100) {
                    config.eeprom.thld_h3 = cmd_params[1].toInt();
                    eeprom_storeconfig = true;
                    config.refresh_display = true;
                }
            }
        } else if (cmd_params[0] == "thld_h4") {
            if (cmd_params[1] != NULL) {
                D(Serial.println("thld_h4: " + cmd_params[1]));
                if (config.eeprom.thld_h4 != cmd_params[1].toInt() && cmd_params[1].toInt() >= 0 && cmd_params[1].toInt() <= 100) {
                    config.eeprom.thld_h4 = cmd_params[1].toInt();
                    eeprom_storeconfig = true;
                    config.refresh_display = true;
                }
            }
        } else if (cmd_params[0] == "thld_r1") {
            if (cmd_params[1] != NULL) {
                D(Serial.println("thld_r1: " + cmd_params[1]));
                if (config.eeprom.thld_r1 != cmd_params[1].toInt() && cmd_params[1].toInt() >= 0 && cmd_params[1].toInt() <= 10000) {
                    config.eeprom.thld_r1 = cmd_params[1].toInt();
                    eeprom_storeconfig = true;
                    config.refresh_display = true;
                }
            }
        } else if (cmd_params[0] == "thld_r2") {
            if (cmd_params[1] != NULL) {
                D(Serial.println("thld_r2: " + cmd_params[1]));
                if (config.eeprom.thld_r2 != cmd_params[1].toInt() && cmd_params[1].toInt() >= 0 && cmd_params[1].toInt() <= 10000) {
                    config.eeprom.thld_r2 = cmd_params[1].toInt();
                    eeprom_storeconfig = true;
                    config.refresh_display = true;
                }
            }
        } else if (cmd_params[0] == "thld_co21") {
            if (cmd_params[1] != NULL) {
                D(Serial.println("thld_co21: " + cmd_params[1]));
                if (config.eeprom.thld_co21 != cmd_params[1].toInt() && cmd_params[1].toInt() >= 0 && cmd_params[1].toInt() <= 10000) {
                    config.eeprom.thld_co21 = cmd_params[1].toInt();
                    eeprom_storeconfig = true;
                    config.refresh_display = true;
                }
            }
        } else if (cmd_params[0] == "thld_co22") {
            if (cmd_params[1] != NULL) {
                D(Serial.println("thld_co22: " + cmd_params[1]));
                if (config.eeprom.thld_co22 != cmd_params[1].toInt() && cmd_params[1].toInt() >= 0 && cmd_params[1].toInt() <= 10000) {
                    config.eeprom.thld_co22 = cmd_params[1].toInt();
                    eeprom_storeconfig = true;
                    config.refresh_display = true;
                }
            }
        } else if (cmd_params[0] == "thld_tvoc1") {
            if (cmd_params[1] != NULL) {
                D(Serial.println("thld_tvoc1: " + cmd_params[1]));
                if (config.eeprom.thld_tvoc1 != cmd_params[1].toInt() && cmd_params[1].toInt() >= 0 && cmd_params[1].toInt() <= 10000) {
                    config.eeprom.thld_tvoc1 = cmd_params[1].toInt();
                    eeprom_storeconfig = true;
                    config.refresh_display = true;
                }
            }
        } else if (cmd_params[0] == "thld_tvoc2") {
            if (cmd_params[1] != NULL) {
                D(Serial.println("thld_tvoc2: " + cmd_params[1]));
                if (config.eeprom.thld_tvoc2 != cmd_params[1].toInt() && cmd_params[1].toInt() >= 0 && cmd_params[1].toInt() <= 10000) {
                    config.eeprom.thld_tvoc2 = cmd_params[1].toInt();
                    eeprom_storeconfig = true;
                    config.refresh_display = true;
                }
            }
        } else if (cmd_params[0] == "d_rotation") {
            if (cmd_params[1] != NULL) {
                D(Serial.println("display_rotation: " + cmd_params[1]));
                if (config.eeprom.display_rotation != cmd_params[1].toInt() && cmd_params[1].toInt() >= 0 && cmd_params[1].toInt() <= 3) {
                    config.eeprom.display_rotation = cmd_params[1].toInt();
                    eeprom_storeconfig = true;
                    #if DISPLAY == ILI9488 || DISPLAY == ILI9341
                    config.display.tft.setRotation(config.eeprom.display_rotation);
                    #endif
                    config.refresh_display = true;
                }
            }
        } else if (cmd_params[0] == "d_brightness") {
            if (cmd_params[1] != NULL) {
                D(Serial.println("display_brightness: " + cmd_params[1]));
                if (config.eeprom.display_brightness != cmd_params[1].toInt() && cmd_params[1].toInt() >= 0 && cmd_params[1].toInt() <= 255) {
                    config.eeprom.display_brightness = cmd_params[1].toInt();
                    eeprom_storeconfig = true;
                    display_SetBrightness(&config.display, config.eeprom.display_brightness);
                }
            }
        } else if (cmd_params[0] == "d_brightness_ontime") {
            if (cmd_params[1] != NULL) {
                D(Serial.println("display_brightness_ontime: " + cmd_params[1]));
                if (config.eeprom.display_brightness_ontime != cmd_params[1].toInt() && cmd_params[1].toInt() >= 0 && cmd_params[1].toInt() <= 255) {
                    config.eeprom.display_brightness_ontime = cmd_params[1].toInt();
                    eeprom_storeconfig = true;
                }
            }
        } else if (cmd_params[0] == "d_brightness_offtime") {
            if (cmd_params[1] != NULL) {
                D(Serial.println("display_brightness_offtime: " + cmd_params[1]));
                if (config.eeprom.display_brightness_offtime != cmd_params[1].toInt() && cmd_params[1].toInt() >= 0 && cmd_params[1].toInt() <= 255) {
                    config.eeprom.display_brightness_offtime = cmd_params[1].toInt();
                    eeprom_storeconfig = true;
                }
            }
        } else if (cmd_params[0] == "d_ontime") {
            if (cmd_params[1] != NULL) {
                D(Serial.println("display_ontime: " + cmd_params[1]));
                if (config.eeprom.display_ontime != cmd_params[1].toInt() && cmd_params[1].toInt() >= 0 && cmd_params[1].toInt() <= 2459) {
                    config.eeprom.display_ontime = cmd_params[1].toInt();
                    eeprom_storeconfig = true;
                }
            }
        } else if (cmd_params[0] == "d_offtime") {
            if (cmd_params[1] != NULL) {
                D(Serial.println("display_offtime: " + cmd_params[1]));
                if (config.eeprom.display_offtime != cmd_params[1].toInt() && cmd_params[1].toInt() >= 0 && cmd_params[1].toInt() <= 2459) {
                    config.eeprom.display_offtime = cmd_params[1].toInt();
                    eeprom_storeconfig = true;
                }
            }
        } else if (cmd_params[0] == "status") {
            mqtt_Send(pub_client, String(config.mqtt_cmd_status) + "," +
                (MODEL) + "," +
                (FW_VERSION) + "," +
                system_get_free_heap_size() + "," +
                eeprom_IsDeviceInitialized(&config.eeprom) + "," +
                config.eeprom.custom_msg + "," +
                String(config.eeprom.aw_sn) + "," +
                String(config.eeprom.t_offset) + "," +
                String(config.eeprom.h_offset) + "," +
                String(config.eeprom.time_to_sleep) + "," +
                String(config.eeprom.msystem) + "," +
                String(config.eeprom.display_rotation) + "," +
                String(config.eeprom.display_brightness) + "," +
                String(config.eeprom.display_brightness_ontime) + "," +
                String(config.eeprom.display_brightness_offtime) + "," +
                String(config.eeprom.display_ontime) + "," +
                String(config.eeprom.display_offtime)
            );
        } else if (cmd_params[0] == "authtoken") {
            if (cmd_params[1] != NULL) {
                D(Serial.println("authtoken: " + cmd_params[1]));
                if (cmd_params[1].length() == 32) {
                    strcpy(config.eeprom.mqtt_ci, cmd_params[1].c_str());
                    eeprom_storeconfig = true;
                    config.restart_device = true;
                }
            }
        } else if (cmd_params[0] == "fw_update") {
            if (cmd_params[1] != NULL) {
                D(Serial.println("fw_update: " + cmd_params[1]));
                config.fw_update_binfile = cmd_params[1];
            }
        } else if (cmd_params[0] == "f_reset") {
            if (cmd_params[1] != NULL) {
                if (strcmp(cmd_params[1].c_str(), wifi_GetMAC().c_str()) == 0) {
                    // Init default config.
                    eeprom_InitDefaultConfig(&default_config_eeprom);
                    // Init eeprom with default values.
                    eeprom_Reset();
                    eeprom_StoreConfig(&default_config_eeprom);
                    // Reset wifi settings.
                    config.reset_wifi = true;
                    config.restart_device = true;
                }
            }
        }
    }
    if (eeprom_storeconfig == true) {
        eeprom_StoreConfig(&config.eeprom);
    }
}

bool mqtt_Send(PubSubClient *client, String payload) {
    bool status = false;
    if (wifi_IsWifi() == true) {
        payload = String(time(nullptr)) + "," + payload; // Add timestamp to payload.

        D(Serial.println(F("[MQTT] -------------------------------------")));
        D(Serial.println(F("[MQTT] Tx payload:")));
        D(Serial.print(F("[MQTT] ")));
        D(Serial.println(payload));
        D(Serial.print(F("[MQTT] size:")));
        D(Serial.println(payload.length()));
        D(Serial.println(F("[MQTT] -------------------------------------")));

        unsigned long start = millis();
        while (((status = client->publish((char *)(String(config.eeprom.mqtt_ci) + config.mqtt_tt).c_str(), payload.c_str(), false)) == false) &&
            millis() < start + CONNECT_WAIT_SECONDS * SECONDS_TO_MILLIS) {
            delay(500);
            D(Serial.print(F(".")));
        }
        if (status == false) {
            D(Serial.println(F("[MQTT] error: unable to send, either connection lost, or message too large.")));
        }
    } else {
        D(Serial.println(F("[MQTT] error: unable to send, wifi not enabled.")));
    }

    return status;
}

// -----------------------------------------------------------------------------
// HTTP functions.
// -----------------------------------------------------------------------------

// Get HTTP resource and store in "dst" with a max bytes of "dst_length" - 1 to account for NULL string delimeter. Set insecure to true if you need to access a non-sybraset resource.
// Example:
// const size_t content_size = 150;
// char content[content_size] = "";
// http_Get(&ssl_client, "iot.sybraset.com", 8100, "/test.txt");
// http_Get(&ssl_client, "iot.sybraset.com", 8100, "/test.txt", content, content_size);
// http_Get(&ssl_client, "google.com", 443, "/", content, content_size, true);
// Serial.printf("Content received: [%s]\n", content);

uint http_Get(WiFiClientSecure *ssl_client, const char *server, int port, const char *resource, char *dst, size_t dst_length, bool insecure) {
    uint content_length = 0;
    uint content_downloaded_length = 0;

    if (insecure == true) {
        // ssl_client->setInsecure();
    }

    D(Serial.println(F("\nAttempting HTTP connection...")));
    int timeout = 5; // 5 seconds.
    while (!ssl_client->connect(server, port) && (timeout-- > 0)) {
        D(Serial.print(F(".")));
        delay(1000);
    }

    if (!ssl_client->connected()) {
        D(Serial.println(F("failed to connect")));
    } else {
        D(Serial.println(F("connected")));
        // Make HTTP GET request.
        ssl_client->print(String("GET ") + resource + " HTTP/1.1\r\n" +
            "Host: " + server + "\r\n" +
            "Connection: close\r\n\r\n");
        // Read HTTP header.
        D(Serial.println(F("Header:")));
        D(Serial.println(F("============")));
        while (ssl_client->connected()) {
            String line = ssl_client->readStringUntil('\n');
            D(Serial.println(line));
            if (line == "\r") {
                break;
            }
            if (line.startsWith("Content-Length")) {
                size_t max_params = 2;
                String params[max_params];
                util_SplitString(&line, params, max_params, ":");
                if (params[1] != NULL) {
                    D(Serial.println("Content-Length: " + params[1]));
                    content_length = params[1].toInt();
                }
            }
        }
        D(Serial.println(F("============\n")));
        // Read HTTP content.
        D(Serial.println(F("Content:")));
        D(Serial.println(F("============")));
        while (ssl_client->available()) {
            String line = ssl_client->readStringUntil('\n'); // Read line by line.
            content_downloaded_length += line.length() + 1;
            if (dst != NULL && dst_length > 1) { // Store 1 char less to account for NULL string delimeter.
                if (strlen(dst) < dst_length - 1) {
                    size_t max_length = (dst_length - strlen(dst)) - 1;
                    strncat(dst, line.c_str(), max_length);
                }
            }
            D(Serial.println(line));
        }
        D(Serial.println(F("============")));
        D(Serial.println("Content downloaded length: " + String(content_downloaded_length)));
        D(Serial.println(F("closing connection")));
        ssl_client->stop();
    }

    return content_downloaded_length;
}

// -----------------------------------------------------------------------------
// OTA functions.
// -----------------------------------------------------------------------------

void ota_UpdateFw(WiFiClientSecure *ssl_client, String binfile) {
    // Stop existing connection; otherwise won't be able to connect.
    ssl_client->stop();

    // Flash led during update procedure.
    // httpUpdate.setLedPin(LED, LOW);

    printStatusMsg(&config.display, i10n_GetText(STR_FW_UPDATE, config.eeprom.lang).c_str());

    config.eeprom.fw_updated = true;
    eeprom_StoreConfig(&config.eeprom);

    t_httpUpdate_return ret = httpUpdate.update(*ssl_client, "https://" + config.mqtt_svr + ":" + String(config.fs_port) + binfile);

    switch (ret) {
    default:
    case HTTP_UPDATE_FAILED:
        D(Serial.printf("HTTP_UPDATE_FAILED Error (%d): %s\n", httpUpdate.getLastError(), httpUpdate.getLastErrorString().c_str()));
        config.eeprom.fw_updated = false;
        eeprom_StoreConfig(&config.eeprom);
        break;
    case HTTP_UPDATE_NO_UPDATES:
        D(Serial.println("HTTP_UPDATE_NO_UPDATES"));
        break;
    case HTTP_UPDATE_OK:
        D(Serial.println("HTTP_UPDATE_OK"));
        break;
    }
}

// -----------------------------------------------------------------------------
// Localization functions.
// -----------------------------------------------------------------------------

String i10n_GetText(int strid, Localization lang) {
    switch (strid) {
    case STR_AIR_QUALITY_IS_GOOD:
        return (lang == fr) ? F("QUALIT AIR BONNE") : F("AIR QUALITY IS GOOD");
    break;
    case STR_AIR_QUALITY_IS_OK:
        return (lang == fr) ? F("QUALIT DAIR OK") : F("AIR QUALITY IS OK");
    break;
    case STR_AIR_QUALITY_IS_POOR:
        return (lang == fr) ? F("QUALIT AIR PAUVRE") : F("AIR QUALITY IS POOR");
    break;
    case STR_SENSORS_CALIBRATING:
        return (lang == fr) ? F("CALIBRATION SENSEURS") : F("SENSORS CALIBRATING");
    break;
    case STR_PLIFEP:
        return (lang == fr) ? F("(vie)") : F("(life)");
    break;
    case STR_HUMIDITY:
        return (lang == fr) ? String("Humidit") + (char)0x82: "Humidity";
    break;
    case STR_PRESSURE:
        return (lang == fr) ? F("Pression") : F("Pressure");
    break;
    case STR_SEARCHING_AIRTHINGS:
        return (lang == fr) ? F("Recherche pour appareil Airthings...") : F("Searching for Airthings device...");
    break;
    case STR_SEARCHING_AIRTHINGS_SN:
        return (lang == fr) ? String("Recherche pour appareil Airthings avec num")+(char)0x82+"ro de s" +(char)0x82+"rie :\n\n" : "Searching for Airthings device with S/N:\n\n";
    break;
    case STR_WIFI_CONNECT:
        return (lang == fr) ? F("Connexion au Wi-Fi...") : F("Connecting to Wi-Fi...");
    break;
    case STR_FW_UPDATE:
        return (lang == fr) ? String("Installation de la mise ")+(char)0x85+" jour..." : "Installing update...";
    break;
    }

    return "";
}

// -----------------------------------------------------------------------------
// Util functions.
// -----------------------------------------------------------------------------

// Set time via NTP, as required for x.509 validation.
void util_SetClockWithNTP(struct tm *ctime) {
    setenv("TZ", "EST+5EDT,M3.2.0/2,M11.1.0/2", 1);
    tzset();

    configTime(0, 0, "pool.ntp.org", "time.nist.gov");
    D(Serial.print(F("[NTP] waiting for NTP time sync: ")));
    time_t now = time(nullptr);
    while (now < 8 * 3600 * 2) {
        delay(500);
        D(Serial.print(F(".")));
        now = time(nullptr);
    }

    struct tm timeinfo;
    // gmtime_r(&now, &timeinfo);
    // Serial.print("\nCurrent time: ");
    // Serial.print(asctime(&timeinfo));
    D(Serial.printf("\n[NTP] current local time and date: %s", asctime(localtime_r(&now, &timeinfo))));
    localtime_r(&now, &timeinfo);
    *ctime = timeinfo;
}

void util_GetCurrentTime(struct tm *ctime) {
    time_t now = time(nullptr);
    struct tm timeinfo;

    // D(Serial.printf("\n[TIME] current local time and date: %s", asctime(localtime_r(&now, &timeinfo))));
    localtime_r(&now, &timeinfo);
    *ctime = timeinfo;
}

void util_SplitString(String *src_str, String *arr, size_t arr_length, char *sep) {
    char *token, *str, *tofree;

    tofree = str = strdup(src_str->c_str());

    int i = 0;
    while ((token = strsep(&str, sep))) {
        arr[i] = token;
        arr[i].trim(); // trim white space
        i++;
        if (i == arr_length) break;
    };

    free(tofree);
}

// Time white space from str.
char *util_Trim(char *str) {
    size_t len = 0;
    char *frontp = str;
    char *endp = NULL;

    if (str == NULL) { return NULL; }
    if (str[0] == '\0') { return str; }

    len = strlen(str);
    endp = str + len;
    /* Move the front and back pointers to address the first non-whitespace
    * characters from each end.
    */
    while (isspace((unsigned char) *frontp)) { ++frontp; }
    if ( endp != frontp ) {
        while (isspace((unsigned char) *(--endp)) && endp != frontp) {}
    }

    if (frontp != str && endp == frontp) {
        *str = '\0';
    }
    else if (str + len - 1 != endp) {
        *(endp + 1) = '\0';
    }
    /* Shift the string so that it starts at str so that if it's dynamically
    * allocated, we can still free it on the returned pointer.  Note the reuse
    * of endp to mean the front of the string buffer now.
    */
    endp = str;
    if ( frontp != str ) {
        while (*frontp) { *endp++ = *frontp++; }
        *endp = '\0';
    }

    return str;
}

// Convert string of hex to hex array.
int util_Char2Int(char input) {
    if (input >= '0' && input <= '9')
        return input - '0';
    if (input >= 'A' && input <= 'F')
        return input - 'A' + 10;
    if (input >= 'a' && input <= 'f')
        return input - 'a' + 10;
    throw std::invalid_argument("Invalid input string");
}

// This function assumes src to be a zero terminated sanitized string with an even number of [0-9a-f] characters, and target to be sufficiently large.
void util_Hex2Bin(const char *src, char *target) {
    while (*src && src[1]) {
        *(target++) = util_Char2Int(*src)*16 + util_Char2Int(src[1]);
        src += 2;
    }
}
