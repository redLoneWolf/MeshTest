#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>

#include <Adafruit_HMC5883_U.h>
#include <HardwareSerial.h>

#include <TinyGPS++.h>
#include <Arduino_JSON.h>
#include<Preferences.h>





#define GPS_BAUD_RATE 115200
#define GPS_RX GPIO_NUM_32
#define GPS_TX GPIO_NUM_33

void initSensors(Preferences preferences);

void initJson();

void initGPS(int mode);

void initBMP();

void initCompass();

void initAll();

void updateAll();

JSONVar getJsonData();

String getJsonString();


void HotStartGPS();

void WarmStartGPS();

void ColdStartGPS();