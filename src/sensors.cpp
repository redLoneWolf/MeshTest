#include "sensors.h"
#include <cmath>



Adafruit_BMP280 bmp; // use I2C interface
Adafruit_Sensor *bmp_temp = bmp.getTemperatureSensor();
Adafruit_Sensor *bmp_pressure = bmp.getPressureSensor();

TinyGPSPlus gps;

Adafruit_HMC5883_Unified mag = Adafruit_HMC5883_Unified(12345);

float temperature, pressure;
float headingDegrees;
TinyGPSLocation location;
int noOfSats = 0;
double speed, altitude;

float ADC_VALUE = 0;
float voltage_value = 0;

JSONVar jsonReadings;
bool gpsStatus = false;
bool magStatus = false;
bool bmpStatus = false;

void initJson()
{
    jsonReadings["temp"] = "Hello Guys";
    //  jsonReadings["temp"] = temperature;
    // jsonReadings["pres"] = pressure;
    // jsonReadings["lat"] = location.lat();
    // jsonReadings["long"] = location.lng();
    // jsonReadings["alt"] = altitude;
    // jsonReadings["speed"] = speed;
    // jsonReadings["volt"] = voltage_value;
}

void HotStartGPS(){
    Serial1.print("$PMTK101*32");
}
void WarmStartGPS(){
    Serial1.print("$PMTK102*31");
}

void ColdStartGPS(){
    Serial1.print("$PMTK103*30");
}

void initGPS(int mode=1)
{   gpsStatus= true;
    Serial1.begin(GPS_BAUD_RATE, SERIAL_8N1, GPS_RX, GPS_TX);
    // HotStartGPS();
    switch (mode)
    {
    case 1:
        HotStartGPS();
        break;
    case 2:
        WarmStartGPS();
        break;
    case 3:
        ColdStartGPS();
        break;
    
    default:
        break;
    }
}

void initBMP()
{   bmpStatus = true;
    unsigned status;
    status = bmp.begin(BMP280_ADDRESS_ALT, BMP280_CHIPID);
    // status = bmp.begin();
    if (!status)
    {
        //  Send Error
        Serial.println("BMP error");
        // while (1)
        //     delay(10);
    }
    Serial.println("BMP Ok");

    /* Default settings from datasheet. */
    bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                    Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                    Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                    Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                    Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */

    bmp_temp->printSensorDetails();
}

void initCompass()
{   magStatus = true;
    if (!mag.begin())
    {
        /* There was a problem detecting the HMC5883 ... check your connections */
        Serial.println("Ooops, no HMC5883 detected ... Check your wiring!");
        // while (1)
        //     ;
    }
    Serial.println("Compass Fine");
}


void initSensors(Preferences preferences){
  if(preferences.getBool("bmp")){
    initBMP();
  }
  if(preferences.getBool("mag")){
    initCompass();
  }
  if(preferences.getBool("gps")){
      initGPS();
  }
}

void initAll()
{
    initBMP();
    initCompass();
    initGPS();
}
sensors_event_t temp_event, pressure_event;
sensors_event_t event;
void updateBMP()
{
    
    if(bmp_temp->getEvent(&temp_event) && bmp_pressure->getEvent(&pressure_event)){
        temperature = temp_event.temperature;

        pressure = pressure_event.pressure;
    }
    
}

void updateCompass()
{
    
    if (mag.getEvent(&event))
    {
        float heading = atan2(event.magnetic.y, event.magnetic.x);
        // Serial.println(heading);

        // Once you have your heading, you must then add your 'Declination Angle', which is the 'Error' of the magnetic field in your location.
        // Find yours here: http://www.magnetic-declination.com/
        // Mine is: -13* 2' W, which is ~13 Degrees, or (which we need) 0.22 radians
        // If you cannot find your Declination, comment out these two lines, your compass will be slightly off.
        float declinationAngle = 0.22;
        heading += declinationAngle;

        // Correct for when signs are reversed.
        if (heading < 0)
            heading += 2 * PI;

        // Check for wrap due to addition of declination.
        if (heading > 2 * PI)
            heading -= 2 * PI;

        // Convert radians to degrees for readability.
        headingDegrees = heading * 180 / M_PI;
    }
}

void updateGPS()
{
    if (gps.encode(Serial1.read())) // encode gps data
    {
        noOfSats = gps.satellites.value();
        location = gps.location;
        altitude = gps.altitude.meters();
        speed = gps.speed.mps();
    }
}

void updateBat()
{
    // ADC_VALUE = analogRead(GPIO_NUM_4);

    // voltage_value = (ADC_VALUE * 3.038) / (4095);
}

void updateJson()
{
    jsonReadings["temp"] = std::ceil(temperature * 100.0) / 100.0;
    jsonReadings["pres"] = std::ceil(pressure * 100.0) / 100.0;

    jsonReadings["heading"] = std::ceil(headingDegrees * 100.0) / 100.0;

    jsonReadings["lat"] = location.lat();
    jsonReadings["long"] = location.lng();
    jsonReadings["alt"] = altitude;
    jsonReadings["speed"] = speed;
    jsonReadings["volt"] = voltage_value;
}

void updateAll()
{
    if(bmpStatus)
        updateBMP();
    
    if(magStatus)
        updateCompass();
    
    if(gpsStatus)
        updateGPS();
    // updateBat();
    updateJson();
}

JSONVar getJsonData()
{

    return jsonReadings;
}

String getJsonString()
{
    return JSON.stringify(jsonReadings);
}