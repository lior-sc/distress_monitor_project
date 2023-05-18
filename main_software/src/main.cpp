#include <Arduino.h>

// adxl345 libs
#include <Wire.h>
#include <Adafruit_I2CDevice.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL345_U.h>

// ST7735 tft lcd libs
#include <Adafruit_ST7735.h>
#include <SPI.h>

// Neo-6m libs
#include <SoftwareSerial.h>

// Wifi libs
#include <UrlEncode.h>
#include <HTTPClient.h>

////////////// GPS defines /////////////
#define GPS_RX_PIN 32
#define GPS_TX_PIN 33

////////////// ST7735 defines //////////////
#define TFT_CS 4   // Marked as pin A5 on the board
#define TFT_RST 26 // Marked as pin A0 on the board
#define TFT_A0 13  // Marked as pin A12 on the board
#define MOSI_PIN 18
#define MISO_PIN 19
#define SCK_PIN 5

////////////// Wifi variables //////////////

// Wifi address
const char *ssid = "Galaxy A53 5G 4C72";
const char *password = "otkm7250";

/////////// callmebot variables ///////////
String phoneNumber = "+972528854006";
String apiKey = "7335050";
String messsage = "Here's the location you requested: https://www.google.com/maps/search/?api=1&query=32.095368,34.769905";

// ST7735 variables
Adafruit_ST7735 tft = Adafruit_ST7735(TFT_CS, TFT_A0, TFT_RST);

// Neo-6m variables
SoftwareSerial gpsSerial(GPS_RX_PIN, GPS_TX_PIN);

// ADXL345 variables
Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified(12345);

// put function declarations here:
int myFunction(int, int);

void setup()
{
  // put your setup code here, to run once:
  int result = myFunction(2, 3);
}

void loop()
{
  // put your main code here, to run repeatedly:
}

// put function definitions here:
int myFunction(int x, int y)
{
  return x + y;
}