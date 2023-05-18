#include <Arduino.h>

// adxl345 libs
#include <Adafruit_I2CDevice.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

// ST7735 tft lcd libs
#include <Adafruit_ST7735.h>
#include <SPI.h>

// Neo-6m libs
#include <SoftwareSerial.h>

// GPS defines
#define GPS_RX_PIN 32
#define GPS_TX_PIN 33

// ST7735 defines
#define TFT_CS 4   // Marked as pin A5 on the board
#define TFT_RST 26 // Marked as pin A0 on the board
#define TFT_A0 13  // Marked as pin A12 on the board
#define MOSI_PIN 18
#define MISO_PIN 19
#define SCK_PIN 5

// ST7735 variables
Adafruit_ST7735 tft = Adafruit_ST7735(TFT_CS, TFT_A0, TFT_RST);

// Neo-6m variables
SoftwareSerial gpsSerial(GPS_RX_PIN, GPS_TX_PIN);

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