#include <Arduino.h>

// adxl345 libs
#include <Adafruit_I2CDevice.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <SPI.h>

// Neo-6m libs
#include <SoftwareSerial.h>

// GPS defines
#define GPS_RX_PIN 32
#define GPS_TX_PIN 33

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