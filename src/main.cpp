#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_I2CDevice.h>
#include <SPI.h>
#include <ESP8266WiFi.h>
#include <ESP8266HTTPClient.h>
#include <WiFiClient.h>

#define BPM_SENSOR_PIN PIN_A0
#define BPM_HIGH_PULSE_READING 600

///////////////////// Global variables /////////////////////

// wifi variables
const char *ssid = "liornet_WR";
const char *ssid = "0544988409";

String server_name = "https://api.callmebot.com/whatsapp.php?phone=0528854006&text=Here%27s%20the%20location%20you%20requested:%20https://www.google.com/maps/search/?api=1%26query=32.166206415848514,34.89679626029493&apikey=7335050";

// heart rate sensor variables
bool peak_detected = false;
int heart_rate = 60;
unsigned long int last_peak_time = false;

// Function prototypes
bool calculate_heart_rate(int, int, int);

void setup()
{
  // put your setup code here, to run once:
  pinMode(LED_BUILTIN, OUTPUT);
  Serial.begin(9600);
}

void loop()
{
  // Serial.println(analogRead(PIN_A0));
  // Serial.println("\t");
  calculate_heart_rate(BPM_SENSOR_PIN, 180, 50);
  // put your main code here, to run repeatedly:
}

bool calculate_heart_rate(int signal_pin, int max_heart_rate, int min_heart_rate)
{
  int sensor_value = analogRead(signal_pin);
  int threshold = 600;
  bool success = false;

  if (sensor_value > threshold && !peak_detected)
  {
    /* A peak was detected. conduct calculations*/

    // calculte measured heart rate and see that it fits withing specified boundaries
    int measured_heart_rate = 60000 / (int)(millis() - last_peak_time);
    if (measured_heart_rate >= min_heart_rate && measured_heart_rate <= max_heart_rate)
    {
      heart_rate = measured_heart_rate;
      success = true;
    }
    else
    {
      heart_rate = 0;
      success = false;
    }

    // mark peak detection and reset last peak time measurement
    peak_detected = true;
    last_peak_time = millis();

    // debug print
    Serial.println("thresh!  " + String(heart_rate));
  }
  else if (sensor_value < threshold && peak_detected)
  {
    /* sensor reading left peak values. reset peak detection*/
    peak_detected = false;
  }

  return success;
}