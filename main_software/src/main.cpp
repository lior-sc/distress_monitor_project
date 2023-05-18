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
#include <TinyGPS++.h>

// Wifi libs
#include <UrlEncode.h>
#include <HTTPClient.h>

////////////// GPS defines /////////////
#define GPS_RX_PIN 32
#define GPS_TX_PIN 33
#define GPS_SERIAL_READ_DELAY 2 // milliseconds (tested with 2ms)

////////////// ST7735 defines //////////////
#define TFT_CS 4   // Marked as pin A5 on the board
#define TFT_RST 26 // Marked as pin A0 on the board
#define TFT_A0 13  // Marked as pin A12 on the board
#define MOSI_PIN 18
#define MISO_PIN 19
#define SCK_PIN 5

////////////// Adxl345 defines //////////////
#define ACCEL_BUFFER_SIZE 10 // circular buffer size. used for filtering
#define ACC_MAX_VALUE 22     // m/s^2
#define ACC_MIN_VALUE 4      // m/s^2

////////////// Heart rate sensors defines //////////////
#define HR_SENSOR_PIN PIN_A0      // pin number
#define HR_HIGH_PULSE_READING 550 // 10 bit analog read value
#define MAX_HEART_RATE 180        // bpm
#define MIN_HEART_RATE 60         // bpm
#define HR_BUFFER_SIZE 15

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
TinyGPSPlus gps;
double lattitude = 32.095302;
double longitude = 34.769904;

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

bool sendWhatsappMessage(String message)
{
  bool success = false;
  // Data to send with HTTP POST
  String url = "http://api.callmebot.com/whatsapp.php?phone=" + phoneNumber + "&apikey=" + apiKey + "&text=" + urlEncode(message);
  WiFiClient client;
  HTTPClient http;
  http.begin(client, url);

  // Specify content-type header
  http.addHeader("Content-Type", "application/x-www-form-urlencoded");

  // Send HTTP POST request
  int httpResponseCode = http.POST(url);
  if (httpResponseCode == 200 || httpResponseCode == 503 || httpResponseCode == -11)
  {
    Serial.print("Message sent successfully\n");
    success = true;
  }
  else
  {
    Serial.println("Error sending the message");
    Serial.print("HTTP response code: ");
    Serial.println(httpResponseCode);
  }

  // Free resources
  http.end();

  return success;
}

bool send_location_msg()
{
  bool success = false;
  // tft.fillScreen(ST7735_BLACK);
  // tft.setCursor(0, 0);
  // tft.setTextColor(ST7735_WHITE);
  // tft.setTextSize(1);
  // tft.printf("Trying to send distress signal");

  char msg_buffer[100];
  sprintf(msg_buffer, "Help! here's my location: https://www.google.com/maps/search/?api=1&query=%f,%f", lattitude, longitude);

  success = sendWhatsappMessage(String(msg_buffer));

  // if (success)
  // {
  //   tft.fillScreen(ST7735_BLACK);
  //   tft.setCursor(0, 0);
  //   tft.setTextColor(ST7735_WHITE);
  //   tft.setTextSize(2);
  //   tft.printf("message sent!\n");
  //   tft.setTextSize(1);
  //   tft.printf("lat: %f\nlong: %f", lattitude, longitude);
  //   Serial.printf("lat: %f\nlong: %f", lattitude, longitude);
  // }
  while (gpsSerial.available())
  {
    int tmp = gpsSerial.read();
    delay(GPS_SERIAL_READ_DELAY);
  }
  // delay(1000);

  return success;
}