#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_I2CDevice.h>
#include <Adafruit_ADXL345_U.h>
#include <Adafruit_ST7735.h>
#include <SPI.h>
#include <ESP8266WiFi.h>
#include <ESP8266HTTPClient.h>
#include <WiFiClient.h>
#include <TFTv2.h>

#define BPM_SENSOR_PIN PIN_A0
#define TFT_CS 15            // ESP8266 GPIO NUMBER
#define TFT_RST 0            // ESP8266 GPIO NUMBER
#define TFT_DC 2             // ESP8266 GPIO NUMBER
#define ACCEL_BUFFER_SIZE 10 // circular buffer size. used for filtering

#define BPM_HIGH_PULSE_READING 600

////////////////////////////// Global variables //////////////////////////////

/////////// accelerometer variables
Adafruit_ADXL345_Unified accel(12345); // The number 12345 is actually just a placeholder value used to initialize the ADXL345 sensor object.

// circular buffer methodology (search this term if something is unclear)
float accel_buffer[ACCEL_BUFFER_SIZE];
int accel_next_index = 0;
int accel_oldest_index = 0;

/** @note
 * I need to implement the circular buffer method on the heart rate monitor
 */

///////////  TFT variables
Adafruit_ST7735 tft = Adafruit_ST7735(TFT_CS, TFT_DC, TFT_RST);

///////////  wifi variables
const char *ssid = "liornet_WR";
const char *password = "0544988409";

String server_name = "https://api.callmebot.com/whatsapp.php?phone=0528854006&text=Here%27s%20the%20location%20you%20requested:%20https://www.google.com/maps/search/?api=1%26query=32.166206415848514,34.89679626029493&apikey=7335050";

// heart rate sensor variables
bool peak_detected = false;
int heart_rate = 60;
unsigned long int last_peak_time = false;

////////////////////////////// Function prototypes //////////////////////////////
bool calculate_heart_rate(int, int, int);
void TFT_setup(void);
bool accel_setup(void);
bool setup_wifi_connection(void);
float accel_get_acceleration_norm(void);
void accel_buffer_add_data(float);
float accel_buffer_get_oldest_data(void);

////////////////////////////// Setup & Loop functions //////////////////////////////

void setup()
{
  // put your setup code here, to run once:
  pinMode(LED_BUILTIN, OUTPUT);
  Serial.begin(9600);
  TFT_setup();
  accel_setup();
}

void loop()
{
  // Serial.println(analogRead(PIN_A0));
  // Serial.println("\t");
  // calculate_heart_rate(BPM_SENSOR_PIN, 180, 50);
  // put your main code here, to run repeatedly:
}

////////////////////////////// Functions //////////////////////////////
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

inline bool setup_wifi_connection(void)
{
  return true;
}

inline void TFT_setup(void)
{
  tft.initR(INITR_GREENTAB);
  tft.fillScreen(ST77XX_BLACK);

  tft.setCursor(0, 0);
  tft.setTextColor(ST7735_GREEN);
  tft.setTextWrap(true);
  tft.setTextSize(2);
  tft.println("Hello !");
  tft.setTextColor(ST7735_CYAN);
  tft.setTextSize(3);
  tft.println("Hello !");
}

inline bool accel_setup()
{
  return accel.begin();
}

float accel_get_acceleration_norm()
{
  // get accelerometer data
  sensors_event_t event;
  accel.getEvent(&event);

  // calculate and return acceleration vector norm
  return sqrt(pow(event.acceleration.x, 2) + pow(event.acceleration.y, 2) + pow(event.acceleration.z, 2));
}

void accel_buffer_add_data(float newData)
{
  accel_buffer[accel_next_index] = newData;
  accel_next_index = (accel_next_index + 1) % ACCEL_BUFFER_SIZE;
  if (accel_next_index == accel_oldest_index)
  {
    accel_oldest_index = (accel_oldest_index + 1) % ACCEL_BUFFER_SIZE;
  }
}

float accel_buffer_get_oldest_data()
{
  float oldestData = accel_buffer[accel_oldest_index];
  accel_oldest_index = (accel_oldest_index + 1) % ACCEL_BUFFER_SIZE;
  return oldestData;
}