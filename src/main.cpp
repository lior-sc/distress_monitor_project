#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_I2CDevice.h>
#include <Adafruit_ADXL345_U.h>
#include <Adafruit_ST7735.h>
#include <SPI.h>
#include <ESP8266WiFi.h>
#include <WiFiClientSecure.h>
#include <ESP8266Ping.h>
#include <TFTv2.h>
#include <SoftwareSerial.h>

#define BPM_SENSOR_PIN PIN_A0
#define TFT_CS 15            // ESP8266 GPIO NUMBER
#define TFT_RST 0            // ESP8266 GPIO NUMBER
#define TFT_DC 2             // ESP8266 GPIO NUMBER
#define ACCEL_BUFFER_SIZE 10 // circular buffer size. used for filtering
#define BPM_HIGH_PULSE_READING 600

#define GPS_SERIAL_RX 4
#define GPS_SERIAL_TX 5

////////////////////////////// Global variables //////////////////////////////

SoftwareSerial gpsSerial(GPS_SERIAL_RX, GPS_SERIAL_TX);

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
sensors_event_t accel_event[2];

///////////  wifi variables
const char *ssid = "liornet";
const char *password = "0544988409";
const char *host = "api.callmebot.com";
const int httpsPort = 443;
const String api_key = "7335050";
const String phone = "+972528854006";
const String message = "Hello from CallMeBot!";

String server_name = "https://api.callmebot.com/whatsapp.php?phone=0528854006&text=Here%27s%20the%20location%20you%20requested:%20https://www.google.com/maps/search/?api=1%26query=32.166206415848514,34.89679626029493&apikey=7335050";

// heart rate sensor variables
bool peak_detected = false;
int heart_rate = 60;
unsigned long int last_peak_time = false;

////////////////////////////// Function prototypes //////////////////////////////
bool calculate_heart_rate(int, int, int);
void TFT_setup(void);
bool accel_setup(void);
bool wifi_setup(void);
float accel_get_acceleration_norm(void);
void accel_buffer_add_data(float);
float accel_buffer_get_oldest_data(void);
void accel_test_loop(void);
void send_whatsapp_message();
void sendPing();
void gps_setup();
void get_gps_raw();

////////////////////////////// Setup & Loop functions //////////////////////////////

void setup()
{
  // put your setup code here, to run once:
  pinMode(LED_BUILTIN, OUTPUT);
  Serial.begin(9600);
  // TFT_setup();
  // accel_setup();
  // wifi_setup();
  // send_whatsapp_message();
  gps_setup();
}

void loop()
{
  // accel_test_loop();
  get_gps_raw();
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

inline bool wifi_setup(void)
{
  // establishing wifi connection
  WiFi.begin(ssid, password);
  Serial.print("Connecting to ");
  Serial.print(ssid);
  Serial.println("...");

  while (WiFi.status() != WL_CONNECTED)
  {
    delay(1000);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("WiFi connected ");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());

  Serial.println();

  sendPing();

  return true;
}

inline void TFT_setup(void)
{
  tft.initR(INITR_GREENTAB);
  tft.fillScreen(ST77XX_BLACK);

  tft.setRotation(1);
  tft.setCursor(0, 0);
  tft.setTextColor(ST7735_GREEN);
  tft.setTextWrap(true);
  tft.setTextSize(2);
  tft.println("Hello !");
  tft.setTextColor(ST7735_CYAN);
  tft.setTextSize(3);
  tft.println("Hello !");
  delay(1000);
  tft.fillScreen(ST77XX_BLACK);
}

inline bool accel_setup()
{
  // begin accelerometer operation
  bool success = accel.begin();
  // set range to +-16g (+-156.9 m/s)
  accel.setRange(ADXL345_RANGE_16_G);
  // get and print sensor data on serial
  sensor_t sensor_data;
  accel.getSensor(&sensor_data);
  Serial.println("\n////////////////////// adxl345 sensor data //////////////////////");
  Serial.println("Sensor id: " + String(sensor_data.sensor_id));
  Serial.println("Resolution: " + String(sensor_data.resolution));
  Serial.println("max_value: " + String(sensor_data.max_value) + " [m/s^2]");
  Serial.println("min_value: " + String(sensor_data.min_value) + " [m/s^2]");
  // set delay to see the daya should any other information arrive
  delay(1000);
  // return the success of the accel.begin method
  return success;
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

inline void accel_test_loop()
{
  // get adxl345 reading
  sensors_event_t event;
  accel.getEvent(&event);

  // set TFT text configurations and clear existing text
  tft.setCursor(0, 0);
  tft.setTextColor(ST7735_MAGENTA);
  tft.setTextWrap(true);
  tft.setTextSize(2);
  tft.fillRect(36, 0, 100, 52, ST7735_BLACK);
  // print acceleration values
  tft.println("x: " + String(event.acceleration.x));
  tft.println("y: " + String(event.acceleration.y));
  tft.println("z: " + String(event.acceleration.z));
  tft.print("");

  /** @note
   * in order to avoid screen blinking we need to rewrite the number only if it is changed.
   * we need to do a comparison
   * */

  delay(10);
}

void send_whatsapp_message()
{
  WiFiClientSecure client;
  Serial.print("Connecting to ");
  Serial.println(host);
  if (!client.connect(host, httpsPort))
  {
    Serial.println("Connection failed");
    return;
  }

  String url = "/whatsapp.php?phone=" + phone + "&text=" + message + "&apikey=" + api_key;
  Serial.print("Requesting URL: ");
  Serial.println(url);

  client.print(String("GET ") + url + " HTTP/1.1\r\n" +
               "Host: " + host + "\r\n" +
               "User-Agent: ESP8266\r\n" +
               "Connection: close\r\n\r\n");

  Serial.println("Request sent");

  while (client.connected())
  {
    String line = client.readStringUntil('\n');
    if (line == "\r")
    {
      Serial.println("Headers received");
      break;
    }
  }

  Serial.println("Response received");
  String response = client.readStringUntil('\n');
  Serial.println(response);

  Serial.println("Closing connection");
}

void sendPing()
{
  Serial.println("Pinging google to check connection");
  if (Ping.ping("www.google.com", 5))
  {
    Serial.println("Website is reachable");
    Serial.println("average ping time: " + String(Ping.averageTime()) + " [ms]");
  }
  else
  {
    Serial.println("Website is not reachable");
  }
}

void gps_setup()
{
  Serial.println("////////////////////// NEO-6M GPS setup //////////////////////");
  Serial.println("Initiate softwareSerial on pins " + String(GPS_SERIAL_RX) + " (RX)  and pin" + String(GPS_SERIAL_TX) + " (TX)");
  gpsSerial.begin(9600);
  delay(1000);
  while (gpsSerial.available())
  {
    Serial.print(gpsSerial.read());
    delay(1);
  }

  Serial.println("\n\nFinished GPS setup");
}

void get_gps_raw()
{
  if (!gpsSerial.available())
  {
    return;
  }
  Serial.println();

  while (gpsSerial.available())
  {
    Serial.print(gpsSerial.read());
    delay(1);
  }

  Serial.println("\n");
}
