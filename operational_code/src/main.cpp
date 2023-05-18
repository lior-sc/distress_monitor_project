#include <Arduino.h>
#include <vector>
#include <stdio.h>

// accelerometer
#include <Wire.h>
#include <Adafruit_I2CDevice.h>
#include <Adafruit_ADXL345_U.h>

// lcd
#include <Adafruit_ST7735.h>
#include <SPI.h>

// GPS
#include <SoftwareSerial.h>
#include <TinyGPS++.h>

// WiFi
#include <ESP8266WiFi.h>

// Callmebot
#include <ESP8266HTTPClient.h>
#include <WiFiClient.h>
#include <UrlEncode.h>
// #include <Callmebot_ESP8266.h>

#define PRINT_VERBOUSE true
#define PRINT_TFT_SCREEN false

#define TFT_CS 15 // ESP8266 GPIO NUMBER
#define TFT_RST 0 // ESP8266 GPIO NUMBER
#define TFT_DC 2  // ESP8266 GPIO NUMBER

#define PIN_I2C_SCL 22
#define PIN_I2C_SDA 23
#define ACCEL_BUFFER_SIZE 10 // circular buffer size. used for filtering
#define ACC_MAX_VALUE 22     // m/s^2
#define ACC_MIN_VALUE 4      // m/s^2

#define HR_SENSOR_PIN PIN_A0      // pin number
#define HR_HIGH_PULSE_READING 550 // 10 bit analog read value
#define MAX_HEART_RATE 180        // bpm
#define MIN_HEART_RATE 60         // bpm
#define HR_BUFFER_SIZE 15

#define GPS_SERIAL_RX 0 // pin number
// #define GPS_SERIAL_TX 16 // pin number
#define GPS_SERIAL_TX -1 // pin number

#define GPS_SERIAL_READ_DELAY 2 // milliseconds (tested with 2ms)

////////////////////////////// Global variables //////////////////////////////

///////////  wifi variables

//// WR wifi
// const char *ssid = "liornet_WR_2.4_Ghz";
// const char *password = "0544988409";

//// home wifi
// const char *ssid = "liornet";
// const char *password = "0544988409";

// phone wifi
const char *ssid = "Galaxy A53 5G 4C72";
const char *password = "otkm7250";

// callmebot variables
String phoneNumber = "+972528854006";
String apiKey = "7335050";
String messsage = "Here's the location you requested: https://www.google.com/maps/search/?api=1&query=32.095368,34.769905";

/////////// Neo-6M gps variables
SoftwareSerial gpsSerial(GPS_SERIAL_RX, GPS_SERIAL_TX);
TinyGPSPlus gps;
double lattitude = 32.095302;
double longitude = 34.769904;

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
// sensors_event_t accel_event[2];

/////////// Heart rate sensor variables
typedef struct
{
  int buffer[100];
  int buffer_size;
  int head;
  int tail;

} CIRCULAR_BUFFER;

CIRCULAR_BUFFER hr_cb;

bool peak_detected = false;
int avg_heart_rate = 60;
unsigned long int last_peak_time = 0;

////////////////////////////// Function prototypes //////////////////////////////

// setup functions
inline void TFT_setup(void);
inline bool accel_setup(void);
inline bool wifi_setup(void);
inline void gps_setup(void);

// test functions
inline void accel_test_loop(void);
inline void accel_test_loop_serial(void);
inline void heart_rate_test_loop(void);

// main algorithm functions
inline void main_operational_setup(void);
inline void main_operational_loop(void);

// misc functions
int calculate_heart_rate(void);
float accel_get_acceleration_norm(void);
void accel_buffer_add_data(float);
float accel_buffer_get_oldest_data(void);
void get_gps_raw(void);
bool get_gps_lat_long(void);
char msg_buffer[100];
bool send_location_msg();
bool sendWhatsappMessage(String);
void exception_handler(char *msg);
void tft_print_headline(String);
void tft_print_ok(void);
void add_to_buffer(CIRCULAR_BUFFER *cb, int value);
int read_from_buffer(CIRCULAR_BUFFER *cb);
void update_avg_heart_rate();
void update_heart_rate();
void HR_setup();

////////////////////////////// Setup & Loop functions //////////////////////////////

void setup()
{
  // main_operational_setup();
  Serial.begin(115200);
  accel_setup();
}

void loop()
{
  // main_operational_loop();
  accel_test_loop_serial();
}

////////////////////////////// Functions //////////////////////////////

// main algorithm
inline void main_operational_setup(void)
{
  Serial.begin(115200);
  HR_setup();
  TFT_setup();
  wifi_setup();
  accel_setup();
  gps_setup();
  tft_print_ok();
}

inline void main_operational_loop(void)
{
  // get location
  // get_gps_lat_long();

  // read sensors
  float acc_norm = accel_get_acceleration_norm();
  // bool HR_calculation_success = calculate_heart_rate();
  update_avg_heart_rate();

  // check free fall condition
  if (acc_norm < ACC_MIN_VALUE)
  {
    // fall detected
    tft_print_headline("Fall detected!");
    tft.setTextColor(ST7735_WHITE);
    tft.setTextSize(1);
    tft.printf("sending alarm and current \nlocation\n\n");
    bool success = send_location_msg();

    if (success == true)
    {
      tft.printf("distress message sent!");
    }
    else
    {
      tft.printf("you're on you own my man!");
    }
    delay(2000);
    tft_print_ok();
  }

  // check hit condition
  if (acc_norm > ACC_MAX_VALUE)
  {
    // hit detected
    tft_print_headline("Hit detected!");
    tft.setTextColor(ST7735_WHITE);
    tft.setTextSize(1);
    tft.printf("sending alarm and current \nlocation\n\n");
    bool success = send_location_msg();

    if (success == true)
    {
      tft.printf("distress message sent!");
    }
    else
    {
      tft.printf("you can do it!");
    }
    delay(2000);
    tft_print_ok();
  }

  // check heart rate condition
  if (avg_heart_rate > MAX_HEART_RATE)
  {
    // send alarm
    tft_print_headline("High HR!");
    tft.setTextColor(ST7735_WHITE);
    tft.setTextSize(1);
    tft.printf("sending alarm and current \nlocation\n\n");
    bool success = send_location_msg();

    if (success == true)
    {
      tft.printf("distress message sent!");
    }
    else
    {
      tft.printf("too bad!");
    }
    delay(2000);
    tft_print_ok();
  }

  // if (digitalRead(9) == HIGH)
  // {
  //   // https://randomnerdtutorials.com/esp8266-pinout-reference-gpios/
  //   // need to pull this pin low with 10K resistor and attach to high

  //   // send alarm
  // }
  delay(10);
}

// Heart rate sensor
void HR_setup()
{
  // setup analog pin
  pinMode(HR_SENSOR_PIN, INPUT); // its not harmful although its unnecessary

  // set circular buffer size (<100)
  hr_cb.buffer_size = HR_BUFFER_SIZE; // size of the buffer we intend to utilize

  // put one value in buffer
  int heart_rate = calculate_heart_rate();
  add_to_buffer(&hr_cb, heart_rate);
}

int calculate_heart_rate()
{
  int sensor_value = analogRead(HR_SENSOR_PIN);
  int threshold = HR_HIGH_PULSE_READING;
  int measured_heart_rate = 0;
  int heart_rate = 0;

  if (sensor_value > threshold && !peak_detected)
  {
    /* A peak was detected. conduct calculations*/

    // calculte measured heart rate and see that it fits withing specified boundaries
    measured_heart_rate = 60000 / (int)(millis() - last_peak_time);

    if (measured_heart_rate >= MIN_HEART_RATE && measured_heart_rate <= MAX_HEART_RATE)
    {
      heart_rate = measured_heart_rate;
      // success = true;
    }
    else
    {
      // do nothing. heart_rate stays at -1
    }

    // mark peak detection and reset last peak time measurement
    peak_detected = true;
    last_peak_time = millis();
  }
  else if (sensor_value < threshold && peak_detected)
  {
    /* sensor reading left peak values. reset peak detection*/
    peak_detected = false;
  }

  return heart_rate;
}

void update_heart_rate()
{
  int current_heart_rate = calculate_heart_rate();
  add_to_buffer(&hr_cb, current_heart_rate);

  return;
}
void update_avg_heart_rate()
{
  update_heart_rate();

  int sum = 0;
  for (int i = 0; i < HR_BUFFER_SIZE; i++)
  {
    sum += calculate_heart_rate();
  }
  avg_heart_rate = sum / HR_BUFFER_SIZE;
}

void add_to_buffer(CIRCULAR_BUFFER *cb, int value)
{
  cb->buffer[cb->head] = value;
  cb->head = (cb->head + 1) % cb->buffer_size;
  if (cb->head == cb->tail)
  {
    cb->tail = (cb->tail + 1) % cb->buffer_size; // discard oldest value
  }
}

int read_from_buffer(CIRCULAR_BUFFER *cb)
{
  if (cb->head == cb->tail)
  {
    return -1; // buffer is empty
  }
  int value = cb->buffer[cb->tail];
  cb->tail = (cb->tail + 1) % cb->buffer_size;
  return value;
}

// Wifi
inline bool wifi_setup(void)
{
  // establishing wifi connection
  WiFi.begin(ssid, password);
  tft.fillScreen(ST7735_BLACK);
  tft.setTextSize(2);
  tft.setTextColor(ST7735_MAGENTA);
  tft.setCursor(0, 0);
  tft.print("WiFi setup\n\n");

  tft.setTextSize(1);
  tft.setTextColor(ST7735_WHITE);
  tft.printf("Connecting to: \n\n%s", ssid);

  int count = 0;

  while (WiFi.status() != WL_CONNECTED)
  {
    delay(1000);
    tft.print(".");
    count++;
    if (count >= 20)
    {
      exception_handler("can't connect to Wifi");
    }
  }
  tft.println("WiFi connected ");
  tft.print("Connected to WiFi network with IP address: \n");
  tft.println(WiFi.localIP());
  tft.println();

  delay(1000);

  return true;
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

// lcd
inline void TFT_setup(void)
{
  tft.initR(INITR_GREENTAB);
  // set
  tft.fillScreen(ST77XX_BLACK);
  tft.setRotation(3);
  tft.setTextWrap(true);

  tft.setTextSize(4);
  tft.setTextColor(ST7735_ORANGE);
  tft.setCursor(0, 0);
  tft.println("Hello!");

  tft.setTextSize(1);
  tft.setTextColor(ST7735_WHITE);
  tft.print("\nWhen life gets you down, \njust press the ");
  tft.setTextColor(ST7735_GREEN);
  tft.print("yellow \nbutton!\n\n");
  tft.setTextColor(ST7735_WHITE);

  delay(1000);
  tft.print("\n\n  (It doesn't work!!)");
  delay(250);
}

void tft_print_headline(String headline)
{
  tft.fillScreen(ST7735_BLACK);
  tft.setTextSize(2);
  tft.setTextColor(ST7735_MAGENTA);
  tft.setCursor(0, 0);
  tft.printf("%s\n\n", headline.c_str());
}

void tft_print_ok(void)
{
  tft.fillScreen(ST7735_BLACK);
  tft.setCursor(0, 0);

  tft.setTextColor(ST7735_GREEN);
  tft.setTextSize(2);
  tft.printf("Status: OK\n\n");

  return;
}

// accelerometer
inline bool accel_setup()
{
  // setup the I2C pins of the ESP32 and begin I2C
  Wire.begin(23, 22);

  // begin accelerometer operation
  bool success = accel.begin();

  // set range to +-16g (+-156.9 m/s)
  accel.setRange(ADXL345_RANGE_16_G);

  // get and print sensor data on serial
  sensor_t sensor_data;
  accel.getSensor(&sensor_data);

  if (PRINT_VERBOUSE)
  {
    // Serial print the accelerometer data
    Serial.println("\n////////////////////// adxl345 sensor data //////////////////////");
    Serial.printf("Sensor id: %d", sensor_data.sensor_id);
    Serial.printf("Resolution: %.3f [m/s^2]\n", sensor_data.resolution);
    Serial.printf("max_value: %.2f [m/s^2]\n", sensor_data.max_value);
    Serial.printf("min_value: %.2f [m/s^2]\n", sensor_data.min_value);
  }

  if (PRINT_TFT_SCREEN)
  {
    // print to tft screen
    tft_print_headline("ADXL345");
    tft.setTextColor(ST7735_WHITE);
    tft.setTextSize(1);
    tft.printf("Sensor id: %d\n", sensor_data.sensor_id);
    tft.printf("Resolution: %.3f [m/s^2]\n", sensor_data.resolution);
    tft.printf("max_value: %.2f [m/s^2]\n", sensor_data.max_value);
    tft.printf("min_value: %.2f [m/s^2]\n", sensor_data.min_value);
  }

  if (PRINT_VERBOUSE || PRINT_TFT_SCREEN)
  {
    // set delay to see the data
    delay(1000);
  }

  // return the success of the accel.begin method
  return success;
}

float accel_get_acceleration_norm()
{
  // get accelerometer data
  sensors_event_t event;
  accel.getEvent(&event);

  // calculate and return acceleration vector norm
  return (sqrt(pow(event.acceleration.x, 2) + pow(event.acceleration.y, 2) + pow(event.acceleration.z, 2)) - 2);
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

// GPS
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

  // write to TFT screen
  tft_print_headline("Neo-6m");
  delay(500);
  tft.setTextColor(ST7735_WHITE);
  tft.setTextSize(1);
  tft.printf("Finished GPS setup\n\n");
  tft.printf("getting current location");

  for (int i = 0; i < 5; i++)
  {
    get_gps_lat_long();
    delay(1000);
    tft.print(".");
  }

  tft.printf("\n\nlat: %.6f \nlong: %.6f\n", lattitude, longitude);
  delay(1000);

  for (int i = 0; i < 3; i++)
  {
    tft.print(".");
    if (send_location_msg())
    {
      break;
    }
    delay(2000);
  }
}

void get_gps_raw()
{
  if (!gpsSerial.available())
  {
    return;
    // Serial.println("no gps data. waiting");
    // delay(10);
  }
  Serial.println();

  while (gpsSerial.available())
  {
    Serial.print(gpsSerial.read());
    delay(GPS_SERIAL_READ_DELAY);
  }

  Serial.println("\n");
}

bool get_gps_lat_long()
{
  bool success = false;

  if (gpsSerial.available() > 0)
  {
    while (gpsSerial.available() > 0)
    {
      char temp = (char)gpsSerial.read();
      gps.encode(temp);
      delay(2);
    }
  }

  if (gps.location.isValid())
  {
    lattitude = gps.location.lat();
    longitude = gps.location.lng();

    success = true;
  }
  return success;
}

// test functions
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

inline void accel_test_loop_serial()
{
  // get adxl345 reading
  sensors_event_t event;
  accel.getEvent(&event);

  // print acceleration values
  printf("x: %.3f, y: %.3f, z: %.3f, norm: %.3f\n",
         event.acceleration.x,
         event.acceleration.y,
         event.acceleration.z,
         accel_get_acceleration_norm());

  /** @note
   * in order to avoid screen blinking we need to rewrite the number only if it is changed.
   * we need to do a comparison
   * */

  delay(10);
}

inline void heart_rate_test_loop()
{
  Serial.println(analogRead(PIN_A0));
}

inline void gps_test_loop()
{
  while (!get_gps_lat_long())
  {
    // do nothing
  }
  tft.fillScreen(ST7735_BLACK);
  tft.setTextSize(1);
  tft.setCursor(0, 0);
  tft.setTextColor(ST7735_MAGENTA);
  tft.printf("lat: %.13f \n", lattitude);
  tft.setTextColor(ST7735_ORANGE);
  tft.printf("long: %.13f \n", longitude);
  char str[100];
  sprintf(str, "lat: %.16f  long: %.16f \n", lattitude, longitude);
  Serial.printf("%s\n", str);
  // Serial.printf("lat: %.16f  long: %.16f \n", lattitude, longitude);
  delay(1000);
  return;
}

void exception_handler(char *msg)
{
  tft.fillScreen(ST7735_BLACK);
  tft.setTextColor(ST7735_RED);
  tft.setTextSize(2);
  tft.setCursor(0, 0);
  tft.printf("Exception!\n");
  tft.setTextColor(ST7735_WHITE);
  tft.setTextSize(1);
  tft.printf("%s\n", msg);
  tft.setTextColor(ST7735_GREEN);
  tft.printf("\n\nreset controller to \ncontinue!");
  delay(20000);

  while (1)
  {
    // do nothing
  }
}