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
#define HR_SENSOR_PIN 36               // Marked as pin A3 on the board
#define HR_HIGH_PULSE_READING 2000     // 12 bit analog read value
#define MAX_HEART_RATE 180             // bpm
#define MIN_HEART_RATE 60              // bpm
#define HEART_RATE_THRESH 150          // bpm
#define MAX_HEART_RATE_THREASH_COUNT 5 // how many consecutive times must the thresh be crossed
#define PRINT_PULSE true               // dictates wether HR will be printed of tft screen
#define HR_STALE_TIMEOUT 3000

////////////// Push button defines //////////////
#define PUSH_BUTTON_PIN 39 // Marked as pin A4 on the board

////////////// Wifi variables //////////////
// Wifi address

// phone
const char *ssid = "Galaxy A53 5G 4C72";
const char *password = "otkm7250";

/////////// callmebot variables ///////////
String phoneNumber = "+972528854006";
String apiKey = "7335050";
String messsage = "Here's the location you requested: https://www.google.com/maps/search/?api=1&query=32.095368,34.769905";

// ST7735 variables
Adafruit_ST7735 tft = Adafruit_ST7735(TFT_CS, TFT_A0, TFT_RST);
bool status_ok = false;

// Neo-6m variables
SoftwareSerial gpsSerial(GPS_RX_PIN, GPS_TX_PIN);
TinyGPSPlus gps;
double lattitude = 32.095302;
double longitude = 34.769904;

// ADXL345 variables
Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified(12345);

// Heart rate variables
/////////// Heart rate sensor variables
bool peak_detected = false;
int heart_rate = 60;
unsigned long int last_peak_time = 10; // value lager than 0 (so there will be no division by 0)
unsigned long int last_hr_measurement_time;
int heart_rate_thresh_count = 0;

////////////////////////////// Function prototypes //////////////////////////////

// setup functions
inline void TFT_setup(void);
inline bool accel_setup(void);
inline bool wifi_setup(void);
inline void gps_setup(void);
inline void HR_setup();

// test functions
inline void accel_test_loop(void);
inline void accel_test_loop_serial(void);
inline void heart_rate_test_loop(void);
inline void calculate_heart_rate_test_loop(void);
inline void gps_test_loop(void);

// main algorithm functions
inline void main_operational_setup(void);
inline void main_operational_loop(void);

// misc functions

float accel_get_acceleration_norm(void);
void accel_buffer_add_data(float);
float accel_buffer_get_oldest_data(void);
void get_gps_raw(void);
bool get_gps_lat_long(void);
void tft_print_lat_long(void);
bool send_location_msg(void);
bool sendWhatsappMessage(String);
void exception_handler(String);
void tft_print_headline(String, uint16_t, uint16_t);
void tft_print_headline(String, uint16_t);
void tft_print_ok(void);
bool calculate_heart_rate(void);
void update_heart_rate(void);
void print_HR_on_lcd(void);
bool detect_stale_pulse(void);

void push_button_setup();
bool push_botton_pressed();
void send_push_button_alert();
void send_fall_alert(void);
void send_hit_alert();
void send_heart_rate_alert();

////////////////////// main / loop //////////////////////
void setup()
{
  // put your setup code here, to run once:
  main_operational_setup();
}

void loop()
{
  // put your main code here, to run repeatedly:
  main_operational_loop();
}

///////////////////////////////////////////////////////
////////////////////// Functions //////////////////////
///////////////////////////////////////////////////////

///////////////////////////// main algorithm /////////////////////////////
inline void main_operational_setup(void)
{
  Serial.begin(9600);
  push_button_setup();
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
  get_gps_lat_long();

  // read sensors
  float acc_norm = accel_get_acceleration_norm();
  calculate_heart_rate();

  // check free fall condition
  if (acc_norm < ACC_MIN_VALUE)
  {
    send_fall_alert();
    delay(4000);
    tft_print_ok();
  }

  // check hit condition
  if (acc_norm > ACC_MAX_VALUE)
  {
    send_hit_alert();
    delay(4000);
    tft_print_ok();
  }

  // check heart rate condition
  if (heart_rate > HEART_RATE_THRESH)
  {
    // send alarm
    send_heart_rate_alert();
    delay(4000);
    tft_print_ok();
  }

  if (push_botton_pressed())
  {
    send_push_button_alert();
    delay(4000);
    tft_print_ok();
  }
}

///////////////////////////// Heart rate sensor /////////////////////////////
inline void HR_setup(void)
{
  // setup analog pin
  pinMode(HR_SENSOR_PIN, INPUT); // its not harmful although its unnecessary

  int heart_rate = calculate_heart_rate();

  return;
}

bool calculate_heart_rate()
{
  int sensor_value = analogRead(HR_SENSOR_PIN);
  int threshold = HR_HIGH_PULSE_READING;
  int measured_heart_rate = 0;
  bool success = false;

  if (sensor_value > threshold && !peak_detected)
  {
    /* A peak was detected. conduct calculations*/

    // calculte measured heart rate and see that it fits withing specified boundaries

    measured_heart_rate = (int)(60000.0 / (double)(millis() - last_peak_time));

    if (measured_heart_rate >= MIN_HEART_RATE && measured_heart_rate <= MAX_HEART_RATE)
    {
      heart_rate = measured_heart_rate;
      success = true;
      last_hr_measurement_time = millis();
    }
    else
    {
      // do nothing. heart rate not updated
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

  if ((status_ok && success && PRINT_PULSE == true) || detect_stale_pulse())
  {
    print_HR_on_lcd();
  }
  return success;
}

void print_HR_on_lcd()
{
  int x_offset = 0;
  int y_offset = 108;

  tft.setCursor(x_offset, y_offset);
  tft.setTextSize(2);
  tft.setTextColor(ST7735_INVCTR);
  tft.fillRect(x_offset,
               y_offset - 12,
               x_offset + 100,
               y_offset + 12,
               ST7735_BLACK);

  tft.printf("HR = %d", heart_rate);

  return;
}

void send_heart_rate_alert()
{
  if (heart_rate_thresh_count < MAX_HEART_RATE_THREASH_COUNT)
  {
    heart_rate_thresh_count++;
    heart_rate = 0;
    return;
  }

  heart_rate_thresh_count = 0;
  heart_rate = 0;
  status_ok = false;

  tft_print_headline("High HR detected!", ST7735_WHITE, ST7735_INVCTR);
  tft.setTextColor(ST7735_WHITE);
  tft.setTextSize(1);
  tft.printf("sending alarm and current \nlocation\n\n");
  delay(2000);
  bool success = send_location_msg();

  if (success == true)
  {
    tft.printf("\n\ndistress message sent!");
  }
  else
  {
    tft.printf("\n\nGood luck!");
  }
}

bool detect_stale_pulse()
{
  if (millis() - last_hr_measurement_time > HR_STALE_TIMEOUT)
  {
    last_hr_measurement_time = millis();
    heart_rate = 0;
    return true;
  }
  else
  {
    return false;
  }
}

///////////////////////////// Wifi /////////////////////////////
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
    if (count >= 10)
    {
      exception_handler("can't connect to Wifi");
    }
  }
  tft.println("\n\nWiFi connected ");
  tft.print("Connected to WiFi network with IP address: \n");
  tft.println(WiFi.localIP());
  tft.println();

  delay(1000);

  return true;
}

bool send_location_msg()
{
  bool success = false;
  char msg_buffer[100];
  sprintf(msg_buffer, "Help! here's my location: https://www.google.com/maps/search/?api=1&query=%f,%f", lattitude, longitude);

  success = sendWhatsappMessage(String(msg_buffer));

  if (success)
  {
    tft_print_headline("message sent!", ST7735_GREEN);
    tft.setTextColor(ST7735_WHITE);
    tft.setTextSize(1);
    tft.printf("lat: %f\nlong: %f", lattitude, longitude);
    Serial.printf("lat: %f\nlong: %f", lattitude, longitude);
  }
  while (gpsSerial.available())
  {
    int tmp = gpsSerial.read();
    delay(GPS_SERIAL_READ_DELAY);
  }
  delay(1);

  return success;
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
    Serial.print("\n\nMessage sent successfully\n");
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

///////////////////////////// lcd /////////////////////////////
inline void TFT_setup(void)
{
  SPI.begin();
  tft.initR(INITR_GREENTAB);
  // set
  tft.fillScreen(ST77XX_BLACK);
  tft.setRotation(1);
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

void tft_print_headline(String headline, uint16_t txt_color, uint16_t screen_color)
{
  tft.fillScreen(screen_color);
  tft.setTextSize(2);
  tft.setTextColor(txt_color);
  tft.setCursor(0, 0);
  tft.printf("%s\n\n", headline.c_str());
}

void tft_print_headline(String headline, uint16_t txt_color)
{
  tft.fillScreen(ST7735_BLACK);
  tft.setTextSize(2);
  tft.setTextColor(txt_color);
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

  status_ok = true;
  return;
}

///////////////////////////// accelerometer /////////////////////////////
inline bool accel_setup()
{
  Wire.begin();
  // begin accelerometer operation
  bool success = accel.begin();
  // set range to +-16g (+-156.9 m/s)
  accel.setRange(ADXL345_RANGE_16_G);
  // get and print sensor data on serial
  sensor_t sensor_data;
  accel.getSensor(&sensor_data);

  // Serial print the accelerometer data
  Serial.println("\n////////////////////// adxl345 sensor data //////////////////////");
  Serial.printf("Sensor id: %d", sensor_data.sensor_id);
  Serial.printf("Resolution: %.3f [m/s^2]\n", sensor_data.resolution);
  Serial.printf("max_value: %.2f [m/s^2]\n", sensor_data.max_value);
  Serial.printf("min_value: %.2f [m/s^2]\n", sensor_data.min_value);

  // print to tft screen
  tft_print_headline("ADXL345", ST7735_MAGENTA);
  tft.setTextColor(ST7735_WHITE);
  tft.setTextSize(1);
  tft.printf("Sensor id: %d\n", sensor_data.sensor_id);
  tft.printf("Resolution: %.3f [m/s^2]\n", sensor_data.resolution);
  tft.printf("max_value: %.2f [m/s^2]\n", sensor_data.max_value);
  tft.printf("min_value: %.2f [m/s^2]\n", sensor_data.min_value);

  // set delay to see the data
  delay(1000);

  // return the success of the accel.begin method
  return success;
}

float accel_get_acceleration_norm()
{
  // get accelerometer data
  sensors_event_t event;
  accel.getEvent(&event);

  float offset = -1.0; // [m/s^2]
  float acc_norm = sqrt(pow(event.acceleration.x, 2) + pow(event.acceleration.y, 2) + pow(event.acceleration.z, 2));
  float adj_acc_norm = acc_norm + offset;

  // calculate and return acceleration vector norm
  return adj_acc_norm; // found an offset of around 1 m/s^2
}

void send_fall_alert()
{
  status_ok = false;
  tft_print_headline("Fall detected!", ST7735_WHITE, ST7735_INVCTR);
  tft.setTextColor(ST7735_WHITE);
  tft.setTextSize(1);
  tft.printf("sending alarm and current \nlocation\n\n");
  delay(2000);
  bool success = send_location_msg();

  if (success == true)
  {
    tft.printf("\n\ndistress message sent!");
  }
  else
  {
    tft.printf("\n\nyou're on you own my man!");
  }
}

void send_hit_alert()
{
  status_ok = false;
  tft_print_headline("Hit detected!", ST7735_WHITE, ST7735_INVCTR);
  tft.setTextColor(ST7735_WHITE);
  tft.setTextSize(1);
  tft.printf("sending alarm and current \nlocation\n\n");
  delay(2000);
  bool success = send_location_msg();

  if (success == true)
  {
    tft.printf("\n\ndistress message sent!");
  }
  else
  {
    tft.printf("\n\nGood luck!");
  }
}

///////////////////////////// GPS /////////////////////////////
void gps_setup()
{
  Serial.println("////////////////////// NEO-6M GPS setup //////////////////////");
  Serial.println("Initiate softwareSerial on pins " + String(GPS_RX_PIN) + " (RX)  and pin" + String(GPS_TX_PIN) + " (TX)");
  gpsSerial.begin(9600);
  delay(500);
  while (gpsSerial.available())
  {
    Serial.print(gpsSerial.read());
    delay(1);
  }

  Serial.println("\nFinished GPS setup\n");

  // write to TFT screen
  tft_print_headline("Neo-6m", ST7735_MAGENTA);
  delay(500);
  tft.setTextColor(ST7735_WHITE);
  tft.setTextSize(1);
  tft.printf("Finished GPS setup\n\n");
  tft.printf("getting current location");

  for (int i = 0; i < 2; i++)
  {
    get_gps_lat_long();
    delay(1000);
  }

  tft.printf("\n\nlat: %.6f \nlong: %.6f\n", lattitude, longitude);
  delay(1000);

  // for (int i = 0; i < 3; i++)
  // {
  //   tft.print(".");
  //   if (send_location_msg())
  //   {
  //     break;
  //   }
  //   delay(2000);
  // }
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

    tft_print_lat_long();

    success = true;
  }
  return success;
}

void tft_print_lat_long()
{
  if (!status_ok)
  {
    return;
  }

  int x_offset = 0;
  int y_offset = 36;

  tft.setTextSize(1);
  tft.setTextColor(ST7735_WHITE);
  tft.setCursor(x_offset, y_offset);
  tft.fillRect(x_offset,
               y_offset,
               x_offset + 100,
               y_offset + 24,
               ST7735_BLACK);
  tft.printf("lat: %.6f\nlong: %.6f", lattitude, longitude);

  return;
}

///////////////////////////// Push button /////////////////////////////
void push_button_setup()
{
  pinMode(PUSH_BUTTON_PIN, INPUT);
}

bool push_botton_pressed()
{
  return digitalRead(PUSH_BUTTON_PIN);
}

void send_push_button_alert()
{
  status_ok = false;
  tft_print_headline("Button Press!", ST7735_WHITE, ST7735_INVCTR);
  tft.setTextColor(ST7735_WHITE);
  tft.setTextSize(1);
  tft.printf("sending alarm and current \nlocation\n\n");
  delay(2000);
  bool success = send_location_msg();

  if (success == true)
  {
    tft.printf("\n\ndistress message sent!");
  }
  else
  {
    tft.printf("\n\nyou're on you own my man!");
  }
}

///////////////////////////// tests /////////////////////////////
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
  Serial.println(analogRead(HR_SENSOR_PIN));
}

inline void calculate_heart_rate_test_loop()
{
  Serial.println(calculate_heart_rate());
}

inline void gps_test_loop()
{
  while (!get_gps_lat_long())
  {
    // do nothing
  }
  // tft.fillScreen(ST7735_BLACK);
  tft.setTextSize(1);
  tft.setCursor(0, 0);
  tft.setTextColor(ST7735_MAGENTA);
  tft.printf("\n\nlat: %.13f \n", lattitude);
  tft.setTextColor(ST7735_ORANGE);
  tft.printf("long: %.13f \n", longitude);
  char str[100];
  sprintf(str, "lat: %.16f  long: %.16f \n", lattitude, longitude);
  Serial.printf("%s\n", str);
  // Serial.printf("lat: %.16f  long: %.16f \n", lattitude, longitude);
  delay(1000);
  return;
}

void exception_handler(String msg)
{
  status_ok = false;
  tft.fillScreen(ST7735_BLACK);
  tft.setTextColor(ST7735_INVCTR);
  tft.setTextSize(2);
  tft.setCursor(0, 0);
  tft.printf("Exception!\n\n");
  tft.setTextColor(ST7735_WHITE);
  tft.setTextSize(1);
  tft.printf("%s\n", msg.c_str());
  tft.setTextColor(ST7735_GREEN);
  tft.printf("\n\nreset controller to \ncontinue!");
  delay(20000);

  while (1)
  {
    // do nothing
  }
}