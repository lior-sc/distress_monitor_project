#include <Arduino.h>

// accelerometer
#include <Wire.h>
#include <Adafruit_I2CDevice.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL345_U.h>

Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified(12345);

// put function declarations here:
int myFunction(int, int);
void toggle_led();
inline bool accel_setup();
void get_acc();

void setup()
{
  Serial.begin(115200);
  pinMode(13, OUTPUT);
  accel_setup();
  delay(000);
  // put your setup code here, to run once:
  int result = myFunction(2, 3);
}

void loop()
{
  // put your main code here, to run repeatedly:
  // Serial.println(myFunction(2, 4));
  get_acc();
  toggle_led();
  delay(100);
}

// put function definitions here:
int myFunction(int x, int y)
{
  return x + y;
}

void toggle_led()
{
  digitalWrite(13, !digitalRead(13));
}

void get_acc()
{
  sensors_event_t event;
  accel.getEvent(&event);

  /* Display the results (acceleration is measured in m/s^2) */
  Serial.print("X: ");
  Serial.print(event.acceleration.x);
  Serial.print("  ");
  Serial.print("Y: ");
  Serial.print(event.acceleration.y);
  Serial.print("  ");
  Serial.print("Z: ");
  Serial.print(event.acceleration.z);
  Serial.print("  ");
  Serial.println("m/s^2 ");
}

inline bool accel_setup()
{
  // we need to setup the SDa and SCL pins to be 22,23
  /** @note (Important!!)
   * In the ESP32, the default SCA and SCL pins are 21 and 22 but on out board those
   * are 22 and 23. so we need to configure this or else the I2C bus wont work
   * here is a links with general explanations:
   *
   * 1. https://randomnerdtutorials.com/esp32-i2c-communication-arduino-ide/
   * 2. https://randomnerdtutorials.com/esp32-pinout-reference-gpios/
   */
  Wire.begin();
  // begin accelerometer operation
  bool success = accel.begin();
  // set range to +-16g (+-156.9 m/s)
  accel.setRange(ADXL345_RANGE_16_G);
  // get and print sensor data on serial
  sensor_t sensor_data;
  accel.getSensor(&sensor_data);

  Serial.println("//////////////// accel setup ////////////////");
  if (success)
  {
    Serial.println("success!");
  }
  else
  {
    Serial.println("failed!");
  }
  return success;
}