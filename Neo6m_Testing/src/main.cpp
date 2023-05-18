#include <Arduino.h>
#include <SoftwareSerial.h>

#define GPS_RX_PIN 32
#define GPS_TX_PIN 33

// put function declarations here:
int myFunction(int, int);

SoftwareSerial gpsSerial(GPIO_NUM_32, GPIO_NUM_33);
// SoftwareSerial sserial(rx, tx);
int counter = 0;

void setup()
{
  // put your setup code here, to run once:
  Serial.begin(9600);
  gpsSerial.begin(9600);
  pinMode(13, OUTPUT);

  for (int i = 0; i < 20; i++)
  {
    digitalWrite(13, !digitalRead(13));
    delay(100);
  }
}

void loop()
{

  while (gpsSerial.available())
  {
    Serial.print((char)gpsSerial.read());
  }
  Serial.print("   done!  " + String(counter++));

  delay(1000);
  // gpsSerial.print("hello from gps");
  digitalWrite(13, !digitalRead(13));
  Serial.println();
  // put your main code here, to run repeatedly:
}

// put function definitions here:
int myFunction(int x, int y)
{
  return x + y;
}