#include <Arduino.h>

#include <Adafruit_ST7735.h>
#include <SPI.h>

#define TFT_CS 4   // Marked as pin A5 on the board
#define TFT_RST 26 // Marked as pin A0 on the board
#define TFT_A0 13  // Marked as pin 13 on the board
#define MOSI_PIN 18
#define MISO_PIN 19
#define SCK_PIN 5

Adafruit_ST7735 tft = Adafruit_ST7735(TFT_CS, TFT_A0, TFT_RST);

// put function declarations here:
int myFunction(int, int);
inline void TFT_setup(void);

void setup()
{
  // put your setup code here, to run once:
  // pinMode(TFT_RST, OUTPUT);
  // digitalWrite(TFT_RST, LOW);
  Serial.begin(115200);
  SPI.begin(SCK_PIN, MISO_PIN, MOSI_PIN);
  TFT_setup();
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

inline void TFT_setup(void)
{
  Serial.println("TFT setup");
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