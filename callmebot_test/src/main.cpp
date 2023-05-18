#include <Arduino.h>
#include "UrlEncode.h"
#include "HTTPClient.h"

///////////  wifi variables ///////////

//// WR wifi
// const char *ssid = "liornet_WR_2.4_Ghz";
// const char *password = "0544988409";

//// home wifi
// const char *ssid = "liornet";
// const char *password = "0544988409";

// phone wifi
const char *ssid = "Galaxy A53 5G 4C72";
const char *password = "otkm7250";

/////////// callmebot variables ///////////
String phoneNumber = "+972528854006";
String apiKey = "7335050";
String messsage = "Here's the location you requested: https://www.google.com/maps/search/?api=1&query=32.095368,34.769905";

// put function declarations here:
int myFunction(int, int);
inline bool wifi_setup(void);
void exception_handler(char *msg);
void whatsappMessage(String phoneNumber, String apiKey, String message);

void setup()
{
  Serial.begin(9600);
  // put your setup code here, to run once:
  int result = myFunction(2, 3);
  wifi_setup();
}

void loop()
{
  Serial.println("sending message");
  // whatsappMessage(phoneNumber, apiKey, messsage);
  delay(5000);
  // put your main code here, to run repeatedly:
}

// put function definitions here:
int myFunction(int x, int y)
{
  return x + y;
}

/**
 * CallMeBot WhatsApp Messages
 * @param phoneNumber Indonesia +62, Example: "+62897461238"
 * @param apiKey "apiKey"
 * @param message "textmessage"
 * @returns apiKey : https://www.callmebot.com/blog/free-api-whatsapp-messages/.
 */
void whatsappMessage(String phoneNumber, String apiKey, String message)
{
  // Data to send with HTTP POST
  String url = "http://api.callmebot.com/whatsapp.php?phone=" + phoneNumber + "&apikey=" + apiKey + "&text=" + urlEncode(message);
  WiFiClient client;
  HTTPClient http;
  http.begin(client, url);

  // Specify content-type header
  http.addHeader("Content-Type", "application/x-www-form-urlencoded");

  // Send HTTP POST request
  int httpResponseCode = http.POST(url);
  if (httpResponseCode == 200)
  {
    Serial.print("Message sent successfully");
  }
  else
  {
    Serial.println("Error sending the message");
    Serial.print("HTTP response code: ");
    Serial.println(httpResponseCode);
  }

  // Free resources
  http.end();
}

inline bool wifi_setup(void)
{
  // establishing wifi connection
  WiFi.begin(ssid, password);
  Serial.print("WiFi setup\n\n");
  Serial.printf("Connecting to: \n\n%s", ssid);

  int count = 0;

  while (WiFi.status() != WL_CONNECTED)
  {
    delay(1000);
    Serial.print(".");
    count++;
    if (count >= 20)
    {
      exception_handler("can't connect to Wifi");
    }
  }
  Serial.println("WiFi connected ");
  Serial.print("Connected to WiFi network with IP address: \n");
  Serial.println(WiFi.localIP());
  Serial.println();

  delay(1000);

  return true;
}

void exception_handler(char *msg)
{
  Serial.printf("Exception!\n");
  Serial.printf("\n\nreset controller to \ncontinue!");
  delay(20000);

  while (1)
  {
    // do nothing
  }
}