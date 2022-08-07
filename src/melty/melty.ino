#include "WiFi.h" // ESP32 WiFi include
#include <TinyPICO.h>
#include "arduino_secrets.h"

const uint16_t port = 8090;
const char * host = "192.168.86.46";

TinyPICO tp = TinyPICO();

void ConnectToWiFi()
{
 
  WiFi.mode(WIFI_STA);
  WiFi.begin(SECRET_SSID, SECRET_PASS);
  Serial.print("Connecting to "); Serial.println(SECRET_SSID);
 
  uint8_t i = 0;
  while (WiFi.status() != WL_CONNECTED)
  {
    Serial.print('.');
    delay(500);
 
    if ((++i % 16) == 0)
    {
      Serial.println(F(" still trying to connect"));
    }
  }
 
  Serial.print(F("Connected. My IP address is: "));
  Serial.println(WiFi.localIP());
}

void setup()
{
    // Used for debug output only
    Serial.begin(115200);


    ConnectToWiFi();
}

void loop()
{
    WiFiClient client;
    
 
    if (!client.connect(host, port)) {
 
        Serial.println("Connection to host failed");
 
        delay(1000);
        return;
    }
 
    Serial.println("Connected to server successful!");
 
    client.print("Hello from ESP32!");
    client.print(tp.GetBatteryVoltage());
 
    Serial.println("Disconnecting...");
    client.stop();
 
    delay(10000);
}
