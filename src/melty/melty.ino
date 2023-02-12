#include <SimpleKalmanFilter.h>
//#include <Adafruit_LSM6DSO32.h>
#include <TinyPICO.h>
#include <Adafruit_ADXL375.h>
#include "WiFi.h" // ESP32 WiFi include
#include "arduino_secrets.h"

#define ADXL375_SCK 13
#define ADXL375_MISO 12
#define ADXL375_MOSI 11
#define CHIP_OFFSET 50 
#define ADJUSTMENT_FACTOR 3.1

#define CONNECT_TO_WIFI false

const uint16_t port = 9999;
const char * host = "192.168.86.69";

/*
 SimpleKalmanFilter(e_mea, e_est, q);
 e_mea: Measurement Uncertainty 
 e_est: Estimation Uncertainty 
 q: Process Noise
 */

SimpleKalmanFilter simpleKalmanFilter(.01, .01, 0.01);
Adafruit_ADXL375 accel = Adafruit_ADXL375(12345);
TinyPICO tp = TinyPICO();
int oldTime;
float currentAngle;
WiFiClient client;

// Serial output refresh time
const long SERIAL_REFRESH_TIME = 10;
long refresh_time;

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

void setup() {
  Serial.begin(115200);

  if(CONNECT_TO_WIFI)
  {
    ConnectToWiFi();

    if (!client.connect(host, port)) {
  
          Serial.println("Connection to host failed");
  
          delay(1000);
          return;
      }
    }

  if(!accel.begin())
  {
    /* There was a problem detecting the ADXL375 ... check your connections */
    Serial.println("Ooops, no ADXL375 detected ... Check your wiring!");
    while(1);
  }

  accel.setDataRate(ADXL343_DATARATE_800_HZ); 
  
  accel.printSensorDetails();
  
  Serial.println("");

  oldTime = millis();
  currentAngle = 0;
  tp.DotStar_SetPixelColor( 255, 0, 0 );
}

void loop() {  
  sensors_event_t event;
  accel.getEvent(&event);

  float elaspedS = (millis() - oldTime);
  
  // calculate the estimated value with Kalman Filter
  float estimated_value = simpleKalmanFilter.updateEstimate(event.acceleration.y);

  int adjustmentFactor = 3;
  double calculatedRPM = sqrt(estimated_value / ((CHIP_OFFSET) * 1.118)) * 100 * ADJUSTMENT_FACTOR;  

  if(isnan(calculatedRPM))
  {
    calculatedRPM = 0;
  }

  // convert rpm to deg/sec
  double degreesPerSecond = calculatedRPM * 360/60;

  float estimatedMS = degreesPerSecond / 1000;

  if(estimatedMS < .0005 && estimatedMS > -.0005) {
    estimatedMS = 0;
  }
  
  float foo = estimatedMS * elaspedS;
  currentAngle = currentAngle + foo;

  if(currentAngle < 0)
  {
    currentAngle = currentAngle + 360;
  }

  if(currentAngle > 360) {
    currentAngle = currentAngle - 360;
  }

  if(CONNECT_TO_WIFI)
  {
    client.print(currentAngle);
    client.print("\t");
    client.print(estimated_value);
    client.print("\t");
    client.print(calculatedRPM);
    client.print("|");
  }  

  oldTime = millis();

  if(currentAngle < 10 && currentAngle > -350) 
  {
    tp.DotStar_SetPixelColor( 0, 255, 255 );
  }
  else 
  {
    tp.DotStar_SetPixelColor( 255, 0, 0 );
  }
}