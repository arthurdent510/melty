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
#define ADJUSTMENT_FACTOR 3.2

#define CONNECT_TO_WIFI false
#define WRITE_TO_SERIAL false

const uint16_t port = 9999;
const char * host = "192.168.86.69";

SimpleKalmanFilter simpleKalmanFilter(.01, .01, 0.01);
Adafruit_ADXL375 accel = Adafruit_ADXL375(12345);
TinyPICO tp = TinyPICO();
int oldTime;
float currentAngle;
WiFiClient client;
float nextRotation;
long lastRotation;
long lastLed;

// Serial output refresh time
const long SERIAL_REFRESH_TIME = 10;
long refresh_time;

void ConnectToWiFi()
{
 
  WiFi.mode(WIFI_STA);
  WiFi.begin(SECRET_SSID, SECRET_PASS);
  if(WRITE_TO_SERIAL) {  Serial.print("Connecting to "); Serial.println(SECRET_SSID); }
 
  uint8_t i = 0;
  while (WiFi.status() != WL_CONNECTED)
  {
    if(WRITE_TO_SERIAL) { Serial.print('.'); }
    delay(500);
 
    if ((++i % 16) == 0)
    {
      if(WRITE_TO_SERIAL) { Serial.println(F(" still trying to connect")); }
    }
  }
 
  if(WRITE_TO_SERIAL) { Serial.print(F("Connected. My IP address is: ")); }
  if(WRITE_TO_SERIAL) { Serial.println(WiFi.localIP()); }
}

void setup() {
  if(WRITE_TO_SERIAL) { Serial.begin(115200); }

  if(CONNECT_TO_WIFI)
  {
    ConnectToWiFi();

    if (!client.connect(host, port)) {
  
          if(WRITE_TO_SERIAL) { Serial.println("Connection to host failed"); }
  
          delay(1000);
          return;
      }
    }

  if(!accel.begin())
  {
    /* There was a problem detecting the ADXL375 ... check your connections */
    if(WRITE_TO_SERIAL) { Serial.println("Ooops, no ADXL375 detected ... Check your wiring!"); }
    while(1);
  }

  accel.setDataRate(ADXL343_DATARATE_800_HZ); 
  
  if(WRITE_TO_SERIAL) { 
    accel.printSensorDetails(); 
    Serial.println("");
  }
  
  oldTime = millis();
  currentAngle = 0;
  nextRotation = millis();
  lastLed - millis();
  lastRotation = millis();
  tp.DotStar_SetPixelColor( 255, 0, 0 );
}

void loop() {  
  float rotateSpeed = currentRotationSpeed();
  //float elasped = (millis() - oldTime);

  if(rotateSpeed > 1) {
    calcuateBySpeed(rotateSpeed); 
  }
}

void calcuateBySpeed(float rotationSpeed) {
  // calculate how long it'll take to do one rotation
  float timeToRotate = 0;
  if(rotationSpeed != 0) {
    timeToRotate = 360/rotationSpeed; // this takes X deg/ms and figures out how many ms it takes to do 360degs
  }

  // check if we've hit our rotation mark
  long currentTime = millis();
  if(currentTime >= nextRotation) {
    // update next rotate based on current rpm
    nextRotation = nextRotation + timeToRotate; // bump target by current revolution time    
  }

  if(CONNECT_TO_WIFI) {
      client.print(currentTime);
      client.print("\t");
      client.print(nextRotation);
      client.print("\t");
      client.print(rotationSpeed);
      client.print("\t");
      client.print(timeToRotate);
      client.print("|");
    }  

  // light the led if we're close to the rotation mark
  if(abs(currentTime-nextRotation) < 10) {
    tp.DotStar_SetPixelColor( 0, 255, 255 );
    if(CONNECT_TO_WIFI) {
      client.print("Lighting led");
      client.print("\t");
      client.print(currentTime);
      client.print("\t");
      client.print(lastLed);
      client.print("\t");
      client.print(timeToRotate);
      client.print("\t");
      client.print(nextRotation);
      client.print("|");

      lastLed = currentTime;
    }
  }
  else {
    tp.DotStar_SetPixelColor( 0, 0, 0 );
  }
}

float currentRotationSpeed() {
  sensors_event_t event;
  accel.getEvent(&event);
  
  // calculate the estimated value with Kalman Filter
  float estimated_value = simpleKalmanFilter.updateEstimate(event.acceleration.y);
  //float estimated_value = event.acceleration.y;

  int adjustmentFactor = 3;
  double calculatedRPM = sqrt(estimated_value / ((CHIP_OFFSET) * 1.118)) * 100 * ADJUSTMENT_FACTOR;  

  if(isnan(calculatedRPM)) {
    calculatedRPM = 0;
  }

  // convert rpm to deg/sec
  float degreesPerSecond = calculatedRPM * 360/60;
  float degreesPerMS = degreesPerSecond / 1000;// return as degress per millisecond

  if(CONNECT_TO_WIFI) {
    if(calculatedRPM > 150) {
      client.print(calculatedRPM);
      client.print("\t");
      client.print(degreesPerSecond);
      client.print("\t");
      client.print(degreesPerMS);
      client.print("|");
    }
  }

  return degreesPerMS;
}

void calculateByPosition(float rotationSpeed, float elasped) {
  float degreesMoved = rotationSpeed * elasped;
  currentAngle = currentAngle + degreesMoved;

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
    // client.print(estimated_value);
    // client.print("\t");
    client.print(rotationSpeed);
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