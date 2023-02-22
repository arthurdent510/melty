#include <SimpleKalmanFilter.h>
#include <TinyPICO.h>
#include <Adafruit_ADXL375.h>
#include <ESP32Servo.h>
#include <BluetoothSerial.h>
#include <ArduinoJson.h>
#include "arduino_secrets.h"


#define ADXL375_SCK 13
#define ADXL375_MISO 12
#define ADXL375_MOSI 11
#define CHIP_OFFSET 50 
#define ADJUSTMENT_FACTOR 3.2
#define X_MOTOR_PIN 14
#define Y_MOTOR_PIN 15

#define CONNECT_TO_WIFI false
#define WRITE_TO_SERIAL false

const uint16_t port = 9999;
const char * host = "192.168.86.69";

SimpleKalmanFilter simpleKalmanFilter(.01, .01, 0.01);
Adafruit_ADXL375 accel = Adafruit_ADXL375(12345);
TinyPICO tp = TinyPICO();
int oldTime;
float currentAngle;
float nextRotation;
long lastRotation;
long lastLed;
BluetoothSerial SerialBT;
long lastCommand;
String command;
StaticJsonDocument<200> doc;
Servo xEsc;
Servo yEsc;
int x = 90;
int y = 90;
int spinney = 0;
int failSafeTimeout = 1000;


// Serial output refresh time
const long SERIAL_REFRESH_TIME = 10;
long refresh_time;

void setup() {
  SerialBT.begin("PGGB");
  
  if(WRITE_TO_SERIAL) { Serial.begin(115200); }

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

  xEsc.attach(X_MOTOR_PIN, 1000,2000);
  yEsc.attach(Y_MOTOR_PIN, 1000,2000);

  xEsc.write(90);
  yEsc.write(90);
  delay(500);
}

void loop() {  
  getCommand();  

  float rotateSpeed = currentRotationSpeed();
  //float elasped = (millis() - oldTime);

  if(rotateSpeed > 1) {
    calcuateBySpeed(rotateSpeed); 
  }

  // do translation stuff?

  updateMotors();
}

void updateMotors() {
  if(lastCommand+failSafeTimeout < millis()) {
    // stop the bot if we haven't gotten a command in a while, something probably blew up and we need to stop
    x = 90;
    y = 90;
    spinney = 0;
    
    if(WRITE_TO_SERIAL) { 
      Serial.println("hit failsafe");
    }
  }

  if(WRITE_TO_SERIAL) { 
    Serial.print(x);
    Serial.print(":");
    Serial.println(y);
  }

  xEsc.write(x);
  yEsc.write(y);
}

void getCommand() {
  while(SerialBT.available()){
    command = SerialBT.readStringUntil('}');
    command.concat("}");

    DeserializationError error = deserializeJson(doc, command);

    // Test if parsing succeeds.
    if (error) {
      Serial.print(F("deserializeJson() failed: "));
      Serial.println(error.f_str());
      
      return;
    }

    x = doc["X"];
    y = doc["Y"];
    spinney = doc["Spinney"];

    if(WRITE_TO_SERIAL) {
      Serial.print("x:");
      Serial.print(x);
      Serial.print(", y:");
      Serial.print(y);
      Serial.print(", spinney:");
      Serial.println(spinney);
    }

    command = "";
    lastCommand = millis();
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
  // light the led if we're close to the rotation mark
  if(abs(currentTime-nextRotation) < 10) {
    tp.DotStar_SetPixelColor( 0, 255, 255 );
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