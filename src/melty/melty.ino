#include <SimpleKalmanFilter.h>
#include <TinyPICO.h>
#include <Adafruit_ADXL375.h>
#include <ESP32Servo.h>
#include <BluetoothSerial.h>
#include <ArduinoJson.h>
#include "arduino_secrets.h"

#define CHIP_OFFSET 50 
#define ADJUSTMENT_FACTOR 3.2
#define X_MOTOR_PIN 14
#define Y_MOTOR_PIN 15
#define WRITE_TO_SERIAL false

TaskHandle_t currentRotationTaskHandler;
TaskHandle_t ledTaskHandler;
TaskHandle_t calculateRotationSpeedHandler;

SimpleKalmanFilter simpleKalmanFilter(.01, .01, 0.01);

TinyPICO tp = TinyPICO();
BluetoothSerial SerialBT;

Servo xEsc;
Servo yEsc;

float nextRotation;
long lastCommand;
String command;
StaticJsonDocument<200> doc;

int x = 90;
int y = 90;
int spinney = 0;
int failSafeTimeout = 1000;
float currentRotationSpeed = 0;

void setup() {
  SerialBT.begin("PGGB");
  Serial.begin(115200);
  
  if(WRITE_TO_SERIAL) { Serial.begin(115200); }  
  
  nextRotation = millis();
  tp.DotStar_SetPixelColor( 255, 0, 0 );

  xEsc.attach(X_MOTOR_PIN, 1000,2000);
  yEsc.attach(Y_MOTOR_PIN, 1000,2000);

  xEsc.write(90);
  yEsc.write(90);
  delay(500);

  xTaskCreatePinnedToCore(
                    currentRotationTaskHandlerCode,   /* Task function. */
                    "currentRotationTaskHandler",     /* name of task. */
                    10000,       /* Stack size of task */
                    NULL,        /* parameter of the task */
                    1,           /* priority of the task */
                    &currentRotationTaskHandler,      /* Task handle to keep track of created task */
                    0);          /* pin task to core 0 */ 

  xTaskCreatePinnedToCore(
                    ledTaskHandlerCode,   /* Task function. */
                    "ledTaskHandler",     /* name of task. */
                    10000,       /* Stack size of task */
                    NULL,        /* parameter of the task */
                    1,           /* priority of the task */
                    &ledTaskHandler,      /* Task handle to keep track of created task */
                    1);          /* pin task to core 1 */

  xTaskCreatePinnedToCore(
                    calcuateBySpeed,   /* Task function. */
                    "calculateRotationSpeed",     /* name of task. */
                    10000,       /* Stack size of task */
                    NULL,        /* parameter of the task */
                    0,           /* priority of the task */
                    &calculateRotationSpeedHandler,      /* Task handle to keep track of created task */
                    1);          /* pin task to core 1 */
}

void loop() {  
  getCommand();  

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

void calcuateBySpeed(void * pvParameters) {
  for(;;) {    
    // calculate how long it'll take to do one rotation
    float timeToRotate = 0;
    if(currentRotationSpeed != 0) {
      timeToRotate = 360/currentRotationSpeed; // this takes X deg/ms and figures out how many ms it takes to do 360degs
    }

    // check if we've hit our rotation mark
    long currentTime = millis();
    if(currentTime >= nextRotation) {
      // update next rotate based on current rpm
      nextRotation = nextRotation + timeToRotate; // bump target by current revolution time    
    }
  }
}

void currentRotationTaskHandlerCode(void * pvParameters) {
  Adafruit_ADXL375 accel = Adafruit_ADXL375(12345);

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

  for(;;) {
    sensors_event_t event;
    accel.getEvent(&event);
    
    // calculate the estimated value with Kalman Filter
    float estimated_value = simpleKalmanFilter.updateEstimate(event.acceleration.y);

    int adjustmentFactor = 3;
    double calculatedRPM = sqrt(estimated_value / ((CHIP_OFFSET) * 1.118)) * 100 * ADJUSTMENT_FACTOR;  

    Serial.println(calculatedRPM);
    bluetoothPrintLine(String(calculatedRPM, 3));  

    if(isnan(calculatedRPM)) {
      calculatedRPM = 0;
    }

    // convert rpm to deg/sec
    float degreesPerSecond = calculatedRPM * 360/60;
    currentRotationSpeed = degreesPerSecond / 1000;// return as degress per millisecond  
  }
}

void ledTaskHandlerCode(void * pvParameters) {
  for(;;) {
    if(abs(millis()-nextRotation) < 10) {
      tp.DotStar_SetPixelColor( 0, 255, 255 );
    }
    else {
      tp.DotStar_SetPixelColor( 0, 0, 0 );
    }
  }
}

void bluetoothPrintLine(String line) {
    unsigned l = line.length();
    for(int i=0; i<l; i++)
    {
        if(line[i]!='\0')
            SerialBT.write(byte(line[i]));
    }
    SerialBT.write(10); // \n
}