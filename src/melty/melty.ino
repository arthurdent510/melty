#include <SimpleKalmanFilter.h>
#include <TinyPICO.h>
#include <Adafruit_ADXL375.h>
#include <ESP32Servo.h>
#include <BluetoothSerial.h>
#include <ArduinoJson.h>
//#include "arduino_secrets.h"

#define CHIP_OFFSET 50 
//#define ADJUSTMENT_FACTOR 4.08
#define X_MOTOR_PIN 14
#define Y_MOTOR_PIN 15
#define WRITE_TO_SERIAL false

TaskHandle_t currentRotationTaskHandler;
TaskHandle_t ledTaskHandler;
TaskHandle_t calculateRotationSpeedHandler;
TaskHandle_t getCommandHandler;
TaskHandle_t updateMotorsHandler;

TinyPICO tp = TinyPICO();
BluetoothSerial SerialBT;

Servo xEsc;
Servo yEsc;

float nextRotation;
long lastCommand;


int x = 90;
int y = 90;
int heading = 0;
int magnitude = 0;
int spinney = 0;
int failSafeTimeout = 1000;
float currentRotationSpeed = 0;
double  ADJUSTMENT_FACTOR = 3.19;

Adafruit_ADXL375 accel = Adafruit_ADXL375(12345);
SimpleKalmanFilter simpleKalmanFilter(.01, .01, 0.01);



String command;
StaticJsonDocument<200> doc;

void setup() {
  SerialBT.begin("PGGB");
  Serial.begin(115200);
  
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
  
  if(WRITE_TO_SERIAL) { Serial.begin(115200); }  
  
  nextRotation = millis();
  tp.DotStar_SetPixelColor( 255, 0, 0 );

  //Serial.println("init motors");  

  xEsc.attach(X_MOTOR_PIN, 1000,2000);
  yEsc.attach(Y_MOTOR_PIN, 1000,2000);

  xEsc.write(90);
  yEsc.write(90);
  delay(500);  

  //Serial.println("done with motor init");

  xTaskCreatePinnedToCore(
                    currentRotationTaskHandlerCode,   /* Task function. */
                    "currentRotationTaskHandler",     /* name of task. */
                    10000,       /* Stack size of task */
                    NULL,        /* parameter of the task */
                    2,           /* priority of the task */
                    &currentRotationTaskHandler,      /* Task handle to keep track of created task */
                    0);          /* pin task to core 0 */ 

  delay(500);

  xTaskCreatePinnedToCore(
                    getCommandCode,   /* Task function. */
                    "getCommand",     /* name of task. */
                    10000,       /* Stack size of task */
                    NULL,        /* parameter of the task */
                    3,           /* priority of the task */
                    &getCommandHandler,      /* Task handle to keep track of created task */
                    1);          /* pin task to core 1 */

  delay(500);

  xTaskCreatePinnedToCore(
                    ledTaskHandlerCode,   /* Task function. */
                    "ledTaskHandler",     /* name of task. */
                    10000,       /* Stack size of task */
                    NULL,        /* parameter of the task */
                    3,           /* priority of the task */
                    &ledTaskHandler,      /* Task handle to keep track of created task */
                    0);          /* pin task to core 1 */

  delay(500);

  xTaskCreatePinnedToCore(
                    calcuateBySpeed,   /* Task function. */
                    "calculateRotationSpeed",     /* name of task. */
                    10000,       /* Stack size of task */
                    NULL,        /* parameter of the task */
                    2,           /* priority of the task */
                    &calculateRotationSpeedHandler,      /* Task handle to keep track of created task */
                    0);          /* pin task to core 1 */

  delay(500);

  xTaskCreatePinnedToCore(
                    updateMotorsCode,   /* Task function. */
                    "updateMotors",     /* name of task. */
                    10000,       /* Stack size of task */
                    NULL,        /* parameter of the task */
                    2,           /* priority of the task */
                    &updateMotorsHandler,      /* Task handle to keep track of created task */
                    1);          /* pin task to core 1 */
  
  delay(500);

  //Serial.println("done with setup function");
}

void loop() {   
  while (true){}
}

void updateMotorsCode(void * pvParameters) {
  //Serial.println("bar");
  while(true) {
    //Serial.println("foo");
    
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

    String foo = "motors:";
    foo.concat(x);
    foo.concat(":");
    foo.concat(y);

    //Serial.println(foo);

    if(spinney > 0) {
      String foo = "magnitude: ";
      foo.concat(magnitude);
      bluetoothPrintLine(foo);
      
      // start spin to win!
      float multiplier = 90 * spinney/100;      
      x = 90 + multiplier;
      y = 90 - multiplier;

      // now look to see if we need to translate
      if(magnitude > 0) {
        // check our heading
        int currentHeading = 0;
        
        // current speed in degs/ms, last rotation, from time elasped we should be able to tell where in the rotation we are
        float elasped = nextRotation - millis();
        currentHeading = (int)(elasped*currentRotationSpeed); // rotation speed is deg/ms, so this should give us degrees since our last rotation

        currentHeading = currentHeading % 360;

        // turn off x if we're in the right spot.  We'll want to turn off the motor at +90 degree from our target heading   
        if(abs(heading+90 - currentHeading) < magnitude) {
          x = 90;
        }
        String bar =  "currentHeading:";
        bar.concat(currentHeading);
        bar.concat(", heading: ");
        bar.concat(heading);

        bluetoothPrintLine(bar);        
      }      
    }    
    
    xEsc.write(x);
    yEsc.write(y);

    vTaskDelay(1);
  }
}

void getCommandCode(void * pvParameters) { 
  while(true) {
    /* future note to anyone who actually reads this... I ran into a problem where my app sending bt commands was sending them faster then 
      the tinypico could process them, it was causing the tinypico to constantly crash and restart.  I added a 10 ms delay in my code that was sending
      commands, that seemed to have fixed it.  I think.... 
    */
    //delay(10);

    if(SerialBT.available()) {
      //Serial.println("Serial bt is avaiable");
      command = SerialBT.readStringUntil('}');
      //command = SerialBT.readString();
      command.concat("}");

      //Serial.println(command);

      DeserializationError error = deserializeJson(doc, command);

      // Test if parsing succeeds.
      if (error) {
        Serial.print(F("deserializeJson() failed: "));
        Serial.println(error.f_str());
        
        return;
      }

      x = doc["X"];
      y = doc["Y"];
      heading = doc["Heading"];
      magnitude = doc["Magnitude"];   
      spinney = doc["Spinney"];
      int adjustment = doc["AdjustmentFactor"];

      if(adjustment > 0) {
        ADJUSTMENT_FACTOR += .01;

        String foo = "New adjustment factor:";
        foo.concat(ADJUSTMENT_FACTOR);
        Serial.println(foo);
        bluetoothPrintLine(foo);
      }
      if(adjustment < 0) {
        ADJUSTMENT_FACTOR -= .01;
        
        String foo = "New adjustment factor:";
        foo.concat(ADJUSTMENT_FACTOR);
        Serial.println(foo);
        bluetoothPrintLine(foo);
      }

      String foo = "x:";
      foo.concat(x);
      foo.concat(", y:");
      foo.concat(y);
      foo.concat(", spinney:");
      foo.concat(spinney);

      //Serial.println(foo);

      command = "";
      lastCommand = millis();
    }
    vTaskDelay(1);
  }
}

void calcuateBySpeed(void * pvParameters) {
  while(true) {    
    // calculate how long it'll take to do one rotation
    float timeToRotate = 0;
    if(currentRotationSpeed != 0) {
      timeToRotate = 360/currentRotationSpeed; // this takes X deg/ms and figures out how many ms it takes to do 360degs
    }

    // check if we've hit our rotation mark
    if(millis() >= nextRotation) {
      // update next rotate based on current rpm
      nextRotation = nextRotation + timeToRotate; // bump target by current revolution time    
    }
    vTaskDelay(1);
  }
}

void currentRotationTaskHandlerCode(void * pvParameters) {
  while(true) {
    sensors_event_t event;
    accel.getEvent(&event);
    
    // calculate the estimated value with Kalman Filter
    float estimated_value = simpleKalmanFilter.updateEstimate(event.acceleration.y);

    int adjustmentFactor = 3;
    double calculatedRPM = sqrt(estimated_value / ((CHIP_OFFSET) * 1.118)) * 100 * ADJUSTMENT_FACTOR;  

    //Serial.println(calculatedRPM);
    //bluetoothPrintLine(String(calculatedRPM, 3));  

    if(isnan(calculatedRPM)) {
      calculatedRPM = 0;
    }

    // convert rpm to deg/sec
    float degreesPerSecond = calculatedRPM * 360/60;
    currentRotationSpeed = degreesPerSecond / 1000;// return as degress per millisecond  

    String foo = "";
    foo.concat(currentRotationSpeed);    
    foo.concat(":");
    foo.concat(xPortGetCoreID());

    //Serial.println(foo);
    vTaskDelay(1);
  }
}

void ledTaskHandlerCode(void * pvParameters) {
  while(true) {
    String foo = "led core:";
    foo.concat(xPortGetCoreID());
    //Serial.println(foo);

    if(abs(millis()-nextRotation) < 10) {
      tp.DotStar_SetPixelColor( 0, 255, 255 );
      //Serial.println("lighting led");
    }
    else {
      tp.DotStar_SetPixelColor( 0, 0, 0 );
    }
    vTaskDelay(1);
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