#include <SimpleKalmanFilter.h>
#include <Adafruit_LSM6DSO32.h>

/*
 SimpleKalmanFilter(e_mea, e_est, q);
 e_mea: Measurement Uncertainty 
 e_est: Estimation Uncertainty 
 q: Process Noise
 */

SimpleKalmanFilter simpleKalmanFilter(.01, .01, 0.01);
Adafruit_LSM6DSO32 dso32;
int oldTime;
float currentAngle;

// Serial output refresh time
const long SERIAL_REFRESH_TIME = 100;
long refresh_time;

void setup() {
  Serial.begin(115200);

  if (!dso32.begin_I2C()) {
    while (1) {
      delay(10);
    }
  }

  dso32.setAccelRange(LSM6DSO32_ACCEL_RANGE_8_G);
  dso32.setGyroRange(LSM6DS_GYRO_RANGE_2000_DPS);

  Serial.print("Gyro range set to: ");
  switch (dso32.getGyroRange()) {
  case LSM6DS_GYRO_RANGE_125_DPS:
    Serial.println("125 degrees/s");
    break;
  case LSM6DS_GYRO_RANGE_250_DPS:
    Serial.println("250 degrees/s");
    break;
  case LSM6DS_GYRO_RANGE_500_DPS:
    Serial.println("500 degrees/s");
    break;
  case LSM6DS_GYRO_RANGE_1000_DPS:
    Serial.println("1000 degrees/s");
    break;
  case LSM6DS_GYRO_RANGE_2000_DPS:
    Serial.println("2000 degrees/s");
    break;
  case ISM330DHCX_GYRO_RANGE_4000_DPS:
    break; // unsupported range for the DSO32
  }  
  oldTime = millis();
  currentAngle = 0;
}

void loop() {


  sensors_event_t accel;
  sensors_event_t gyro;
  sensors_event_t temp;
  dso32.getEvent(&accel, &gyro, &temp);  // read a reference value from A0 and map it from 0 to 100
  float gyroz = gyro.gyro.z * RAD_TO_DEG;
  float elaspedS = (millis() - oldTime);
  

  

  // calculate the estimated value with Kalman Filter
  float estimated_value = simpleKalmanFilter.updateEstimate(gyroz);
  float estimatedMS = estimated_value / 1000;

  float foo = estimatedMS * elaspedS;
  currentAngle = currentAngle + foo;
  // Serial.print("Elasped: ");
  // Serial.print(elaspedS);
  // Serial.print(", foo: ");
  //Serial.print(foo);
  //Serial.print(", currentAngle: ");
  //Serial.println(currentAngle);


  oldTime = millis();

  // send to Serial output every 100ms
  // use the Serial Ploter for a good visualization
  if (millis() > refresh_time) { 
    Serial.print(currentAngle,4);
    Serial.print("deg,");
    Serial.print(estimatedMS,4);
    Serial.println("deg/ms");
    
    refresh_time = millis() + SERIAL_REFRESH_TIME;
  }

}