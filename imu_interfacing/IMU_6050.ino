// Basic demo for accelerometer readings from Adafruit MPU6050

#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <iostream>
#include <fstream>
using namespace std;


Adafruit_MPU6050 mpu;
unsigned long current_time; //global variable
float offset_x, offset_y, offset_z;

void calibrate(){ //Calibrates IMU

  float calx, caly, calz;

  calx = 0.00;
  caly = 0.00;
  calz = 0.00;
  int samples = 500;


  for(int i = 0; i < samples; i++){

    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp); //read vals

    calx += a.acceleration.x;
    caly += a.acceleration.y;
    calz += a.acceleration.z;

    delay(2);

  }

  offset_x = calx/samples; //find avg
  offset_y = caly/samples;
  offset_z = calz/samples;
  /*
  Serial.println(offset_x);
  delay(5000);
  Serial.println(offset_y);
  delay(5000);
  Serial.println(offset_z);
  delay(5000);
  */

  Serial.println("Done Calibrating");

}

void setup(void) {
  Serial.begin(115200);
  while (!Serial)
    delay(10); // will pause Zero, Leonardo, etc until serial console opens

  Serial.println("Adafruit MPU6050 test!");

  // Try to initialize!
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!");

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  Serial.print("Accelerometer range set to: ");
  switch (mpu.getAccelerometerRange()) {
  case MPU6050_RANGE_2_G:
    Serial.println("+-2G");
    break;
  case MPU6050_RANGE_4_G:
    Serial.println("+-4G");
    break;
  case MPU6050_RANGE_8_G:
    Serial.println("+-8G");
    break;
  case MPU6050_RANGE_16_G:
    Serial.println("+-16G");
    break;
  }
  mpu.setGyroRange(MPU6050_RANGE_1000_DEG);
  Serial.print("Gyro range set to: ");
  switch (mpu.getGyroRange()) {
  case MPU6050_RANGE_250_DEG:
    Serial.println("+- 250 deg/s");
    break;
  case MPU6050_RANGE_500_DEG:
    Serial.println("+- 500 deg/s");
    break;
  case MPU6050_RANGE_1000_DEG:
    Serial.println("+- 1000 deg/s");
    break;
  case MPU6050_RANGE_2000_DEG:
    Serial.println("+- 2000 deg/s");
    break;
  }

  mpu.setFilterBandwidth(MPU6050_BAND_260_HZ);
  Serial.print("Filter bandwidth set to: ");
  switch (mpu.getFilterBandwidth()) {
  case MPU6050_BAND_260_HZ:
    Serial.println("260 Hz");
    break;
  case MPU6050_BAND_184_HZ:
    Serial.println("184 Hz");
    break;
  case MPU6050_BAND_94_HZ:
    Serial.println("94 Hz");
    break;
  case MPU6050_BAND_44_HZ:
    Serial.println("44 Hz");
    break;
  case MPU6050_BAND_21_HZ:
    Serial.println("21 Hz");
    break;
  case MPU6050_BAND_10_HZ:
    Serial.println("10 Hz");
    break;
  case MPU6050_BAND_5_HZ:
    Serial.println("5 Hz");
    break;
  }

  Serial.println("");
  delay(100);

  current_time = millis();

  calibrate();

}


void loop() {



  /* Get new sensor events with the readings */
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  String x = "Nothing";

  int delay_var = 500;
  int calx, caly, calz;

  /* time step */ 

  unsigned long oldtime = current_time;
  unsigned long dt = (current_time - oldtime)/1000.00; //change to seconds
  current_time = millis();
  //Serial.println(dt);
  
  /* get variables*/

  float acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z;

  acc_x = a.acceleration.x - offset_x; //Store Acceleration for each Axis
  acc_y = a.acceleration.y - offset_y;
  acc_z = a.acceleration.z;

  gyro_x = g.gyro.x; //Store Gyro Values
  gyro_y = g.gyro.y;
  gyro_z = g.gyro.z;

  /* pitch and roll */

  float roll_a, pitch_a;

  roll_a = atan2(acc_x,sqrt((acc_y*acc_y)+(acc_z*acc_z))) * 180/PI; //Roll Calc
  pitch_a = atan2(acc_y,sqrt((acc_x*acc_x)+(acc_z*acc_z))) * 180/PI; //Pitch Calc

  //Serial.println(roll_a);
  //Serial.println(pitch_a);

  if ((gyro_y) < -8){ // Left click

    x = "left click";
    Serial.println(x);
    delay(500);

  }

  if ((gyro_y) > +8){ //Right click

    x = "right click";
    Serial.println(x);
    delay(500);

  }
 
  if ((pitch_a) > 33){ //Scrolling up

    Serial.println("Scrolling up");
  }

  if ((pitch_a) < -33){ //Scrolling down

    Serial.println("Scrolling down");
  }

  //Serial.println(x);
  Serial.println("");
  delay(delay_var);
}

