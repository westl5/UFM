#include <Wire.h>
#include <Adafruit_MPU6050.h> //IMU Header File
#include <Adafruit_DRV2605.h> //Motor Driver
#include <BleMouse.h> //Bluetooth Mouse

// Define sensor objects
Adafruit_MPU6050 mpu;
Adafruit_DRV2605 drv;
BleMouse bleMouse; // Bluetooth Mouse

unsigned long current_time; //global variable
float offset_x, offset_y, offset_z;

void setup() {
  Serial.begin(115200);

  Wire.begin();  // Use default I2C bus (SDA = 21, SCL = 22 for ESP32)

  // Initialize MPU-6050 (IMU)
  if (!mpu.begin(0x68, &Wire)) {  // Default address for MPU-6050 is 0x68
    Serial.println("Failed to find MPU-6050 chip!");
    while (1); // Halt if initialization fails
  }
  Serial.println("MPU-6050 Initialized!");

  // Initialize DRV2605L (Vibration Motor)
  if (!drv.begin()) {  // Default address for DRV2605L is 0x5A
    Serial.println("Failed to find DRV2605L chip!");
    while (1); // Halt if initialization fails
  }
  Serial.println("DRV2605L Initialized!");

  // Configure DRV2605L
  drv.selectLibrary(6); // Select library 1 (basic vibration effects)
  drv.setMode(DRV2605_MODE_INTTRIG); // Internal trigger mode for vibration motor
  
  Serial.begin(115200); //Bluetooth Mouse Initlziation
  Serial.println("Starting BLE work!");
  bleMouse.begin();

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

  //Serial.println(gyro_z);

  /* pitch and roll */

  float roll_a, pitch_a;

  roll_a = atan2(acc_x,sqrt((acc_y*acc_y)+(acc_z*acc_z))) * 180/PI; //Roll Calc
  pitch_a = atan2(acc_y,sqrt((acc_x*acc_x)+(acc_z*acc_z))) * 180/PI; //Pitch Calc

  Serial.println(roll_a);
  Serial.println(pitch_a);

 // Using Rotational velocity for the clicking
 /*
  if ((gyro_y) < -4){ // Left click (-8 was good)

    x = "left click";
    Serial.println(x);
    drv.setWaveform(0, 17); // Effect 17
    drv.setWaveform(1, 0); // End waveform
    drv.go();
    delay(500);

    //drv.setWaveform(0,0); //Stops Vibration
    

  }

  if ((gyro_y) > +4){ //Right click (+8 was good)

    x = "right click";
    Serial.println(x);
    drv.setWaveform(0, 17); // Effect 17 Strong Click 1 - 100 Percent
    drv.setWaveform(1, 0); // End waveform
    drv.go();
    delay(500);
    
    //drv.setWaveform(0,0); //Stops Vibration

  }
  */

 // Using Roll Angle for the clicking
 
  if ((roll_a > 30) && (roll_a < 45)){ // Left click (50 is good)

    x = "left click";
    Serial.println(x);
    bleMouse.click(MOUSE_LEFT);
    drv.setWaveform(0, 17); // Effect 17
    drv.setWaveform(1, 0); // End waveform
    drv.go();
    delay(1000);

    //drv.setWaveform(0,0); //Stops Vibration
    

  }

  if ((roll_a) > +60){ // Double Left click (50 is good)

    x = "double left click";
    Serial.println(x);
    bleMouse.click(MOUSE_LEFT);
    bleMouse.click(MOUSE_LEFT);
    drv.setWaveform(0, 64); // Effect 17
    drv.setWaveform(1, 0); // End waveform
    drv.go();
    delay(1000);

    //drv.setWaveform(0,0); //Stops Vibration
    

  }

  if ( (roll_a < -30) && (roll_a > -45) ){ //Right click (-50 is good)

    x = "right click";
    Serial.println(x);
    bleMouse.click(MOUSE_RIGHT);
    drv.setWaveform(0, 17); // Effect 17 Strong Click 1 - 100 Percent
    drv.setWaveform(1, 0); // End waveform
    drv.go();
    delay(1000);
    
    //drv.setWaveform(0,0); //Stops Vibration

  }

  if ((roll_a) < -60 ){ // Double Right click (-50 is good)

    x = "double right click";
    Serial.println(x);
    bleMouse.click(MOUSE_RIGHT);
    bleMouse.click(MOUSE_RIGHT);
    drv.setWaveform(0, 64); // Effect 17 Strong Click 1 - 100 Percent
    drv.setWaveform(1, 0); // End waveform
    drv.go();
    delay(1000);
    
    //drv.setWaveform(0,0); //Stops Vibration

  }

 
  if ((pitch_a) > 33){ //Scrolling up

    Serial.println("Scrolling up");
    bleMouse.move(0,0,1);
    delay(1);          // Small delay for smoother scrolling
    
  }

  if ((pitch_a) < -33){ //Scrolling down

    Serial.println("Scrolling down");
    bleMouse.move(0,0,-1);
    delay(1);           // Small delay for smoother scrolling
  }

  //Serial.println(x);
  Serial.println("");
  delay(delay_var);
}

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

/* Notes

  Pitch measures the tilt value of tilting the board upwards and downwards.
  Roll measures the tilt value of turning the board leftwards and rightwards.

  gyro_y measures the rototinal velocity when moving the board leftwards and rightwards


*/


