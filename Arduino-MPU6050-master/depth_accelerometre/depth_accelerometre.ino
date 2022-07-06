

#include <MPU6050.h>

/*
    MPU6050 Triple Axis Gyroscope & Accelerometer. Pitch & Roll & Yaw Gyroscope Example.
    Read more: http://www.jarzebski.pl/arduino/czujniki-i-sensory/3-osiowy-zyroskop-i-akcelerometr-mpu6050.html
    GIT: https://github.com/jarzebski/Arduino-MPU6050
    Web: http://www.jarzebski.pl
    (c) 2014 by Korneliusz Jarzebski
*/

#include <Wire.h>


MPU6050 mpu;

// Timers
unsigned long timer = 0;
float timeStep = 0.01;

// Pitch, Roll and Yaw values
float pitch = 0;
float roll = 0;
float yaw = 0;
float yaw_deger=0;
float roll_deger=0;

const float  OffSet = 0.479 ;

float V, P,V_ort,P_ort,V_tot,P_tot,i;
void setup() 
{
  Serial.begin(115200);

  // Initialize MPU6050
  while(!mpu.begin(MPU6050_SCALE_2000DPS, MPU6050_RANGE_2G))
  {
    Serial.println("Could not find a valid MPU6050 sensor, check wiring!");
    delay(500);
  }
  
  // Calibrate gyroscope. The calibration must be at rest.
  // If you don't want calibrate, comment this line.
  mpu.calibrateGyro();

  // Set threshold sensivty. Default 3.
  // If you don't want use threshold, comment this line or set 0.
  mpu.setThreshold(3);
  //normali 3
  V_ort=0.00;
  P_ort=0.00;
  V_tot=0.00;
  P_tot=0.00;
  i=0.00;
}

void loop()
{
  timer = millis();
    //Connect sensor to Analog 0
  V = analogRead(A1) * 5.00 / 1024;     //Sensor output voltage
  P = (V - OffSet) * 400;             //Calculate water pressure
  if (i<5.00){
      i=i+1.00;
      V_tot=V_tot+V;
      P_tot=P_tot+P;
  }
  if(i==5.00){
      V_ort=V_tot/i;
      P_ort=P_tot/i;
      V_tot=0.00;
      P_tot=0.00;          
      i=0.00;
  }
  //Serial.print("Voltage:");
  Serial.print(V_ort, 3);
  Serial.print(" ");
  //Serial.print("V ");

  //Serial.print(" Pressure:");
  Serial.print(P_ort, 2);
  Serial.print(" ");
  //Serial.print(" KPa  ");
  //Serial.println();
  
  // Read normalized values
  Vector norm = mpu.readNormalizeGyro();

  // Calculate Pitch, Roll and Yaw
  pitch = pitch + norm.YAxis * timeStep;
  roll = roll + norm.XAxis * timeStep;
  yaw = yaw + norm.ZAxis * timeStep;

  // Output raw
  // Serial.print(" Pitch = ");
  // Serial.print(pitch);
  //Serial.print(" Pitch = ");
  roll_deger=roll/2;
  yaw_deger=yaw/2;
  
  Serial.print(roll_deger);
  Serial.print(" ");  
  //Serial.print(" Yaw = ");
  Serial.println(yaw_deger);
  
  // Wait to full timeStep period
  delay((timeStep*500) - (millis() - timer));
}
