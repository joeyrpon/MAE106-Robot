#include <Servo.h>
#include <Wire.h>

#include <LIS3MDL.h>
#include <LSM6.h>

LIS3MDL mag;
LSM6 imu;

LIS3MDL::vector<int16_t> m_min = {-5145,  +4738,  +562};
LIS3MDL::vector<int16_t> m_max = {-4976,  +4845,  +722};

Servo myservo;

int servoPin = 3;       // Pin that the servomotor is connected to
int solenoidPin = 2;    // Pin that the mosfet is conected to
int switchPin = 4;      // Pin that the switch is conected to
int pos = 0;            // variable to store the servo position
int switchState;        // variable that stores the Reed switch state
int servoDir = 0;       // variable that stores the direction the motor is turning in the demo program
int solenoidState = LOW;  // variable that stores if solenoid is on or off         
unsigned long previousMillis = 0;        // will store last time solenoid was updated
const long interval = 1000;           // interval at which to turn solenoid on and off (milliseconds)

float filteredServoValue = 0;
float filteredHeadingValue = 0;
int maxAngle = 25;


int reedCount = 0;
int reedOnBool = 0;

// Inital piston on/off time
long intervalPistonOff = 700;
long intervalPistonOn = 300;

// boolean to shortcircuit the reedcounter until next rotation
int pistonState = 0;

// diameter of wheel
// diameter D = 68 mm
float reedDistance = 0.2135; // meters
unsigned long reedTime;
float velocityInterval = 0;
bool velocityThresholdBool = false;

// piston on/off time will change after reaching this velocity
float velocityThreshold = 0.3; // m/s

 


void setup() {
  myservo.attach(servoPin);               // attaches the servo on pin 9 to the servo object
  pinMode(solenoidPin, OUTPUT);           //Sets the pin as an output
  pinMode(switchPin, INPUT_PULLUP);       //Sets the pin as an input_pullup
  Serial.begin(9600);                     // starts serial communication @ 9600 bps
  Wire.begin();

  if (!mag.init())
  {
    Serial.println("Failed to detect and initialize LIS3MDL magnetometer!");
    while (1);
  }
  mag.enableDefault();

  if (!imu.init())
  {
    Serial.println("Failed to detect and initialize LSM6 IMU!");
    while (1);
  }
  imu.enableDefault();



}
  


void loop() {

////////////// SERVOMOTOR ///////////////////////////////////////////////////

  /*if(servoDir == 0)
  {
    pos++;
    if(pos >= 180)
    {
      servoDir = 1;
    }
  }
  else
  {
    pos--;
    if(pos <= 0)
    {
      servoDir = 0;
    }
  }*/

 

  

////////////// MAGNETOMETER ///////////////////////////////////////////////////

  mag.read();
  imu.read();

  float heading = computeHeading();

  float unfilteredHeadingValue = heading;

  // digital low pass filter
  float gamma = 0.75;
  filteredHeadingValue = (1 - gamma) * unfilteredHeadingValue + gamma*filteredServoValue;

  // direction adjustment
  float headingAdjustment = 0;
  heading = heading - headingAdjustment;
  if (heading > 360) {
    heading = heading - 360;
  }
  else if (heading < 0 ) {
    heading = heading + 360;
  }

  //Serial.println(heading);

  

  // 1.3 is the angle proportion
  float servoAngle = (180 - abs(180 - filteredHeadingValue));//1.3;

  

  //Serial.println(filteredHeadingValue);
  //Serial.println(servoAngle);
  /*if (90 -filteredServoValue > maxAngle) {
    filteredServoValue = 90 - maxAngle;
    
  }
  else if (90 - filteredServoValue < -maxAngle) {
    filteredServoValue = 90 + maxAngle;
    
  }*/
  
  

  
  myservo.write(90);              // tell servo to go to position in variable 'pos'
  delay(10); 



////////////// SOLENOID VALVE ///////////////////////////////////////////////////
unsigned long currentMillis = millis();


  /*if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;
    if (solenoidState == LOW) {
      solenoidState = HIGH;
    } else {
      solenoidState = LOW;
    }
    digitalWrite(solenoidPin, solenoidState);    //Switch Solenoid ON/oFF
  }*/


  //piston on
  if (currentMillis - previousMillis >= intervalPistonOff && pistonState == 0) {
    previousMillis = currentMillis;
    solenoidState = HIGH;
    digitalWrite(solenoidPin, solenoidState);
    pistonState = 1;
    //Serial.print("Piston state: ");
    //Serial.println(solenoidState);
  }

  //piston off
  else if (currentMillis - previousMillis >= intervalPistonOn && pistonState == 1){
    previousMillis = currentMillis;
    solenoidState = LOW;
    digitalWrite(solenoidPin, solenoidState);
    pistonState = 0;
    //Serial.print("Piston state: ");
    //Serial.println(solenoidState);
  }

////////////// REED SWITCH ///////////////////////////////////////////////////
  switchState = digitalRead(switchPin);

////////////// Serial Print  ///////////////////////////////////////////////////
  //Serial.print("Reed Switch: ");
  //Serial.println(switchState);
  //Serial.print("   Magnetometer: ");
  //Serial.println(heading);
  


  

  if (switchState == 0) {
    
    if (reedOnBool == 1) {
      ++reedCount;
      reedOnBool = 0;
      Serial.print("Reed count: ");
      Serial.println(reedCount);

      unsigned long reedInterval = currentMillis - reedTime;
      reedTime = millis();
      Serial.print("Reed interval (millisec): ");
      Serial.println(reedInterval);
      Serial.print("Velocity interval (m/s): ");
      velocityInterval = (reedDistance / reedInterval) * 1000;
      Serial.println(velocityInterval);

    }
    
    
  }
  if (velocityInterval > velocityThreshold && !velocityThresholdBool){


    // CHANGE THESE Piston on/off time after threshold//////// 

    intervalPistonOff = 800;
    intervalPistonOn = 200;

    ////////////////////////


    Serial.println("Passed velocity threshold");
    velocityThresholdBool = true;
  }

  if (switchState == 1){
    reedOnBool = 1;
  }

  //delay(100);
   /*Serial.print(currentMillis);
   Serial.print(" ");
   Serial.print(solenoidState);
   Serial.print(" ");
   Serial.print(heading);
   Serial.println("");*/

}




//heading function for magnetometer

template <typename T> float computeHeading(LIS3MDL::vector<T> from)
{
  LIS3MDL::vector<int32_t> temp_m = {mag.m.x, mag.m.y, mag.m.z};

  // copy acceleration readings from LSM6::vector into an LIS3MDL::vector
  LIS3MDL::vector<int16_t> a = {imu.a.x, imu.a.y, imu.a.z};

  // subtract offset (average of min and max) from magnetometer readings
  temp_m.x -= ((int32_t)m_min.x + m_max.x) / 2;
  temp_m.y -= ((int32_t)m_min.y + m_max.y) / 2;
  temp_m.z -= ((int32_t)m_min.z + m_max.z) / 2;

  // compute E and N
  LIS3MDL::vector<float> E;
  LIS3MDL::vector<float> N;
  LIS3MDL::vector_cross(&temp_m, &a, &E);
  LIS3MDL::vector_normalize(&E);
  LIS3MDL::vector_cross(&a, &E, &N);
  LIS3MDL::vector_normalize(&N);

  // compute heading
  float heading = atan2(LIS3MDL::vector_dot(&E, &from), LIS3MDL::vector_dot(&N, &from)) * 180 / PI;
  if (heading < 0) heading += 360;
  return heading;
}

/*
Returns the angular difference in the horizontal plane between a
default vector (the +X axis) and north, in degrees.
*/
float computeHeading()
{
  return computeHeading((LIS3MDL::vector<int>){1, 0, 0});
}
