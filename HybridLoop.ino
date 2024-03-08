#include <Servo.h>
#include <Wire.h>

#include <LIS3MDL.h>
#include <LSM6.h>

LIS3MDL mag;
LSM6 imu;

// calibration parameters  
LIS3MDL::vector<int16_t> m_min = {-5049,  +3350,  +1300};
LIS3MDL::vector<int16_t> m_max = {-4964,  +3429,  +1408};

Servo myservo;

int servoPin = 3;       // Pin that the servomotor is connected to
int solenoidPin = 2;    // Pin that the mosfet is conected to
int switchPin = 4;      // Pin that the switch is conected to
int pos = 0;            // variable to store the servo position
int switchState;        // variable that stores the Reed switch state
int servoDir = 0;       // variable that stores the direction the motor is turning in the demo program
int solenoidState = LOW;  // variable that stores if solenoid is on or off         
unsigned long previousMillis = 0;        // will store last time solenoid was updated

float filteredHeadingValue = 0;
int maxAngle = 25;


int reedCount = 0;
int reedOnBool = 0; // boolean to shortcircuit the reedcounter until next rotation

// Inital piston on/off time
long intervalPistonOff = 700;
long intervalPistonOn = 300;

// Post piston on/off time
long postPistonOff = 800;
long postPistonOn = 200;

// Determines if piston is on or off
int pistonState = 0;

// diameter of wheel = 68 mm
float reedDistance = 0.2135; // distance(meters) the robot travels for each reed click
unsigned long reedTime; // Time between each reed click
float velocityInterval = 0; // velocity(m/s) between each reed click
bool velocityThresholdBool = false; // True when robot has surpassed the velocity threshold

// piston on/off time will change after reaching this velocity
float velocityThreshold = 0.3; // m/s



// Hybrid loop parameters
int c2_n_ticks_before_turn = 5;
float c2_total_time_turning = 3; // seconds
float c2_Kp = 0.5;

unsigned long startTimeTurning;

// State for hybrid
int state = 1;

 


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


float headingAdjustment(float heading, float adjustment) {
  heading = heading - adjustment + 90;
  if (heading > 360) {
    heading = heading - 360;
  }
  else if (heading < 0 ) {
    heading = heading + 360;
  }
  return heading;
}


float belowMax(float servoValue, int maxAngle, float kp) {
  
  if ( (90 -servoValue) * kp > maxAngle) {
    servoValue = 90 - maxAngle;
  }
  else if ( (90 - servoValue) *kp < -maxAngle) {
    servoValue = 90 + maxAngle;
  }
  return servoValue;
}

float getServoAngle(float direction, int maxAngle, float kp) {
  float servoAngle = (180 - abs(180 - direction));
  return belowMax(servoAngle,maxAngle,kp);
}


void loop() {


////////////// MAGNETOMETER ///////////////////////////////////////////////////

  mag.read();
  imu.read();

  float heading = computeHeading();

  float unfilteredHeadingValue = heading;

  // digital low pass filter
  float gamma = 0.9;
  filteredHeadingValue = (1 - gamma) * unfilteredHeadingValue + gamma*filteredHeadingValue;


  float direction = 263; // Direction to drive straight
  float straight = headingAdjustment(filteredHeadingValue,direction);


  ////////////// SERVOMOTOR ///////////////////////////////////////////////////

  // < 90 turns the robot to the right
  // > 90 turns robot to the left
  float servoAngle = getServoAngle(straight,maxAngle,c2_Kp);
  
  
  //myservo.write(servoAngle);
  delay(10); 



////////////// SOLENOID VALVE ///////////////////////////////////////////////////
unsigned long currentMillis = millis();



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
      //Serial.print("Reed interval (millisec): ");
      //Serial.println(reedInterval);
      //Serial.print("Velocity interval (m/s): ");
      velocityInterval = (reedDistance / reedInterval) * 1000;
      //Serial.println(velocityInterval);
    }
  }
  if (switchState == 1){
    reedOnBool = 1;
  }


  if (velocityInterval > velocityThreshold && !velocityThresholdBool){
    intervalPistonOff = postPistonOff;
    intervalPistonOn = postPistonOn;
    Serial.println("Passed velocity threshold");
    velocityThresholdBool = true;
  }

  


  switch (state) {
    
    // straight
    case 1:
      myservo.write(90);
      if (reedCount >= c2_n_ticks_before_turn) {
        state = 2;
        startTimeTurning = millis();
        Serial.println("State 2");
        
      }
      break;
    // max turn
    case 2:
      myservo.write(90 - maxAngle);
      if (currentMillis - startTimeTurning > c2_total_time_turning * 1000){
        state = 3;
        Serial.println("State 3");
      }
      break;

    // feedback control
    case 3:
      myservo.write(servoAngle);
      Serial.println(filteredHeadingValue);
      break;
  }

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
