/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       VEX                                                       */
/*    Created:      Thu Sep 26 2019                                           */
/*    Description:  Competition Template                                      */
/*                                                                            */
/*----------------------------------------------------------------------------*/

// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// Controller1          controller                    
// RMMotor              motor         10              
// RFMotor              motor         9               
// RBMotor              motor         18              
// LMMotor              motor         1               
// LFMotor              motor         2               
// LBMotor              motor         11              
// Intake               motor         8               
// Cata                 motor         15              
// ExpansionPnuematics1 led           A               
// ExpansionPnuematics2 led           B               
// LimitSwitch          limit         C               
// Gyro                 inertial      4               
// Color                optical       17              
// VisionSensor         vision        16              
// CataPistons          led           D               
// EncoderL             encoder       E, F            
// EncoderR             encoder       G, H            
// Expander13           triport       13              
// EncoderB             encoder       A, B            
// ---- END VEXCODE CONFIGURED DEVICES ----

#include "vex.h"
#include "cmath"

using namespace vex;


// A global instance of competition
competition Competition;

//Defining Constants for Odometry
#define PI 3.14159265
const double toRadians = PI / 180; //Converts to Radians
const double toDegrees = 180 / PI; //Converts to Degrees
const double TrackingCircumference = 4 * PI; //Calculates Tracking Wheel Circumference
const double leftOffset = 5; //Subject to change when we add tracking wheels, Distance from Tracking Center to Left Tracking Wheel
const double rightOffset = 5; //Subject to change when we add tracking wheels, Distance from Tracking Center to Right Tracking Wheel
const double backOffset = 5; //Subject to change when we add tracking wheels, Distance from Tracking Center to Back Tracking Wheel



//--------------------------------Variables----------------------------------//
//ODOMETRY Variables
double globalX = 0; //Global X Coordinate of Robot
double globalY = 0; //Global Y Coordinate of Robot

float curLeft = 0; //Current Position of Left Encoder
float curRight = 0; //Current Position of Right Encoder
float curBack = 0; //Current Position of Back Encoder

float prevLeft = 0; //Last Position of Left Encoder
float prevRight = 0; //Last Position of Right Encoder
float prevBack = 0; //Last Position of Back Encoder
float prevGyroRadians; //Last Gyro Position (Radians)

float deltaL = 0; //Change in Left Encoder
float deltaR = 0; //Change in Right Encoder
float deltaB = 0; //Change in Back Encoder

float deltaTheta = 0; //Change in Arc Angle

float deltaX = 0; //Change in X Position
float deltaY = 0; //Change in X Position

float deltaDistance; //Change in dstance moved
float deltaDistanceSide; //Change in Side Distance
double absoluteAngleOfMovement = 0; //Direction Of Movement

float totalDistance = 0; //Tracks Total Distance traveled

double targetX = 0; //Target X-Position
double targetY = 0; //Target Y-Position
double targetDeg = 0; //Target Degree
double targetDistance = 0; //Straightest Path to Target 

int stopTime = 0; //Used for isStopped Function


//Auton Selection Vars
bool redAuton = true;
bool bothSides = true;
bool rollerSide = false;
bool skills = false;

//Function Variables
bool firingCata = false;


//---------------------------------------------------------------------------//
//--------------------------------Functions----------------------------------//
//---------------------------------------------------------------------------//

//--------------------------------Odometry/Math Functions----------------------------------//
double degreesToInches(double ticks){ //Converts Encoder Ticks to Inches Traveled
  return (ticks / 360) * TrackingCircumference;
}

double keepInRange (double n, double low, double high){ //Keeps input value (n) between two defined constraints
  if (n > high){
    n = high;
  }
  if (n < low){
    n = low;
  }
  return n;
}

double distanceTo (double x1, double y1){ //Calculates Distance Between Current Location (GlobalX, GlobalY) and target points (x1, y1)
  return sqrt(pow(globalX - x1, 2) + pow(globalY - y1, 2));
}

double getRightReading(){
  return degreesToInches(EncoderR);
}

double getLeftReading(){
  return degreesToInches(EncoderL);
}

double getBackReading(){
  return degreesToInches(EncoderB);
}

bool isStopped(){ //Checks to see if Robot isn't moving: Makes sure it doens't run into walls (COULD ENCOUNTER PROBLEMS!)

  if (fabs(deltaR) < 0.001 && fabs(deltaL) < 0.001){
    stopTime += 1;
  } else {
    stopTime = 0;
  }
  if (stopTime == 50){
    return true;
  } else {
    return false;
  }

}

double getRobotRotation(){ //Gets robot orientation in degrees
  return -Gyro.orientation(yaw, degrees); //Negative because of weird trig math, left is positive, right is negative
}

double getRobotRadians(){ //Gets robot orientation in radians
  return getRobotRotation() * toRadians;
}

//Changes angle to make sure the shorter distance is traveled, ie if a the angle is more than 180 degrees, turn the other direction so it's faster
double angleWrap(double angle){
  double robotAngle = getRobotRotation();
  while (angle > robotAngle + 180){ 
    angle -= 360;
  }
  while (angle < robotAngle - 180){
    angle += 360;
  }

  return angle; //return new angle
}

double getDegToPosition(double x, double y){ //Find Angle That points to the desired point

  double relativeX = x - globalX;
  double relativeY = y - globalY;

  double degToPosition = toDegrees * atan2(relativeY, relativeX);
  
  degToPosition = angleWrap(degToPosition);

  return degToPosition;

}

//Movement Functions

void leftDrive(double power) {
  LBMotor.spin(fwd, power, pct);
  LMMotor.spin(fwd, power, pct);
  LFMotor.spin(fwd, power, pct);
}

void rightDrive(double power){
  RMMotor.spin(fwd, power, pct);
  RFMotor.spin(fwd, power, pct);
  RBMotor.spin(fwd, power, pct);
}

void drive(double power){
  RMMotor.spin(fwd, power, pct);
  RFMotor.spin(fwd, power, pct);
  RBMotor.spin(fwd, power, pct);
  LBMotor.spin(fwd, power, pct);
  LMMotor.spin(fwd, power, pct);
  LFMotor.spin(fwd, power, pct);
}

void stopDrive(){
  RMMotor.stop(coast);
  RBMotor.stop(coast);
  RFMotor.stop(coast);
  LMMotor.stop(coast);
  LFMotor.stop(coast);
  LBMotor.stop(coast);
}




//--------------------------------Auton Selection----------------------------------//

void drawRectangles() {

  Brain.Screen.clearScreen();

  if (redAuton == true){
    //Red Button
    Brain.Screen.setFillColor(green);
    Brain.Screen.drawRectangle(0, 0, 240, 136);
    //Blue Button
    Brain.Screen.setFillColor(blue);
    Brain.Screen.drawRectangle(240, 0, 240, 136);

    

    //Red Text 
    Brain.Screen.setFillColor(green); 
    Brain.Screen.setPenColor(white); 
    Brain.Screen.printAt(100, 68, "Red");
    //Blue Text 
    Brain.Screen.setFillColor(blue); 
    Brain.Screen.printAt(340, 68, "Blue");

  } else {
    //Red Button
    Brain.Screen.setFillColor(red);
    Brain.Screen.drawRectangle(0, 0, 240, 136);
    //Blue Button
    Brain.Screen.setFillColor(green);
    Brain.Screen.drawRectangle(240, 0, 240, 136);

    //Red Text 
    Brain.Screen.setFillColor(red); 
    Brain.Screen.setPenColor(white); 
    Brain.Screen.printAt(100, 68, "Red");
    //Blue Text 
    Brain.Screen.setFillColor(green); 
    Brain.Screen.printAt(340, 68, "Blue");

  }
  if (rollerSide == true) {
    Brain.Screen.setFillColor(green);
    Brain.Screen.drawRectangle(0, 136, 120, 136);
    Brain.Screen.setFillColor(white);
    Brain.Screen.drawRectangle(120, 136, 120, 136);
    Brain.Screen.setFillColor(white);
    Brain.Screen.drawRectangle(240, 136, 120, 136);
    Brain.Screen.setFillColor(white);
    Brain.Screen.drawRectangle(360, 136, 120, 136);

    //Roller Side text 
    Brain.Screen.setFillColor(green); 
    Brain.Screen.setPenColor(white); 
    Brain.Screen.printAt(10, 190, "Roller Side");
    //Both Sides Text 
    Brain.Screen.setFillColor(white); 
    Brain.Screen.setPenColor(black); 
    Brain.Screen.printAt(130, 190, "Both Sides");
    //Non Roller Side Text 
    Brain.Screen.setFillColor(white); 
    Brain.Screen.setPenColor(black); 
    Brain.Screen.printAt(250, 190, "Non Roller Side");
    //Skills Text
    Brain.Screen.setFillColor(white); 
    Brain.Screen.setPenColor(black); 
    Brain.Screen.printAt(370, 190, "Skills");




  } if (bothSides == true){
    Brain.Screen.setFillColor(white);
    Brain.Screen.drawRectangle(0, 136, 120, 136);
    Brain.Screen.setFillColor(green);
    Brain.Screen.drawRectangle(120, 136, 120, 136);
    Brain.Screen.setFillColor(white);
    Brain.Screen.drawRectangle(240, 136, 120, 136);
    Brain.Screen.setFillColor(white);
    Brain.Screen.drawRectangle(360, 136, 120, 136);

    //Roller Side text 
    Brain.Screen.setFillColor(white); 
    Brain.Screen.setPenColor(black); 
    Brain.Screen.printAt(10, 190, "Roller Side");
    //Both Sides Text 
    Brain.Screen.setFillColor(green); 
    Brain.Screen.setPenColor(white); 
    Brain.Screen.printAt(130, 190, "Both Sides");
    //Non Roller Side Text 
    Brain.Screen.setFillColor(white); 
    Brain.Screen.setPenColor(black); 
    Brain.Screen.printAt(250, 190, "Non Roller Side");
    //Skills Text
    Brain.Screen.setFillColor(white); 
    Brain.Screen.setPenColor(black); 
    Brain.Screen.printAt(370, 190, "Skills");

  } else if (bothSides == false && rollerSide == false && skills == false){
    Brain.Screen.setFillColor(white);
    Brain.Screen.drawRectangle(0, 136, 120, 136);
    Brain.Screen.setFillColor(white);
    Brain.Screen.drawRectangle(120, 136, 120, 136);
    Brain.Screen.setFillColor(green);
    Brain.Screen.drawRectangle(240, 136, 120, 136);
    Brain.Screen.setFillColor(white);
    Brain.Screen.drawRectangle(360, 136, 120, 136);

    //Roller Side text 
    Brain.Screen.setFillColor(white); 
    Brain.Screen.setPenColor(black); 
    Brain.Screen.printAt(10, 190, "Roller Side");
    //Both Sides Text 
    Brain.Screen.setFillColor(white); 
    Brain.Screen.setPenColor(black); 
    Brain.Screen.printAt(130, 190, "Both Sides");
    //Non Roller Side Text 
    Brain.Screen.setFillColor(green); 
    Brain.Screen.setPenColor(white); 
    Brain.Screen.printAt(250, 190, "Non Roller Side");
    //Skills Text
    Brain.Screen.setFillColor(white); 
    Brain.Screen.setPenColor(black); 
    Brain.Screen.printAt(370, 190, "Skills");

  } else if (skills == true) {

    Brain.Screen.setFillColor(white);
    Brain.Screen.drawRectangle(0, 136, 120, 136);
    Brain.Screen.setFillColor(white);
    Brain.Screen.drawRectangle(120, 136, 120, 136);
    Brain.Screen.setFillColor(white);
    Brain.Screen.drawRectangle(240, 136, 120, 136);
    Brain.Screen.setFillColor(green);
    Brain.Screen.drawRectangle(360, 136, 120, 136);

    //Roller Side text 
    Brain.Screen.setFillColor(white); 
    Brain.Screen.setPenColor(black); 
    Brain.Screen.printAt(10, 190, "Roller Side");
    //Both Sides Text 
    Brain.Screen.setFillColor(white); 
    Brain.Screen.setPenColor(black); 
    Brain.Screen.printAt(130, 190, "Both Sides");
    //Non Roller Side Text 
    Brain.Screen.setFillColor(white); 
    Brain.Screen.setPenColor(black); 
    Brain.Screen.printAt(250, 190, "Non Roller Side");
    //Skills Text
    Brain.Screen.setFillColor(green); 
    Brain.Screen.setPenColor(white); 
    Brain.Screen.printAt(370, 190, "Skills");

  }

}

void ButtonPressed(){

  double xPos = Brain.Screen.xPosition();
  double yPos = Brain.Screen.yPosition();

  if (yPos >= 0 && yPos < 136){

    if (xPos >= 0 && xPos < 240){
      redAuton = true;
    } else {
      redAuton = false;
    }

  } else {

    if (xPos >= 0 && xPos < 120){
      rollerSide = true;
      bothSides = false;
      skills = false;
    } else if (xPos >= 120 && xPos < 240) {
      rollerSide = false;
      bothSides = true;
      skills = false;
    } else if (xPos >=240 && xPos < 360){
      rollerSide = false;
      bothSides = false;
      skills = false;
    } else {
      rollerSide = false;
      bothSides = false;
      skills = true;
    }

  }
    
    drawRectangles();

}

//---------------------------------------------------------------------------------------------------------//
//---------------------------------------------------------------------------------------------------------//
//----------------------------------------------ODOM-------------------------------------------------------//
//---------------------------------------------------------------------------------------------------------//
//---------------------------------------------------------------------------------------------------------//

//--------------------------------PID for ODOM----------------------------------//

float prevError = 0;
float totalError = 0;
float der = 0;
float heading = 0;

float fwdPIDCycle(double targetDist, double maxSpeed){

  float kP = 2;
  float kI = 0.01;
  float kD = 1;

  float integralPowerLimit = 45 / kD;
  float integralActiveZone = 10;
  float errorThreshold = 0.5;

  float speed = 0;
  float error = targetDist;

  if (error > errorThreshold){ //Check if error is over the threshold

    if (fabs(error) < integralActiveZone){
      totalError += error;
    } else {
      totalError = 0;
    }
    
    totalError = keepInRange(totalError, -integralPowerLimit, integralPowerLimit);
    der = error - prevError;

    speed = kP * error + kI * totalError + kD * der;

    speed = keepInRange(speed, -maxSpeed, maxSpeed);
    
    if (speed > 0 && speed < 2) {
      speed = 2;
    } if (speed < 0 && speed > -2) {
      speed = -2;
    }



  } else {
    totalError = 0;
    der = 0;
    speed = 0;
  }

  prevError = error;

  return speed;


}

float turnPIDCycle(double targetDeg, double maxSpeed){

  float kP = 2;
  float kI = 0.01;
  float kD = 1;

  float integralPowerLimit = 45 / kD;
  float integralActiveZone = 10;
  float errorThreshold = 0.5;

  float speed = 0;
  float error = targetDeg - getRobotRotation();

  if (error > errorThreshold){ //Check if error is over the threshold

    if (fabs(error) < integralActiveZone){
      totalError += error;
    } else {
      totalError = 0;
    }
    
    totalError = keepInRange(totalError, -integralPowerLimit, integralPowerLimit);
    der = error - prevError;

    speed = kP * error + kI * totalError + kD * der;

    speed = keepInRange(speed, -maxSpeed, maxSpeed);
    
    if (speed > 0 && speed < 2) {
      speed = 2;
    } if (speed < 0 && speed > -2) {
      speed = -2;
    }



  } else {
    totalError = 0;
    der = 0;
    speed = 0;
  }
  
  prevError = error;

  return speed;


}

float driftPIDCycle(double maxSpeed){

  float kP = 2;
  float kI = 0.01;
  float kD = 1;

  float integralPowerLimit = 45 / kD;
  float integralActiveZone = 2;
  float errorThreshold = 0;

  float speed = 0;
  float error = heading - getRobotRotation();

  if (error != errorThreshold){ //Check if error is inequal to the threshold

    if (fabs(error) < integralActiveZone){
      totalError += error;
    } else {
      totalError = 0;
    }
    
    totalError = keepInRange(totalError, -integralPowerLimit, integralPowerLimit);
    der = error - prevError;

    speed = kP * error + kI * totalError + kD * der;

    speed = keepInRange(speed, -maxSpeed, maxSpeed);
    
    if (speed > 0 && speed < 2) {
      speed = 1;
    } if (speed < 0 && speed > -2) {
      speed = -1;
    }



  } else {
    totalError = 0;
    der = 0;
    speed = 0;
  }
  
  prevError = error;

  return speed;

}

void fwdPIDFunction(double targetDist, double maxSpeed, double timeoutMil = -1){ //Run a PID loop that drives forward or backwards

  double currentSpeed = 1;

  timer Time;
  Time.reset();

  totalDistance = 0;

  heading = getRobotRotation();

  while (Time < timeoutMil || timeoutMil == -1){

    double currentDist = targetDist - totalDistance;

    currentSpeed = fwdPIDCycle(currentDist, maxSpeed);

    leftDrive(currentSpeed + driftPIDCycle(maxSpeed));
    rightDrive(currentSpeed - driftPIDCycle(maxSpeed));

    task::sleep(10);

  }

  stopDrive();

}

void turnPIDFunction(double targetDeg, double maxSpeed, double timeoutMil = -1){ //Run a PID loop that Turns the Robot to a certain degree

  double currentSpeed = 1;

  timer Time;
  Time.reset();

  while (Time < timeoutMil || timeoutMil == -1){

    currentSpeed = turnPIDCycle(targetDeg, maxSpeed);

    leftDrive(currentSpeed);
    rightDrive(currentSpeed);

    task::sleep(10);

  }

  stopDrive();

}

//---------------------------------------------- Odometry Function -------------------------------------------------------//

int updatePosition(){ //Updates position and rotation of robot through odometry

  //Get Current Encoder Values in Inches
  curLeft = getLeftReading();
  curRight = getRightReading();
  curBack = getBackReading();

  //Get Change in Encoder Value since last cycle
  deltaL = curLeft - prevLeft;
  deltaR = curRight - prevRight;
  deltaB = curBack - prevBack;

  //Calculate the Angle of the arc traveled
  deltaTheta = (deltaL - deltaR) / (leftOffset + rightOffset);

  if (deltaTheta == 0){ //If the robot has not rotated...

    deltaDistance = (deltaL + deltaR) / 2; //Average change in encoder values
    
  } else {

    double radius = deltaR/deltaTheta - rightOffset; //Find radius of arc by adding right offset to right encoder radius

    deltaDistance = 2 * radius * (sin(deltaTheta/2)); //Find Chord Length from start of robot center to end of robot center

  }

  absoluteAngleOfMovement = prevGyroRadians + deltaTheta/2; //Caluclate Angle between last and current position 

  //Calculate Change in Global Position //Convert Polar Coordinates (radius, angle) to Cartesian Coordinates (x, y)
  deltaX = deltaDistance * cos(absoluteAngleOfMovement); 
  deltaY = deltaDistance * sin(absoluteAngleOfMovement);

  //Calculates Global Change by Summing all changes
  globalX += deltaX;
  globalY += deltaY;

  totalDistance += deltaDistance; //total Distance traveled since last reset

  //Save Previous Values
  prevLeft = curLeft;
  prevRight = curRight;
  prevBack = curBack;
  prevGyroRadians = getRobotRadians();

  return 1;
}

void setTarget(double x, double y){

  targetX = x;
  targetY = y;

}

void turnToTarget(double maxTurnSpeed){ //Turn to Face Target Position

  double turnSpeed = 1;

  while (turnSpeed != 0){

    targetDeg = getDegToPosition(targetX, targetY);

    turnSpeed = turnPIDCycle(targetDeg, maxTurnSpeed);

    leftDrive(-turnSpeed);
    rightDrive(turnSpeed);

    task::sleep(5);

  }
  stopDrive();

}

void moveToTarget(double maxFwdSpeed, double maxTurnSpeed){ //Turn to and Move to position simultaneously

  totalDistance = 0;
  stopTime = 0;

  double curFwdSpeed = 1;
  double curTurnSpeed = 1;

  targetDistance = distanceTo(targetX, targetY);

  while (curFwdSpeed != 0 && targetDistance > 3 && !isStopped()) { //Turns only when farther than a few inches to prevent donuts. isStopped could be faulty

    targetDistance = distanceTo(targetX, targetY);
    targetDeg = getDegToPosition(targetX, targetY);

    curFwdSpeed = fwdPIDCycle(targetDistance, maxFwdSpeed);
    curTurnSpeed = turnPIDCycle(targetDeg, maxTurnSpeed);

    leftDrive(curFwdSpeed - curTurnSpeed);
    rightDrive(curFwdSpeed + curTurnSpeed);

    task::sleep(5);
  }
  while (curFwdSpeed != 0 && !isStopped()) { //Only Drive Forward after a few inches

    targetDistance = distanceTo(targetX, targetY);
    
    curFwdSpeed = fwdPIDCycle(targetDistance, maxFwdSpeed);
    curTurnSpeed = 0;

    leftDrive(curFwdSpeed);
    rightDrive(curFwdSpeed);

    task::sleep(5);

  }
  stopDrive();

}

void moveToTargetRev(double maxFwdSpeed, double maxTurnSpeed){ //Turn to and Move to position simultaneously

  totalDistance = 0;
  stopTime = 0;

  double curFwdSpeed = 1;
  double curTurnSpeed = 1;

  targetDistance = -distanceTo(targetX, targetY);

  while (curFwdSpeed != 0 && fabs(targetDistance) > 3 && !isStopped()) { //Turns only when farther than a few inches to prevent donuts. isStopped could be faulty

    targetDistance = -distanceTo(targetX, targetY);
    targetDeg = getDegToPosition(targetX, targetY);
    targetDeg = angleWrap(targetDeg - 180);

    curFwdSpeed = fwdPIDCycle(targetDistance, maxFwdSpeed);
    curTurnSpeed = turnPIDCycle(targetDeg, maxTurnSpeed);

    leftDrive(curFwdSpeed - curTurnSpeed);
    rightDrive(curFwdSpeed + curTurnSpeed);

    task::sleep(5);
  }
  while (curFwdSpeed != 0 && !isStopped()) { //Only Drive Forward after a few inches

    targetDistance = -distanceTo(targetX, targetY);
    
    curFwdSpeed = fwdPIDCycle(targetDistance, maxFwdSpeed);
    curTurnSpeed = 0;

    leftDrive(curFwdSpeed);
    rightDrive(curFwdSpeed);

    task::sleep(5);

  }
  stopDrive();

}

void passTarget (double maxFwdSpeed, double maxTurnSpeed) { //Passes Target: Allows for more control of path

  totalDistance = 0;

  double curTurnSpeed = 1;

  double initialX = globalX;
  double initialY = globalY;

  targetDistance = distanceTo(targetX, targetY);

  bool passedX = false;
  bool passedY = false;

  while (!passedX || !passedY) {

    passedX = (initialX >= targetX && globalX <= targetX) || (initialX <= targetX && globalX >= targetX);
    passedY = (initialY >= targetY && globalY <= targetY) || (initialY <= targetY && globalY >= targetY);

    targetDistance = distanceTo(targetX, targetY);

    if (fabs(targetDistance) > 2){

      targetDeg = getDegToPosition(targetX, targetY);
      curTurnSpeed = turnPIDCycle(targetDeg, maxTurnSpeed);

    } else {

      curTurnSpeed = 0;

    }

    leftDrive(maxFwdSpeed - curTurnSpeed);
    rightDrive(maxFwdSpeed + curTurnSpeed);

    task::sleep(5);

  }

}

void passTargetRev (double maxFwdSpeed, double maxTurnSpeed) { //Passes Target: Allows for more control of path

  totalDistance = 0;

  double curTurnSpeed = 1;

  double initialX = globalX;
  double initialY = globalY;

  targetDistance = -distanceTo(targetX, targetY);

  bool passedX = false;
  bool passedY = false;

  while (!passedX || !passedY) {

    passedX = (initialX >= targetX && globalX <= targetX) || (initialX <= targetX && globalX >= targetX);
    passedY = (initialY >= targetY && globalY <= targetY) || (initialY <= targetY && globalY >= targetY);

    targetDistance = -distanceTo(targetX, targetY);

    if (fabs(targetDistance) > 2){

      targetDeg = getDegToPosition(targetX, targetY);
      targetDeg = angleWrap(targetDeg - 180);
      curTurnSpeed = turnPIDCycle(targetDeg, maxTurnSpeed);

    } else {

      curTurnSpeed = 0;

    }

    leftDrive(maxFwdSpeed - curTurnSpeed);
    rightDrive(maxFwdSpeed + curTurnSpeed);

    task::sleep(5);

  }

}



//---------------------------------------------- OLD PID FUNCTIONS -------------------------------------------------------//

//drivePID Tuning Values
double kP = 0.05;
double kI = 0.0000000000000001;
double kD = 0.0003;

//TurnPID Tuning Values
double turnkP = 0.04;
double turnkI = 0.000000001; //0.00000000000000000001
double turnkD = 0.0003;

//driftPID Tuning Values
double dkP = 0.02;
double dkI = 0.00000000001;
double dkD = 0.0003;

int error; //Sensor Value - Desired Value : Position

int ticks = 0;

int driftError;
int driftPrevError;
int driftTotalError;
int driftDer;

void drivePID (int desiredValue, int maxSpeed = 12){

  //Reset motor Position
  LMMotor.setPosition(0, degrees);
  RMMotor.setPosition(0, degrees);
  LBMotor.setPosition(0, degrees);
  RBMotor.setPosition(0, degrees);
  LFMotor.setPosition(0, degrees);
  RFMotor.setPosition(0, degrees);

  int heading = Gyro.orientation(yaw, degrees);

  while (true) {
    
    int leftMotorPosition = (LBMotor.position(degrees) + LFMotor.position(degrees) + LMMotor.position(degrees)) / 3;
    int rightMotorPosition = (RBMotor.position(degrees) + RFMotor.position(degrees) + RMMotor.position(degrees)) / 3;
    int averagePosition = (leftMotorPosition + rightMotorPosition) / 2;
    driftError = heading - Gyro.orientation(yaw, degrees);

    //potential
    error = averagePosition - desiredValue;
    //derivative
    der = error - prevError;
    //intergral
    totalError += error; 

    //Drift Potential
    driftError = Gyro.orientation(yaw, degrees) - heading;
    //Drift Derivitive
    driftDer = driftError - driftPrevError;
    //Drift Integral
    driftTotalError += driftError;

    int motorDifference = (driftError * dkP + driftDer * dkD) * 50; //+ driftTotalError * dkI

    double lateralMotorPower = (error * kP + der * kD) * 10; //+ totalError * kI

    lateralMotorPower = keepInRange(lateralMotorPower, -maxSpeed, maxSpeed);

    LMMotor.spin(reverse, lateralMotorPower - motorDifference, voltageUnits::volt);
    LFMotor.spin(reverse, lateralMotorPower - motorDifference, voltageUnits::volt);
    LBMotor.spin(reverse, lateralMotorPower - motorDifference, voltageUnits::volt);
    RMMotor.spin(reverse, lateralMotorPower + motorDifference, voltageUnits::volt);
    RFMotor.spin(reverse, lateralMotorPower + motorDifference, voltageUnits::volt);
    RBMotor.spin(reverse, lateralMotorPower + motorDifference, voltageUnits::volt);

    Brain.Screen.clearScreen();
    Brain.Screen.setCursor(5,5);
    Brain.Screen.setPenColor(green);
    Brain.Screen.print("error = ");
    Brain.Screen.setCursor(5, 10);
    Brain.Screen.print(error);
    Brain.Screen.setCursor(7,5);
    Brain.Screen.print("Movement Val = ");
    Brain.Screen.setCursor(7, 20);
    Brain.Screen.print(lateralMotorPower);
    Brain.Screen.setCursor(6,5);
    Brain.Screen.setPenColor(red);
    Brain.Screen.print("Derivative = ");
    Brain.Screen.setCursor(6, 17);
    Brain.Screen.print(der);


    prevError = error;
    driftPrevError = driftError;
    vex::task::sleep(20);

    if (abs(error) < 15){
      Brain.Screen.setCursor(5, 20);
      Brain.Screen.setPenColor(purple);
      Brain.Screen.print("Break");
      break;
    }

  }

  LMMotor.stop();
  LFMotor.stop();
  LBMotor.stop();
  RMMotor.stop();
  RBMotor.stop();
  RFMotor.stop();
  
  LMMotor.setStopping(hold);
  LBMotor.setStopping(hold);
  LFMotor.setStopping(hold);
  RBMotor.setStopping(hold);
  RMMotor.setStopping(hold);
  RFMotor.setStopping(hold);

  vex::task::sleep(20);

  LMMotor.setStopping(brake);
  LBMotor.setStopping(brake);
  LFMotor.setStopping(brake);
  RBMotor.setStopping(brake);
  RMMotor.setStopping(brake);
  RFMotor.setStopping(brake);
  

}

void turnPID (int desiredValue, int maxSpeed = 12){

  //Reset motor Position
  LMMotor.setPosition(0, degrees);
  RMMotor.setPosition(0, degrees);
  LBMotor.setPosition(0, degrees);
  RBMotor.setPosition(0, degrees);
  LFMotor.setPosition(0, degrees);
  RFMotor.setPosition(0, degrees);

  while (true) {

    int gyroPosition = Gyro.orientation(yaw, degrees);

    //potential
    error = gyroPosition - desiredValue;
    //derivative
    der = error - prevError;
    //intergral
    totalError += error; 

    double lateralMotorPower = (error * turnkP + der * turnkD) * 10; //+ totalError * turnkI

    lateralMotorPower = keepInRange(lateralMotorPower, -maxSpeed, maxSpeed);

    LMMotor.spin(reverse, lateralMotorPower, voltageUnits::volt);
    LFMotor.spin(reverse, lateralMotorPower, voltageUnits::volt);
    LBMotor.spin(reverse, lateralMotorPower, voltageUnits::volt);
    RMMotor.spin(forward, lateralMotorPower, voltageUnits::volt);
    RFMotor.spin(forward, lateralMotorPower, voltageUnits::volt);
    RBMotor.spin(forward, lateralMotorPower, voltageUnits::volt);

    Brain.Screen.clearScreen();
    Brain.Screen.setCursor(3,5);
    Brain.Screen.setPenColor(green);
    Brain.Screen.print("error = ");
    Brain.Screen.setCursor(3, 10);
    Brain.Screen.print(error);
    Brain.Screen.setCursor(6,5);
    Brain.Screen.print("Movement Val = ");
    Brain.Screen.setCursor(6, 20);
    Brain.Screen.print(lateralMotorPower);
    Brain.Screen.setCursor(9,5);
    Brain.Screen.setPenColor(red);
    Brain.Screen.print("Derivative = ");
    Brain.Screen.setCursor(9, 17);
    Brain.Screen.print(der);
    Brain.Screen.setCursor(11, 24);
    Brain.Screen.setPenColor(purple);
    Brain.Screen.print("Gyro Heading");
    Brain.Screen.setCursor(11, 40);
    Brain.Screen.print(Gyro.orientation(yaw,degrees));


    prevError = error;
    vex::task::sleep(20);

    Brain.Screen.print(error);

    if (abs(error) == 0){
      Brain.Screen.setCursor(5, 20);
      Brain.Screen.setPenColor(purple);
      Brain.Screen.print("Break");
      break;
    } else if (abs(error) < 6){
      ticks++;
      if (ticks > 30){
        break;
      }
    } else {
      ticks = 0;
    }

  }

  LMMotor.stop();
  LFMotor.stop();
  LBMotor.stop();
  RMMotor.stop();
  RBMotor.stop();
  RFMotor.stop();
  
  LMMotor.setStopping(hold);
  LBMotor.setStopping(hold);
  LFMotor.setStopping(hold);
  RBMotor.setStopping(hold);
  RMMotor.setStopping(hold);
  RFMotor.setStopping(hold);

  vex::task::sleep(20);

  LMMotor.setStopping(brake);
  LBMotor.setStopping(brake);
  LFMotor.setStopping(brake);
  RBMotor.setStopping(brake);
  RMMotor.setStopping(brake);
  RFMotor.setStopping(brake);
  

}

//-----------------------------------Catapult Functions-------------------------------------------//

int cataFire(){

  int limitSwitchCan = 0;

  while (true) {

    if (firingCata == true) {

      Cata.spin(vex::directionType::rev, 100, vex::velocityUnits::pct);
      limitSwitchCan = limitSwitchCan + 1;
      Cata.setStopping(hold);

      if (LimitSwitch.pressing() && limitSwitchCan >= 10){
        limitSwitchCan = 0;
        Cata.stop();
        firingCata = false;
      }

    }

    vex::task::sleep(25);

  }

  return(0);

}

void fireCata() {

  int limitSwitchCan = 0;

  while(true) {

    Cata.spin(vex::directionType::rev, 100, vex::velocityUnits::pct);
    limitSwitchCan = limitSwitchCan + 1;

    if (LimitSwitch.pressing() && limitSwitchCan >= 20){
      break;
    }

    vex::task::sleep(20);
  }

  Cata.stop();
  Cata.setStopping(hold);

}

/*---------------------------------------------------------------------------*/
/*                          Pre-Autonomous Functions                         */
/*                                                                           */
/*  You may want to perform some actions before the competition starts.      */
/*  Do them in the following function.  You must return from this function   */
/*  or the autonomous and usercontrol tasks will not be started.  This       */
/*  function is only called once after the V5 has been powered on and        */
/*  not every time that the robot is disabled.                               */
/*---------------------------------------------------------------------------*/

void pre_auton(void) {
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();
  
  Gyro.calibrate();

  ExpansionPnuematics1.on();
  ExpansionPnuematics2.on();

  drawRectangles();

  ////Red Button
  //Brain.Screen.setFillColor(red);
  //Brain.Screen.drawRectangle(0, 0, 240, 136);
  ////Blue Button
  //Brain.Screen.setFillColor(blue);
  //Brain.Screen.drawRectangle(240, 0, 240, 136);
  ////Roller Buttons
  //Brain.Screen.setFillColor(white);
  //Brain.Screen.drawRectangle(0, 136, 160, 136);
  //Brain.Screen.drawRectangle(160, 136, 160, 136);
  //Brain.Screen.drawRectangle(320, 136, 160, 136);
  //Brain.Screen.setPenColor(white);
  ////Red Text 
  //Brain.Screen.setFillColor(red); 
  //Brain.Screen.setPenColor(white); 
  //Brain.Screen.printAt(100, 68, "Red");
  ////Blue Text 
  //Brain.Screen.setFillColor(blue); 
  //Brain.Screen.printAt(340, 68, "Blue");
  ////Roller Side text 
  //Brain.Screen.setFillColor(white); 
  //Brain.Screen.setPenColor(black); 
  //Brain.Screen.printAt(30, 190, "Roller Side");
  ////Both Sides Text 
  //Brain.Screen.setFillColor(white); 
  //Brain.Screen.setPenColor(black); 
  //Brain.Screen.printAt(190, 190, "Both Sides");
  ////Non Roller Side Text 
  //Brain.Screen.setFillColor(white); 
  //Brain.Screen.setPenColor(black); 
  //Brain.Screen.printAt(330, 190, "Non Roller Side"); 
//
  Brain.Screen.pressed(ButtonPressed);
  // All activities that occur before the competition starts
  // Example: clearing encoders, setting servo positions, ...
}


/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              Autonomous Task                              */
/*                                                                           */
/*  This task is used to control your robot during the autonomous phase of   */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/

void autonomous(void) {
  // ..........................................................................
  
  //enableDrivePID = true;
  //vex::task driveForward(drivePID);

  vex::task t1(cataFire);
  vex::task t2(updatePosition);

  if (rollerSide == true){ //Roller Side Auton
    firingCata = true;
    Intake.spin(forward, 100, vex::velocityUnits::pct);
    drivePID(50, 6);
    drivePID(-100);
    turnPID(135, 9);
    drivePID(100);
    drivePID(-100);
    Intake.stop();
    turnPID(75, 8);
    drivePID(-1000);
    turnPID(0);
    drivePID(500);
    turnPID(-40, 7);
    drivePID(-100);
    wait(0.5, sec);
    firingCata = true;
    wait(0.5, sec);
    drivePID(100);
    turnPID(45);
    Intake.spin(forward, 100, vex::velocityUnits::pct);
    drivePID(400, 6);
    wait(0.5, sec);
    drivePID(600, 11);
    wait(0.5, sec);
    drivePID(-1000);
    turnPID(-40, 7);
    drivePID(-100);
    wait(0.5, sec);
    firingCata = true;
  } else if (bothSides == true){ //Both Sides Auton
    firingCata = true;
    Intake.spin(forward, 100, vex::velocityUnits::pct);
    drivePID(50, 6);
    drivePID(-100);
    turnPID(-135, 11);
    drivePID(400);
    wait(0.5, sec);
    drivePID(100, 6);
    drivePID(500);
    turnPID(-40, 7);
    drivePID(-100);
    wait(0.5, sec);
    firingCata = true;
    wait(0.5, sec);
    drivePID(100);
    turnPID(-135, 11);
    drivePID(1000, 9);
    wait(0.5, sec);
    drivePID(800);
    turnPID(-50);
    drivePID(-100);
    wait(0.5, sec);
    firingCata = true;
    wait(0.5, sec);
    drivePID(100);
    turnPID(-132);
    drivePID(1500);
  } else if (skills == true) { //Skills Auton
    firingCata = true;
    Intake.spin(forward, 100, vex::velocityUnits::pct);
    drivePID(50, 6); //drive Forward and Spin Roller
    drivePID(100);

    
    wait(1, sec);
    drivePID(-100);
    wait(0.2, sec);
    drivePID(90);
    wait(1, sec);
    Intake.stop();
    drivePID(-385); //drive towards other roller
    wait(0.2, sec);
    turnPID(90);
    turnPID(90);
    wait(0.2, sec);
    Intake.spin(reverse, 100, vex::velocityUnits::pct);
    drivePID(490); //Spin Roller
    wait(1, sec);
    drivePID(-20);
    wait(1, sec);
    Intake.stop();
    drivePID(-300); //Drive away from roller
    wait(0.2, sec);
    turnPID(-9);
    turnPID(-9);
    wait(0.2, sec);
    drivePID(-800);
    wait(0.2, sec);
    firingCata = true;
    wait(1, sec);
    drivePID(700);
    wait(0.2, sec);
    turnPID(-135);
    wait(0.2, sec);
    Intake.spin(forward, 100, vex::velocityUnits::pct);
    drivePID(900);
    wait(3, sec);
    turnPID(-45);
    wait(0.2, sec);
    drivePID(-300);
    wait(0.2, sec);
    firingCata = true;
    wait(1, sec);
    drivePID(300);
    wait(0.2, sec);
    turnPID(50);
    turnPID(50);
    wait(0.2, sec);
    drivePID(1200);
    wait(1, sec);
    ExpansionPnuematics1.off();
    ExpansionPnuematics2.off();

  } else { //Non Roller Side Auton
    firingCata = true;
    Intake.spin(forward, 100, vex::velocityUnits::pct);
    drivePID(-750);
    turnPID(-140, 8);
    drivePID(-100);
    wait(0.5, sec);
    firingCata = true;
    wait(0.5, sec);
    drivePID(100);
    turnPID(45, 8);
    drivePID(100, 10);
    wait(0.5, sec);
    drivePID(-100);
    turnPID(-45);
    drivePID(400, 9);
    turnPID(-135, 7);
    drivePID(-100);
    wait(0.5, sec);
    firingCata = true;
    wait(0.5, sec);
    drivePID(100);
    turnPID(137, 10);
    drivePID(1500);
  }
  
  



  // ..........................................................................
}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              User Control Task                            */
/*                                                                           */
/*  This task is used to control your robot during the user control phase of */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/

void usercontrol(void) {
  // User control code here, inside the loop
  

  //Variables

  double turnImportance = 0.5; //Double determines how heavily turning affects driving

  int multiplier = 1;

  //Catapult Variables
  bool cataDown = true;
  bool cataMoving = false;
  bool limitSwitchVar = false;
  int limitSwitchCan = 0;
  bool bCoast = false; //Currently not in use, is used to coast motor when needed

  //Robot Speed Variables
  double currentPosition;
  double prevPosition = 0;
  double motorSpeed;

  bool driftCorrect = false;
  double heading;
  double motorCorrection = 0;

  bool motorCoast = true;


  //Sensor Variables
  bool bDetect; //Vision Sensor is Detecting Object
  
  while (1) {

    //Clear Scren to prepare for text
    Brain.Screen.clearScreen();

    //---------------------------------------Drivecode---------------------------------------//
    double turnVal = Controller1.Axis1.position(pct);
    double forwardVal = Controller1.Axis3.position(pct) * multiplier;

    double turnVolts = turnVal * -0.12;
    double forwardVolts = forwardVal * 0.12 * (1-(std::abs(turnVolts)/12) * turnImportance);


    LMMotor.spin(fwd, ((forwardVolts - turnVolts) + motorCorrection) , voltageUnits::volt);
    LFMotor.spin(fwd, ((forwardVolts - turnVolts) + motorCorrection) , voltageUnits::volt);
    LBMotor.spin(fwd, ((forwardVolts - turnVolts) + motorCorrection) , voltageUnits::volt);
    RMMotor.spin(fwd, ((forwardVolts +  turnVolts) - motorCorrection), voltageUnits::volt);
    RFMotor.spin(fwd, ((forwardVolts + turnVolts) - motorCorrection) , voltageUnits::volt);
    RBMotor.spin(fwd, ((forwardVolts + turnVolts) - motorCorrection) , voltageUnits::volt);

    //Motor Speed Calculations
    currentPosition = (LBMotor.position(degrees) + LFMotor.position(degrees) + LMMotor.position(degrees) + RBMotor.position(degrees) + RFMotor.position(degrees) + RMMotor.position(degrees)) / 6;
    motorSpeed = currentPosition - prevPosition;

    //Motor Coast
    if (motorCoast){

      LMMotor.setStopping(coast);
      LFMotor.setStopping(coast);
      LBMotor.setStopping(coast);
      RMMotor.setStopping(coast);
      RFMotor.setStopping(coast);
      RBMotor.setStopping(coast);

    } else{

      LMMotor.setStopping(brake);
      LFMotor.setStopping(brake);
      LBMotor.setStopping(brake);
      RMMotor.setStopping(brake);
      RFMotor.setStopping(brake);
      RBMotor.setStopping(brake);

    }

    if (Controller1.ButtonRight.pressing()){

      motorCoast = true;

    } else if (Controller1.ButtonDown.pressing()){

      motorCoast = false;

    }

    //Drift Correction?
    if (forwardVal != 0) {

      if (turnVal == 0) {

        if (driftCorrect == false) {

          heading = Gyro.orientation(yaw, degrees);
          driftCorrect = true;

        } else {

          motorCorrection = (heading - Gyro.orientation(yaw, degrees)) * 0;

        }


      } else{

        driftCorrect = false;
        motorCorrection = 0;

      }
    } else {

      driftCorrect = false;
      motorCorrection = 0;

    }

    //---------------------------------------Intake---------------------------------------//
    if (Controller1.ButtonL2.pressing()){

      Intake.spin(vex::directionType::fwd, 100, vex::velocityUnits::pct);

    } else if (Controller1.ButtonL1.pressing()){

      Intake.spin(vex::directionType::rev, 100, vex::velocityUnits::pct);

    } else {

      Intake.stop();

    }

    //---------------------------------------Expansion---------------------------------------//
    if (Controller1.ButtonB.pressing()){
      ExpansionPnuematics1.off();
      Controller1.rumble(".");
    }

    if (Controller1.ButtonY.pressing()){
      ExpansionPnuematics2.off();
      Controller1.rumble(".");
    }

    if (Controller1.ButtonRight.pressing()){
      ExpansionPnuematics1.on();
      ExpansionPnuematics2.on();
      Controller1.rumble(".");
    }

    //---------------------------------------Catapult---------------------------------------//

    limitSwitchVar = LimitSwitch.pressing();//Variable for Limit Switch

    if (Controller1.ButtonR1.pressing()){

      CataPistons.on();

    } else {

      CataPistons.off();

    }


    //Cata Stop
    if (limitSwitchVar == true && limitSwitchCan >= 20) {
      Controller1.rumble("-");
      cataMoving = false;
      cataDown = true;
      limitSwitchCan = 0;
      Cata.stop();
    }

    //Cata Fire
    if (Controller1.ButtonR2.pressing()){
      if (cataDown == true) {
        cataMoving = true;
        cataDown = false;
        Cata.setPosition(0, degrees);
        limitSwitchCan = 1;
      }
    }

    //Cata Cooldown
    if (limitSwitchCan > 0) {
      limitSwitchCan = limitSwitchCan + 1;
    }

    //Coast Motor if not down (not in use)
    if (LimitSwitch.pressing() && bCoast == true){
      wait(1, sec);
      if (LimitSwitch.pressing()){
        bCoast = false;
      }

    } //else {
      //bCoast = true;
    //}

    //Cata recoil  -- Changed Stopping Mode, Changed velocity, org velocity: 100pct
    if (cataMoving == true){
      Cata.spin(vex::directionType::rev, 100, vex::velocityUnits::pct);
      Cata.setStopping(hold); //Coast Before
    } else {
      Cata.stop();
      if (bCoast == true){
        Cata.setStopping(coast); //Coast Before
      } else {
        Cata.setStopping(hold);
      }
    }

    //Cata Hardstop
    if (Controller1.ButtonA.pressing()){
      bCoast = true;
      cataDown = true;
      cataMoving = false;
      limitSwitchCan = 0;
      Cata.stop();
    }

    //Coast Hardstop
    if (Controller1.ButtonX.pressing()){
      bCoast = false;
    }

    //Manual Cata
    if (Controller1.ButtonUp.pressing()){
      Cata.spin(vex::directionType::rev, 100, vex::velocityUnits::pct);
    } 

    if (Controller1.ButtonLeft.pressing()){
      Cata.spin(vex::directionType::fwd, 100, vex::velocityUnits::pct);
    } 


    //---------------------------------------Vision Sensor---------------------------------------//
    if (redAuton == true) {
      VisionSensor.takeSnapshot(VisionSensor__RED_GOAL); //Looks for Red Goal

    } else {
      VisionSensor.takeSnapshot(VisionSensor__BLUE_GOAL); //Looks for Blue Goal
    }

    //Is in Range
    if (VisionSensor.largestObject.exists) {
      if ((VisionSensor.largestObject.centerX < 185 && VisionSensor.largestObject.centerX > 160)){ 
        bDetect = true;
      } else {
        bDetect = false;
      }

    } else {
      bDetect = false;
    }


    //---------------------------------------Robot Statistics/BrainPrinting---------------------------------------//
    Brain.Screen.setPenWidth(20);

    //Alliiance Color
    if (redAuton == true) {
      Brain.Screen.setCursor(2,2);
      Brain.Screen.setPenColor(red);
      Brain.Screen.print("Red Auton Selected");
    } else {
      Brain.Screen.setCursor(2,2);
      Brain.Screen.setPenColor(blue);
      Brain.Screen.print("Blue Auton Selected");
    }

    //Auton Side
    if (rollerSide == true) {
      Brain.Screen.setCursor(5,2);
      Brain.Screen.setPenColor(white);
      Brain.Screen.print("Roller Side");
    } else if (bothSides == true){
      Brain.Screen.setCursor(5,2);
      Brain.Screen.setPenColor(white);
      Brain.Screen.print("Both Sides");
    }
    
    //Catamoving?
    Brain.Screen.setCursor(7,2);
    Brain.Screen.setPenColor(green);
    Brain.Screen.print(cataMoving);

    //DriveSpeed
    Brain.Screen.setCursor(9,2);
    Brain.Screen.setPenColor(orange);
    Brain.Screen.print("Drive Speed = ");
    Brain.Screen.setCursor(9,15);
    Brain.Screen.print(motorSpeed);
    
    //LimitSwitchCooldownTimer
    Brain.Screen.setCursor(11,2);
    Brain.Screen.setPenColor(purple);
    Brain.Screen.print("Limit Switch:");
    Brain.Screen.setCursor(11, 15);
    Brain.Screen.print(limitSwitchCan);
    
    //Vision Sensor
    Brain.Screen.setCursor(5, 24);
    Brain.Screen.setPenColor(purple);
    if (bDetect) {
      Brain.Screen.print("Detected Goal");
    } else {
      Brain.Screen.print("Didn't");
    }
    
    //Vision Sensor Object Location
    Brain.Screen.setCursor(7, 30);
    Brain.Screen.print(VisionSensor.largestObject.centerX);
    Brain.Screen.setCursor(7, 35);
    Brain.Screen.print(VisionSensor.largestObject.centerY);

    //Gyroscope Position
    Brain.Screen.setCursor(11, 24);
    Brain.Screen.setPenColor(red);
    Brain.Screen.print("Gyro Position");
    Brain.Screen.setCursor(11, 38);
    Brain.Screen.print(Gyro.orientation(yaw, degrees));

    //Drift Correction
    Brain.Screen.setCursor(2, 24);
    Brain.Screen.setPenColor(green);
    Brain.Screen.print("Heading");
    Brain.Screen.setCursor(2, 33);
    Brain.Screen.print(heading);
    Brain.Screen.setCursor(3, 24);
    Brain.Screen.print("Rotation");
    Brain.Screen.setCursor(3, 35);
    Brain.Screen.print(Gyro.orientation(yaw, degrees));

    prevPosition = currentPosition;
    wait(20, msec); // Sleep the task for a short amount of time to
                    // prevent wasted resources.
  }
}

//
// Main will set up the competition functions and callbacks.
//
int main() {
  // Set up callbacks for autonomous and driver control periods.
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);

  // Run the pre-autonomous function.
  pre_auton();

  // Prevent main from exiting with an infinite loop.
  while (true) {
    wait(100, msec);
  }
}
