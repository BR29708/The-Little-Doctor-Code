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
// RBMotor              motor         21              
// LMMotor              motor         1               
// LFMotor              motor         2               
// LBMotor              motor         14              
// Intake               motor         8               
// Cata                 motor         15              
// ExpansionPnuematics1 led           A               
// ExpansionPnuematics2 led           B               
// LimitSwitch          limit         C               
// Gyro                 inertial      3               
// Color                optical       17              
// VisionSensor         vision        16              
// CataPistons          led           D               
// EncoderL             encoder       C, D            
// EncoderR             encoder       E, F            
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
const double rightOffset = 0; //Subject to change when we add tracking wheels, Distance from Tracking Center to Right Tracking Wheel
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
    n = high; //if the number is higher than the max number, set it to the max number
  }
  if (n < low){
    n = low; //if the number is lower than the min number, set it to the min number
  }
  return n;
}

double distanceTo (double x1, double y1){ //Calculates Distance Between Current Location (GlobalX, GlobalY) and target points (x1, y1)
  return sqrt(pow(globalX - x1, 2) + pow(globalY - y1, 2));
}

double getRightReading(){ //gets the reading of the right encoder
  return degreesToInches(EncoderR.position(degrees));
}

double getLeftReading(){ //gets the reading of the left encoder
  return degreesToInches(EncoderL);
}

double getBackReading(){ //gets the reading of the back encoder
  return degreesToInches(EncoderB);
}

bool isStopped(){ //Checks to see if Robot isn't moving: Makes sure it doens't run into walls (COULD ENCOUNTER PROBLEMS!)

  if (fabs(deltaR) <= 2 && fabs(deltaL) <= 2){
    stopTime += 1; //increases stop time for every cycle tha the encoders are not changing values
  } else {
    stopTime = 0;
  }
  if (stopTime >= 50){ //if stopped for 50 ticks...
    return true; //return true, stop motors
  } else { //else...
    return false; //return false, continue motors
  }

}

double getRobotRotation(){ //Gets robot orientation in degrees
  return -Gyro.orientation(yaw, degrees); //Negative because when working in radians, left is positive, right is negative
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

  double relativeX = x - globalX; //get the relative distance between the robots coordinates and the desired point
  double relativeY = y - globalY; 

  double degToPosition = toDegrees * atan2(relativeY, relativeX); //find the necessary angle to turn to to face the desired point 
  
  degToPosition = angleWrap(degToPosition); //angle wrap so we turn less

  return degToPosition; //return angle

}

//Movement Functions

void leftDrive(double power) { //drive the left motors based on the power variable
  LBMotor.spin(reverse, power, voltageUnits::volt);
  LMMotor.spin(reverse, power, voltageUnits::volt);
  LFMotor.spin(reverse, power, voltageUnits::volt);
}

void rightDrive(double power){ //drive the right motors based on the power variable
  RMMotor.spin(reverse, power, voltageUnits::volt);
  RFMotor.spin(reverse, power, voltageUnits::volt);
  RBMotor.spin(reverse, power, voltageUnits::volt);
}

void drive(double power){ //drive both right and left motors based on the power variable
  RMMotor.spin(reverse, power, voltageUnits::volt);
  RFMotor.spin(reverse, power, voltageUnits::volt);
  RBMotor.spin(reverse, power, voltageUnits::volt);
  LBMotor.spin(reverse, power, voltageUnits::volt);
  LMMotor.spin(reverse, power, voltageUnits::volt);
  LFMotor.spin(reverse, power, voltageUnits::volt);
}

void stopDrive(){ //stop the drive and set all motors to coast
  RMMotor.stop(coast);
  RBMotor.stop(coast);
  RFMotor.stop(coast);
  LMMotor.stop(coast);
  LFMotor.stop(coast);
  LBMotor.stop(coast);
}




//--------------------------------Auton Selection----------------------------------//

void drawRectangles() { //Function to draw rectangles to the screen for the Auton selector

  Brain.Screen.clearScreen(); //clears screen every cycle to make sure we don't draw over the top of each other

  if (redAuton == true){ //if our allinace color is red...
    //Red Button
    Brain.Screen.setFillColor(green); //set the red square to green
    Brain.Screen.drawRectangle(0, 0, 240, 136);
    //Blue Button
    Brain.Screen.setFillColor(blue); //set the blue button ot blue
    Brain.Screen.drawRectangle(240, 0, 240, 136);

    

    //Red Text 
    Brain.Screen.setFillColor(green); //print the text that says "red" over the button
    Brain.Screen.setPenColor(white); 
    Brain.Screen.printAt(100, 68, "Red");
    //Blue Text 
    Brain.Screen.setFillColor(blue); //print text that says "blue" over the button
    Brain.Screen.printAt(340, 68, "Blue");

  } else { //if our allinace color is blue...
    //Red Button
    Brain.Screen.setFillColor(red); //set the red square to red
    Brain.Screen.drawRectangle(0, 0, 240, 136);
    //Blue Button
    Brain.Screen.setFillColor(green); //set the blue square to blue
    Brain.Screen.drawRectangle(240, 0, 240, 136);

    //Red Text 
    Brain.Screen.setFillColor(red); //print the text that says "red" over the button
    Brain.Screen.setPenColor(white); 
    Brain.Screen.printAt(100, 68, "Red");
    //Blue Text 
    Brain.Screen.setFillColor(green); //print the text that says "blue" over the button
    Brain.Screen.printAt(340, 68, "Blue");

  }
  if (rollerSide == true) { //if the roller side (left side) auton is selected...
    //draw roller side button, set it to green
    Brain.Screen.setFillColor(green);
    Brain.Screen.drawRectangle(0, 136, 120, 136);
    //draw both side button
    Brain.Screen.setFillColor(white);
    Brain.Screen.drawRectangle(120, 136, 120, 136);
    //draw non-roller side button
    Brain.Screen.setFillColor(white);
    Brain.Screen.drawRectangle(240, 136, 120, 136);
    //draw skills button
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




  } if (bothSides == true){ //if both sides auton is selected...
    //draw roller side button
    Brain.Screen.setFillColor(white);
    Brain.Screen.drawRectangle(0, 136, 120, 136);
    //draw both sides button, set it to green
    Brain.Screen.setFillColor(green);
    Brain.Screen.drawRectangle(120, 136, 120, 136);
    //draw non-roller side auton
    Brain.Screen.setFillColor(white);
    Brain.Screen.drawRectangle(240, 136, 120, 136);
    //draw skills button
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

  } else if (bothSides == false && rollerSide == false && skills == false){ //if non-roller side (right side) auton is selected...
    //draw roller side button
    Brain.Screen.setFillColor(white);
    Brain.Screen.drawRectangle(0, 136, 120, 136);
    //draw both sides button
    Brain.Screen.setFillColor(white);
    Brain.Screen.drawRectangle(120, 136, 120, 136);
    //draw non roller side button, color it green
    Brain.Screen.setFillColor(green);
    Brain.Screen.drawRectangle(240, 136, 120, 136);
    //draw skills button
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

  } else if (skills == true) { //if skills auton is selected...
    //draw roller side button
    Brain.Screen.setFillColor(white);
    Brain.Screen.drawRectangle(0, 136, 120, 136);
    //draw both sides button
    Brain.Screen.setFillColor(white);
    Brain.Screen.drawRectangle(120, 136, 120, 136);
    //draw non roller side button
    Brain.Screen.setFillColor(white);
    Brain.Screen.drawRectangle(240, 136, 120, 136);
    //draw skills button, color it green
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

void ButtonPressed(){ //function that is tied to when the screen is pressed. Finds which button was pressed and sets variables accordingly

  double xPos = Brain.Screen.xPosition(); //gets the x-position of where the screen was pressed
  double yPos = Brain.Screen.yPosition(); //gets the x-position of where the screen was pressed

  if (yPos >= 0 && yPos < 136){ //if the cursor is in the top half of the screen...

    if (xPos >= 0 && xPos < 240){ //if the cursor is in the red button area..
      redAuton = true; //set the allinace color to red
    } else { //if the cursor is in the blue button area...
      redAuton = false; //set the alliance color to blue
    }

  } else { //if the cursor is on the bottom half of the screen 

    if (xPos >= 0 && xPos < 120){ //if the cursor is in the roller side button area...
      rollerSide = true; //set roller side selected to true
      bothSides = false; 
      skills = false;
    } else if (xPos >= 120 && xPos < 240) { //if the cursor is in the both sides button area...
      rollerSide = false;
      bothSides = true; //set both side selected to true
      skills = false;
    } else if (xPos >=240 && xPos < 360){ //if the cursor is in the non-roller side area... 
      rollerSide = false;
      bothSides = false; //set all variables to false, making the non-roller side true
      skills = false;
    } else { //if the cursor is in the skills area...
      rollerSide = false;
      bothSides = false;
      skills = true; //set skills selected to true
    }

  }
    
    drawRectangles(); //draw the rectangles using the above function

}

//---------------------------------------------------------------------------------------------------------//
//---------------------------------------------------------------------------------------------------------//
//----------------------------------------------ODOM-------------------------------------------------------//
//---------------------------------------------------------------------------------------------------------//
//---------------------------------------------------------------------------------------------------------//

//--------------------------------PID for ODOM----------------------------------//

float prevError = 0; //Error from previous PID Cycle
float totalError = 0; //
float der = 0;
float heading = 0;

float fwdPIDCycle(double targetDist, double maxSpeed){

  float kP = 0.27;
  float kI = 0.001; //0.000000001
  float kD = 0.01; //0.031

  float integralPowerLimit = 45 / kD;
  float integralActiveZone = 600;
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

    Brain.Screen.setCursor(7, 2);
    Brain.Screen.setPenColor(green);
    Brain.Screen.print("Error = ");
    Brain.Screen.setCursor(7, 11);
    Brain.Screen.print(error);
    Brain.Screen.setCursor(9, 2);
    Brain.Screen.print("Speed = ");
    Brain.Screen.setCursor(9, 11);
    Brain.Screen.print(speed);

  } else {
    totalError = 0;
    der = 0;
    speed = 0;
  }

  prevError = error;

  return speed;


}

float turnPIDCycle(double targetDeg, double maxSpeed, double kp = 0.1){

  float kP = kp;
  float kI = 0.000001;;
  float kD = 0.08;

  float integralPowerLimit = 45 / kD;
  float integralActiveZone = 10;
  float errorThreshold = 0.5;

  float speed = 0;
  float error = targetDeg - getRobotRotation();

  if (fabs(error) > errorThreshold){ //Check if error is over the threshold

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

    speed = speed * -1;

    

  } else {
    totalError = 0;
    der = 0;
    speed = 0;
  }

  Brain.Screen.setCursor(7, 2);
  Brain.Screen.setPenColor(green);
  Brain.Screen.print("Error = ");
  Brain.Screen.setCursor(7, 11);
  Brain.Screen.print(error);
  Brain.Screen.setCursor(9, 2);
  Brain.Screen.print("Speed = ");
  Brain.Screen.setCursor(9, 11);
  Brain.Screen.print(speed);


  
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

  while(true){

    //Get Current Encoder Values in Inches
    curLeft = getLeftReading();
    curRight = getRightReading();
    curBack = getBackReading();

    //Get Change in Encoder Value since last cycle
    deltaL = curLeft - prevLeft;
    deltaR = curRight - prevRight;
    deltaB = curBack - prevBack;

    //Calculate the Angle of the arc traveled
    deltaTheta = getRobotRadians() - prevGyroRadians;//(deltaL - deltaR) / (leftOffset + rightOffset);

    if (deltaTheta == 0){ //If the robot has not rotated...

      deltaDistance = deltaR;//(deltaL + deltaR) / 2; //Average change in encoder values
    
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

    Brain.Screen.clearScreen();

    Brain.Screen.setCursor(1, 2);
    Brain.Screen.setPenColor(purple);
    Brain.Screen.print("currentPos");

    Brain.Screen.setPenColor(purple);
    Brain.Screen.setCursor(2, 2);
    Brain.Screen.print(globalX);
    Brain.Screen.setCursor(2, 9);
    Brain.Screen.print(",");
    Brain.Screen.setCursor(2, 11);
    Brain.Screen.print(globalY);

    Brain.Screen.setPenColor(red);
    Brain.Screen.setCursor(4, 2);
    Brain.Screen.print("EncoderPos");
    Brain.Screen.setCursor(5, 2);
    Brain.Screen.print(getRightReading());//getRightReading());

  }

  return 1;

}

void setTarget(double x, double y){

  targetX = x;
  targetY = y;

}

void turnToTarget(double maxTurnSpeed, double kp = 0.1){ //Turn to Face Target Position

  double turnSpeed = 1;

  while (turnSpeed != 0){

    targetDeg = getDegToPosition(targetX, targetY);

    turnSpeed = turnPIDCycle(targetDeg, maxTurnSpeed, kp);

    leftDrive(-turnSpeed);
    rightDrive(turnSpeed);

    Brain.Screen.setCursor(5, 20);
    Brain.Screen.print("hello");

    task::sleep(5);

  }
  stopDrive();

}

void turnToTargetRev(double maxTurnSpeed, double kp = 0.1){ //Turn to Face Target Position

  double turnSpeed = 1;

  while (turnSpeed != 0){

    targetDeg = angleWrap(getDegToPosition(targetX, targetY) + 180);

    turnSpeed = turnPIDCycle(targetDeg, maxTurnSpeed, kp);

    leftDrive(-turnSpeed);
    rightDrive(turnSpeed);

    Brain.Screen.setCursor(5, 20);
    Brain.Screen.print("hello");

    task::sleep(5);

  }
  stopDrive();

}

void moveToTarget(double maxFwdSpeed, double maxTurnSpeed, double turnInfluence = 0.7){ //Turn to and Move to position simultaneously

  totalDistance = 0;
  stopTime = 0;

  double curFwdSpeed = 1;
  double curTurnSpeed = 1;

  targetDistance = distanceTo(targetX, targetY);

  while (curFwdSpeed != 0 && targetDistance > 3){ //&& !isStopped()) { //Turns only when farther than a few inches to prevent donuts. isStopped could be faulty

    targetDistance = distanceTo(targetX, targetY);
    targetDeg = getDegToPosition(targetX, targetY);

    curFwdSpeed = fwdPIDCycle(targetDistance, maxFwdSpeed);
    curTurnSpeed = (turnPIDCycle(targetDeg, maxTurnSpeed)) * turnInfluence;

    Brain.Screen.setCursor(11, 2);
    Brain.Screen.print(curFwdSpeed);

    leftDrive(curFwdSpeed - curTurnSpeed);
    rightDrive(curFwdSpeed + curTurnSpeed);

    task::sleep(5);
  }
  while (curFwdSpeed != 0){ //&& !isStopped()) { //Only Drive Forward after a few inches

    targetDistance = distanceTo(targetX, targetY);
    
    curFwdSpeed = fwdPIDCycle(targetDistance, maxFwdSpeed);
    curTurnSpeed = 0;

    leftDrive(curFwdSpeed);
    rightDrive(curFwdSpeed);

    task::sleep(5);

  }
  stopDrive();

}

void moveToTargetRev(double maxFwdSpeed, double maxTurnSpeed, double turnInfluence = 0.7){ //Turn to and Move to position simultaneously

  totalDistance = 0;
  stopTime = 0;

  double curFwdSpeed = 1;
  double curTurnSpeed = 1;

  targetDistance = -distanceTo(targetX, targetY);

  while (curFwdSpeed != 0 && fabs(targetDistance) > 3 && !isStopped()) { //Turns only when farther than a few inches to prevent donuts. isStopped could be faulty

    targetDistance = -distanceTo(targetX, targetY);
    targetDeg = getDegToPosition(targetX, targetY);
    targetDeg = angleWrap(targetDeg - 180);

    curFwdSpeed = (fwdPIDCycle(targetDistance, maxFwdSpeed)) * -1;
    curTurnSpeed = (turnPIDCycle(targetDeg, maxTurnSpeed)) * turnInfluence;

    leftDrive(curFwdSpeed - curTurnSpeed);
    rightDrive(curFwdSpeed + curTurnSpeed);

    task::sleep(5);
  }
  while (curFwdSpeed != 0 && !isStopped()) { //Only Drive Forward after a few inches

    targetDistance = -distanceTo(targetX, targetY);
    
    curFwdSpeed = (fwdPIDCycle(targetDistance, maxFwdSpeed)) * -1;
    curTurnSpeed = 0;

    leftDrive(curFwdSpeed);
    rightDrive(curFwdSpeed);

    task::sleep(5);

  }
  stopDrive();

}

void passTarget (double maxFwdSpeed, double maxTurnSpeed, double turnInfluence = 0.7) { //Passes Target: Allows for more control of path

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
      curTurnSpeed = (turnPIDCycle(targetDeg, maxTurnSpeed)) * turnInfluence;

    } else {

      curTurnSpeed = 0;

    }

    leftDrive(maxFwdSpeed - curTurnSpeed);
    rightDrive(maxFwdSpeed + curTurnSpeed);

    task::sleep(5);

  }

}

void passTargetRev (double maxFwdSpeed, double maxTurnSpeed, double turnInfluence = 0.7) { //Passes Target: Allows for more control of path

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
      curTurnSpeed = (turnPIDCycle(targetDeg, maxTurnSpeed)) * turnInfluence;

    } else {

      curTurnSpeed = 0;

    }

    leftDrive((maxFwdSpeed * -1) - curTurnSpeed);
    rightDrive((maxFwdSpeed * -1) + curTurnSpeed);

    task::sleep(5);

  }

}



//---------------------------------------------- OLD PID FUNCTIONS -------------------------------------------------------//

//drivePID Tuning Values
double kP = 0.03; //tuning value for proportional
double kI = 0.000000001; //tuning value for integral 
double kD = 0.031; //tuning value for derivitive 

//stopping values
double lastPosition;
double stopTimer;

//TurnPID Tuning Values
double turnkP = 0.1; 
double turnkI = 0.000001; 
double turnkD = 0.05; 

//driftPID Tuning Values
double dkP = 0.004; 
double dkI = 0; 
double dkD = 0.015;

float error; //Sensor Value - Desired Value : Position

int ticks = 0; //makes sure we aren't stopped for too long

double driftError; //error value for drift PID 
double driftPrevError; //previous error value for drift PID
double driftTotalError; //total error value for drift PID
double driftDer; //derivitive for drift PID

/*Drive PID, a function that accelerates a decelerates the robot to drive more precise distances.
Uses potential, integral, and derivitive values to accelerate the motors and decelerate them accurately
Parameters: 
  Desired value - the desired value we want the motors to turn to
  Max Speed - the maximum speed (in voltage) that the motor should run at.Default is 12 (100%)
  Error Threshold - the threshold for the error, if the error is below the threshold, stop the function. 
*/
void drivePID (int desiredValue, int maxSpeed = 12, int errorThreshold = 5){  

  //Reset motor Position
  LMMotor.setPosition(0, degrees);
  RMMotor.setPosition(0, degrees);
  LBMotor.setPosition(0, degrees);
  RBMotor.setPosition(0, degrees);
  LFMotor.setPosition(0, degrees);
  RFMotor.setPosition(0, degrees);

  int heading = Gyro.orientation(yaw, degrees); //find the current heading of the robot based on the intertial sensor values

  while (true) { //while loop that runs the PID
    
    float leftMotorPosition = (LBMotor.position(degrees) + LFMotor.position(degrees) + LMMotor.position(degrees)) / 3; //averages the left motor values
    float rightMotorPosition = (RBMotor.position(degrees) + RFMotor.position(degrees) + RMMotor.position(degrees)) / 3; //averages the right motor values
    float averagePosition = (leftMotorPosition + rightMotorPosition) / 2; //averages left and right motor values for the average position of the robot
    double integralActiveZone = 100; //only use integral when within a few inches (messes up drive if we don't)

    //if (desiredValue < 0){
    //  dkP = 0.025;
    //} else {
    //  dkP = 0.005;
    //}

    //potential
    error = averagePosition - desiredValue; 
    //derivative
    der = error - prevError;
    //Integral
    if (fabs(error) < integralActiveZone){ //if the error value is within the integral active zone..
      totalError += error; //use the integral
    } else { //else...
      totalError = 0; //set integral to 0
    }

    //Drift Potential
    driftError = Gyro.orientation(yaw, degrees) - heading;
    //Drift Derivitive
    driftDer = driftError - driftPrevError;
    //Drift Integral
    driftTotalError += driftError;

    //Variable that is added or subtracted to each motor speed to keep the robot driving straight. 
    //Based on the potential, derivitive, and integral of the robot heading
    double motorDifference = (driftError * dkP + driftDer * dkD) * 3; //+ driftTotalError * dkI

    //Variable that is applied to each motor. This value causes the motors to go move. 
    //It is affected by the potential, integral, and derivitive to make sure it accelerates and decelerates
    double lateralMotorPower = (error * kP + der * kD + totalError * kI); 

    //Makes sure the motors are staying within the speed limit (see function parameters)
    lateralMotorPower = keepInRange(lateralMotorPower, -maxSpeed, maxSpeed);

    //Makes sure the motor speed never drops below 2 so we aren't driving extremely slowly forever
    if (lateralMotorPower > 0 && lateralMotorPower < 2){
      lateralMotorPower = 2;
    } else if (lateralMotorPower < 0 && lateralMotorPower > -2){
      lateralMotorPower = -2;
    }

    //spin all the motors, lateral motor power is the base speed, motor difference allows for drift correction
    LMMotor.spin(reverse, lateralMotorPower - motorDifference, voltageUnits::volt);
    LFMotor.spin(reverse, lateralMotorPower - motorDifference, voltageUnits::volt);
    LBMotor.spin(reverse, lateralMotorPower - motorDifference, voltageUnits::volt);
    RMMotor.spin(reverse, lateralMotorPower + motorDifference, voltageUnits::volt);
    RFMotor.spin(reverse, lateralMotorPower + motorDifference, voltageUnits::volt);
    RBMotor.spin(reverse, lateralMotorPower + motorDifference, voltageUnits::volt);

    if (fabs(averagePosition - lastPosition) <= 5){

      stopTime ++;

      if (stopTime >= 50){
        break;
      }

    } else {

      stopTime = 0;

    }


    //print to screen
    //Print error value
    Brain.Screen.clearScreen();
    Brain.Screen.setCursor(5,5);
    Brain.Screen.setPenColor(green);
    Brain.Screen.print("error = ");
    Brain.Screen.setCursor(5, 10);
    Brain.Screen.print(error);
    //print speed
    Brain.Screen.setCursor(7,5);
    Brain.Screen.print("Movement Val = ");
    Brain.Screen.setCursor(7, 20);
    Brain.Screen.print(lateralMotorPower);
    //print derivative
    Brain.Screen.setCursor(6,5);
    Brain.Screen.setPenColor(red);
    Brain.Screen.print("Derivative = ");
    Brain.Screen.setCursor(6, 17);
    Brain.Screen.print(der);


    prevError = error; //set previous error for next cycle
    lastPosition = averagePosition;
    driftPrevError = driftError; //set drift previous error for next cycle
    vex::task::sleep(20); //sleep to not overheat the CPU

    if (fabs(error) < errorThreshold){ //If the error value is under the threshold
      Brain.Screen.setCursor(5, 20);
      Brain.Screen.setPenColor(purple);
      Brain.Screen.print("Break"); //print break
      break; //stop the while loop
    }

  }

  //Stop all motors once while loop is broken
  LMMotor.stop();
  LFMotor.stop();
  LBMotor.stop();
  RMMotor.stop();
  RBMotor.stop();
  RFMotor.stop();

  //set stopping mode to hold to immediately stop the motors at the correct distance
  LMMotor.setStopping(hold);
  LBMotor.setStopping(hold);
  LFMotor.setStopping(hold);
  RBMotor.setStopping(hold);
  RMMotor.setStopping(hold);
  RFMotor.setStopping(hold);

  vex::task::sleep(20); //wait 20 miliseconds before... 

  //setting all motors to coast in order to not overheat the drive
  LMMotor.setStopping(coast);
  LBMotor.setStopping(coast);
  LFMotor.setStopping(coast);
  RBMotor.setStopping(coast);
  RMMotor.setStopping(coast);
  RFMotor.setStopping(coast);
  

}

/*Turn PID, a function that accelerates a decelerates the robot to turn to precise angles based on the inertial sensor readings.
Uses potential, integral, and derivitive values to accelerate the motors and decelerate them accurately
Parameters: 
  Desired value - the desired angle we want the robot to turn to
  Max Speed - the maximum speed (in voltage) that the motor should run at.Default is 12 (100%)
*/
void turnPID (int desiredValue, int maxSpeed = 12, double tkP = 0.1){

  //Reset motor Position
  LMMotor.setPosition(0, degrees);
  RMMotor.setPosition(0, degrees);
  LBMotor.setPosition(0, degrees);
  RBMotor.setPosition(0, degrees);
  LFMotor.setPosition(0, degrees);
  RFMotor.setPosition(0, degrees);

  while (true) { //while loop that runs PID loop

    int gyroPosition = Gyro.orientation(yaw, degrees); //current robot rotation
    double integralActiveZone = 30; //only use integral when within 30 degrees 

    turnkP = tkP;

    //potential
    error = gyroPosition - desiredValue;
    //derivative
    der = error - prevError;
    //intergral
    if (fabs(error) < integralActiveZone){
      totalError += error;
    } else {
      totalError = 0;
    }

    double lateralMotorPower = (error * turnkP + der * turnkD + totalError * turnkI); //+ totalError * turnkI

    lateralMotorPower = keepInRange(lateralMotorPower, -maxSpeed, maxSpeed);

    if (lateralMotorPower > 0 && lateralMotorPower < 2){
      lateralMotorPower = 2;
    } else if (lateralMotorPower < 0 && lateralMotorPower > -2){
      lateralMotorPower = -2;
    }

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

    if (fabs(error) < 10){
      Brain.Screen.setCursor(5, 20);
      Brain.Screen.setPenColor(purple);
      Brain.Screen.print("Break");
      break;
    } else if (fabs(error) < 4){
      ticks++;
      if (ticks > 30){
        break;
      }
    } else {
      ticks = 0;
    }

  }

  LMMotor.stop(coast);
  LFMotor.stop(coast);
  LBMotor.stop(coast);
  RMMotor.stop(coast);
  RBMotor.stop(coast);
  RFMotor.stop(coast);
  
  //LMMotor.setStopping(hold);
  //LBMotor.setStopping(hold);
  //LFMotor.setStopping(hold);
  //RBMotor.setStopping(hold);
  //RMMotor.setStopping(hold);
  //RFMotor.setStopping(hold);

  vex::task::sleep(20);

  LMMotor.setStopping(coast);
  LBMotor.setStopping(coast);
  LFMotor.setStopping(coast);
  RBMotor.setStopping(coast);
  RMMotor.setStopping(coast);
  RFMotor.setStopping(coast);
  

}

//-----------------------------------Catapult Functions-------------------------------------------//

int cataFire(){

  int limitSwitchCan = 0;

  while (true) {

    if (firingCata == true) {

      Cata.spin(vex::directionType::rev, 100, vex::velocityUnits::pct);
      limitSwitchCan = limitSwitchCan + 1;
      Cata.setStopping(hold);

      if (LimitSwitch.pressing() && limitSwitchCan >= 30){
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

    if (LimitSwitch.pressing() && limitSwitchCan >= 60){
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
  CataPistons.off();

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
  vex::thread t2(updatePosition);

  //rollerSide = true;
  bothSides = false;
  skills = true;

  if (rollerSide == true){ //Roller Side Auton -----------------------------------------------------------------------------------------------------------------------------------------------
    if (LimitSwitch.pressing() == false){
      firingCata = true;
    }
    Intake.spin(forward, 40, vex::velocityUnits::pct); //Spin intake: 60 percent so it doesn't spin past the screws
    wait(0.2, sec); //wait for intake to speed up
    drivePID(112, 11, 75); //Drive to roller
    drivePID(-80, 12); //Drive away from roller
    Intake.spin(forward, 100, vex::velocityUnits::pct); //Spin intake at full speed to intake disc
    turnPID(117, 12); //Turn to face disc on auton line
    drivePID(172, 12, 50); //drive towards disc
    wait(0.5, sec); //delay in order to intake disc
    drivePID(-170, 10); //drive away from disc (-240 before)
    turnPID(88, 11); //turn to face a bit to the right of the triple stack
    Intake.spin(forward, 90, vex::velocityUnits::pct); //Spin intake at full speed to intake disc
    drivePID(-680, 12); //drive towards the low goal
    //Intake.spin(forward, 50, vex::velocityUnits::pct);
    turnPID(0); //turn
    drivePID(-335, 12); //drive towards high goal
    turnPID(-34, 12); //turn to face high goal -------------------
    //drivePID(100);
    //drivePID(-100);
    Intake.spin(reverse, 100, vex::velocityUnits::pct); //Spin intake at full speed to intake disc
    drivePID(-142); //drive towards high goal
    //wait(0.1, sec);
    CataPistons.on();
    Intake.stop(); //stop intake
    firingCata = true; //fire catapult
    wait(0.2, sec);
    //drivePID(50); //drive back slightly
    
    turnPID(41); //turn to face tripple stack
    CataPistons.off(); 
    if (LimitSwitch.pressing() == false){
      firingCata = true;
    }
    Intake.spin(forward, 100, vex::velocityUnits::pct); //start intake 
    drivePID(285, 12); //drive quickly towards the triple stack and knock it over
    wait(0.3, sec);
    //Intake.spin(reverse, 90, vex::velocityUnits::pct); //start intake 
    drivePID(-255, 9);
    turnPID(52);
    //Intake.spin(reverse, 90, vex::velocityUnits::pct); //start intake 
    //wait(0.2, sec);
    Intake.spin(forward, 100, vex::velocityUnits::pct); //start intake 
    //drivePID(935, 5);
    wait(0.3, sec);
    drivePID(275, 7);
    wait(0.1, sec);
    turnPID(52);
    drivePID(195, 12);
    wait(0.3, sec);
    drivePID(400, 12);
    turnPID(40);
    //wait(1, sec);
    //drivePID(100);
    wait(0.1, sec);
    //turnPID(15);
    //drivePID(50);
    //drivePID(-50);
    //turnPID(25, 10);
    drivePID(-700); //drive back to firing spot
    turnPID(-31, 11); //turn to face high goal -------------------------------------------------------------
    Intake.spin(reverse, 100, vex::velocityUnits::pct);
    drivePID(-50); //drive towards high goal 130 before
    //wait(0.1, sec);
    //CataPistons.on(); 
    CataPistons.on();
    firingCata = true; //fire catapult
    Intake.stop();
    wait(1, sec);
    CataPistons.off();
  } else if (bothSides == true){ //Both Sides Auton -----------------------------------------------------------------------------------------------------------------------------------------------
    
    //drivePID(500);
    //turnPID(90);
    //wait(20, sec);
    //drivePID(200);
    //turnPID(-32);
    //drivePID(50);
    //drivePID(200);
    //drivePID(200);
    //turnPID(-100);
    //drivePID(100);
    //wait(20, sec);

    if (LimitSwitch.pressing() == false){
      firingCata = true;
    }
    Intake.spin(forward, 33, vex::velocityUnits::pct); //Spin intake: 60 percent so it doesn't spin past the screws
    wait(0.2, sec); //Delay slightly so it gets up to speed
    drivePID(112, 11, 75); //Drive to roller
    wait(0.2, sec);
    drivePID(-80); //Drive away from roller
    Intake.spin(forward, 100, vex::velocityUnits::pct); //Spin intake at full speed to intake disc
    turnPID(-129); //turn to face triple stack
    drivePID(530); //drive towards triple stack
    wait(0.5, sec);
    drivePID(-200, 6);
    //drivePID(430, 4); //intake triple stack
    wait(0.1, sec);
    drivePID(200);
    wait(0.2, sec);
    drivePID(200);
    wait(0.3, sec);
    drivePID(180);
    turnPID(-45, 11); //turn to face high goal --------------------------
    drivePID(200, 12, 160);
    drivePID(-200, 12, 160);
    drivePID(200, 12, 160);
    //drivePID(-200, 12, 160);
    Intake.spin(reverse, 100, vex::velocityUnits::pct);
    drivePID(-130); //drive towards high goal
    //wait(0.5, sec);
    //turnPID(-80, 11);
    //turnPID(-35);
    //wait(0.2, sec);
    CataPistons.on();
    firingCata = true; //fire goal 
    wait(0.5, sec);
    CataPistons.off();
    Intake.spin(forward, 100, vex::velocityUnits::pct);
    drivePID(75); //drive away from high goal
    turnPID(-136); //turn to face triple line
    drivePID(1425, 7); //intake triple line 
    wait(0.5, sec);
    //drivePID(-1000); //drive back to high goal
    //turnPID(-50); //turn to face high goal --------------------
    //drivePID(-100); //drive towards high goal
    //wait(1.5, sec);
    //CataPistons.on();
    //firingCata = true; //fire catapult
    //wait(0.5, sec);
    //CataPistons.off(); 
    //drivePID(100); //drive away from high goal
    //turnPID(-160); //turn to face roller
    wait(0.1, sec);
    Intake.spin(forward, 70, vex::velocityUnits::pct);
    turnPID(-87);
    //drivePID(600); //drive and spin other roller (1400 before)
    wait(1, sec);
    drivePID(200);
    turnPID(-90);
    //drivePID(-50, 4);
  } else if (skills == true) { //Skills Auton -----------------------------------------------------------------------------------------------------------------------------------------------
    
    EncoderR.setPosition(0, degrees);
    globalX = 8;
    globalY = 30;

    if (LimitSwitch.pressing() == false){
      firingCata = true;
    }
    Intake.spin(forward, 100, vex::velocityUnits::pct); //Spin intake: 80 percent so it doesn't spin back to blue
    wait(0.1, sec); //Delay slightly so it gets up to speed
    //drivePID(29);
    drivePID(180, 11, 145); //Drive to roller
    wait(0.2, sec); //wait for roller to spin
    Intake.spin(forward, 40, vex::velocityUnits::pct);
    drivePID(-50);
    drivePID(150, 11, 95); //Drive to roller

    setTarget(40, -40);
    moveToTarget(12, 12, 0.4);
    setTarget(108, 12);
    turnToTarget(12);
    setTarget(45, -16);
    passTarget(12, 12);
    setTarget(0, 0);
    moveToTarget(12, 12);
    //setTarget(-24, 120);
    //turnToTarget(12);

    wait(100, sec); 


    
    wait(0.2, sec);
    drivePID(-150); //Drive away from roller
    Intake.spin(forward, 100, vex::velocityUnits::pct); //Spin intake at full speed to intake disc
    wait(0.1, sec);

    turnPID(110); //Turn to face disc on auton line
    drivePID(200); //drive towards disc
    
    turnPID(135, 12, 0.15); //turn towards 2nd roller
    //wait(1, sec);
    Intake.spin(forward, 70, vex::velocityUnits::pct);
    drivePID(230); //drive towards 2nd roller
    turnPID(90, 12, 0.12); //face roller
    drivePID(185, 8); //spin roller
    //wait(0.3, sec);
    drivePID(-90); //drive away from roller
    Intake.spin(forward, 100, vex::velocityUnits::pct);

    turnPID(-1); //turn towards high goal
    drivePID(-990, 9); //drive towards high goal
    turnPID(14, 12, 0.17); //turn to face high goal
    wait(0.5, sec);
    firingCata = true; //fire catapult ------------------------------------------------------------------- 1st shot
    wait(0.5, sec);
    turnPID(-10, 12, 0.17); //turn to face foward
    
    drivePID(855, 10); //drive to where we should be for triple line
    turnPID(-132); //turn to face triple line
    //drivePID(400);
    //drivePID(400); //was used for picking up tripple line, wasn't as consistent as driving slowly
    //drivePID(400);
    drivePID(1275, 6); //drive and intake triple line
    wait(1, sec); //wait to intake third disc
    
    turnPID(-51); //turn to face high goal
    drivePID(-140);
    firingCata = true; //fire catapult ------------------------------------------------------------------- 2nd shot
    wait(0.5, sec);
    drivePID(127);
    
    turnPID(-135); //turn to face triple stack
    drivePID(550, 8); //drive slow and knock over triple stack
    wait(0.5, sec);
    drivePID(-200, 4);
    wait(0.5, sec);
    drivePID(200);
    wait(0.5, sec);
    drivePID(200);
    wait(0.6, sec);
    drivePID(365);
    turnPID(-184, 11, 0.12); //turn to face roller
    Intake.spin(forward, 70, vex::velocityUnits::pct);
    drivePID(157); //drive towards and spin roller
    drivePID(-150); //drive away from roller
    Intake.spin(forward, 100, vex::velocityUnits::pct);
    //drivePID(500, 4); //drive at a moderate speed to intake discs
    wait(1, sec); //wait to intake discs

    turnPID(-91);//, 12, 0.12); //turn to face high goal
    drivePID(-780); //drive towards high goal
    turnPID(-100, 12, 0.12); //turn to face high goal
    wait(0.5, sec);
    firingCata = true; //fire catapult ------------------------------------------------------------------- 3rd shot
    wait(0.5, sec);
    turnPID(-93);

    drivePID(375); //drive towards 4th tripple stack
    turnPID(-45, 8); //turn to face tripple stack
    drivePID(450); //knock over tripple stack
    wait(0.5, sec);
    drivePID(-200, 4); //drive back
    wait(0.5, sec);
    drivePID(300); //intake 
    wait(0.5, sec);
    drivePID(300); //intake
    wait(0.6, sec);
    drivePID(-200); //drive back to 4 tiles
    
    turnPID(-95, 12, 0.13); //turn towards roller
    Intake.spin(forward, 100, vex::velocityUnits::pct);
    drivePID(470); //drive to and spin roller
    wait(0.2, sec);
    drivePID(-100); //drive away from roller
    Intake.spin(forward, 100, vex::velocityUnits::pct);

    turnPID(-184); //turn towards high goal
    drivePID(-800, 9); //drive towards high goal
    turnPID(-170, 12, 0.13); //turn to face high goal
    wait(0.5, sec);
    firingCata = true; //fire catapult ------------------------------------------------------------------- 4th shot
    wait(0.5, sec);

    drivePID(775); //drive to where we should be for tripple line
    turnPID(44, 5); //turn to face tripple line
    wait(0.2, sec);
    drivePID(1300, 5); //drive and intake tripple line
    
    turnPID(130); //turn to face high goal
    drivePID(-100); //drive towards high goal
    firingCata = true; //fire catapult ------------------------------------------------------------------- 5th shot
    wait(0.5, sec);
    drivePID(85); //drive away from high goal

    turnPID(44); //turn to face tripple stack
    drivePID(600, 8); //drive slow and knock over triple stack
    wait(0.5, sec);
    drivePID(-200, 4);
    wait(0.5, sec);
    drivePID(200);
    wait(0.5, sec);
    drivePID(400);
    wait(0.6, sec); 
    turnPID(95); //turn towards high goal
    drivePID(-730); //drive towards high goal
    //turnPID(90, 12, 0.13); //turn to face high goal
    firingCata = true; //fire catapult ------------------------------------------------------------------- 6th shot
    wait(0.5, sec);
    
    turnPID(87, 12, 0.13); //turn to face 4 tiles
    drivePID(1200);

    turnPID(45, 12, 0.15); //turn to face field
    //drivePID(100);
    ExpansionPnuematics1.off(); //fire expansion
    ExpansionPnuematics2.off(); //fire expansion

    /*

    */

  } else { //Non Roller Side Auton ----------------------------------------------------------------------------------------------------------------------------------------------------
    if (LimitSwitch.pressing() == false){
      firingCata = true;
    }
    Intake.spin(forward, 100, vex::velocityUnits::pct); //Spin intake to intake disc
    drivePID(500, 8); //drive towards disc
    wait(0.1, sec);
    //turnPID(-150, 10); //turn to face high goal --------------------------------------------------------------
    //wait(2, sec);
    //drivePID(-150, 12, 110); //drive towards high goal in at quick speed to add more velocity to discs
    //CataPistons.on();
    //firingCata = true; //fire catapult
    //wait(0.5, sec);
    //drivePID(25); //drive away from high goal
    //CataPistons.off();
    //turnPID(45, 6); //turn to face disc on auton line
    //drivePID(150, 10); //drive towards and intake disc
    //wait(0.5, sec); //delay to intake disc
    //drivePID(-100); //drive back
    turnPID(-42, 6); //turn to face double line (already intaked the first)
    drivePID(600, 6); //drive and intake double line
    wait(1, sec); //delay to intake discs
    turnPID(-135, 7); //turn to face high goal --------------------------------------------------------------
    drivePID(-50); //drive towards high goal
    drivePID(150);
    drivePID(-150);
    wait(0.5, sec);
    CataPistons.on();
    firingCata = true; //fire catapult
    wait(0.5, sec);
    CataPistons.off();
    drivePID(50); //drive away from goal
    turnPID(136, 6); //turn to face 
    drivePID(1300); //drive to and spin roller
    Intake.spin(forward, 60, vex::velocityUnits::pct);
    wait(1, sec);
    drivePID(100);
    wait(1, sec);
    Intake.stop();
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

  int intakeSpeed = 100;

  //Sensor Variables
  bool bDetect; //Vision Sensor is Detecting Object
  
  while (1) {

    //Clear Scren to prepare for text
    Brain.Screen.clearScreen();

    //---------------------------------------Drivecode---------------------------------------//
    double turnVal = Controller1.Axis1.position(pct); //turn value, based on the value of Axis 1 (right stick left and right axis)
    double forwardVal = Controller1.Axis3.position(pct) * multiplier; //forward value, based on the value of axis 3 (left stick and up and down axis)

    double turnVolts = turnVal * -0.12; //converts axis value into voltage by multiplying it by -0.12 (the motors can go up to 12 volts)
    /*converts forward axis value to volts by multiplying by 0.12. Also multiplies by the absolute value of turn volts multiplied by the
    turn importance. This decreases the forward value ammount based on the turning ammount. This allows us to turn and move forward
    simultaneously, which helps a lot with controling the drive to a precision*/
    double forwardVolts = forwardVal * 0.12 * (1-(std::abs(turnVolts)/12) * turnImportance); 

    //Turn all motors. Left motors are forward volts - turn volts to make the left motors spin in reverse while turning, right motors are opposite
    LMMotor.spin(fwd, ((forwardVolts - turnVolts)) , voltageUnits::volt);
    LFMotor.spin(fwd, ((forwardVolts - turnVolts)) , voltageUnits::volt);
    LBMotor.spin(fwd, ((forwardVolts - turnVolts)) , voltageUnits::volt);
    RMMotor.spin(fwd, ((forwardVolts +  turnVolts)), voltageUnits::volt);
    RFMotor.spin(fwd, ((forwardVolts + turnVolts)) , voltageUnits::volt);
    RBMotor.spin(fwd, ((forwardVolts + turnVolts)) , voltageUnits::volt);

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

      //motorCoast = false;

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
    if (Controller1.ButtonL2.pressing()){ //if button L2 is being pressed...

      Intake.spin(vex::directionType::fwd, intakeSpeed, vex::velocityUnits::pct); //spin the intake forward

    } else if (Controller1.ButtonL1.pressing()){ //if button L1 is being pressed...

      Intake.spin(vex::directionType::rev, intakeSpeed, vex::velocityUnits::pct); //spin the intake reverse

    } else { //if neither button is being pressed...

      Intake.stop(); //stop the intake

    }

    if (Controller1.ButtonX.pressing()){
      intakeSpeed = 60;
    }

    if (Controller1.ButtonDown.pressing()){
      intakeSpeed = 100;
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
