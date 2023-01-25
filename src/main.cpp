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
// ---- END VEXCODE CONFIGURED DEVICES ----

#include "vex.h"
#include "cmath"

using namespace vex;






// A global instance of competition
competition Competition;

// define your global instances of motors and other devices here


//--------------------------------Variables----------------------------------//
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

//----------------------------------------------PID-------------------------------------------------------//

//drivePID Tuning Values
double kP = 0.005;
double kI = 0.0000000000000001;
double kD = 0.0003;

//TurnPID Tuning Values
double turnkP = 0.004;
double turnkI = 0.000000001; //0.00000000000000000001
double turnkD = 0.0003;

int error; //Sensor Value - Desired Value : Position
int prevError; //Position 20ms ago
int der; //derivative : Speed
int totalError; //

int ticks = 0;



void drivePID (int desiredValue){

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
    int driftVal = heading - Gyro.orientation(yaw, degrees);
    int motorDifference = driftVal / 2;

    //potential
    error = averagePosition - desiredValue;
    //derivative
    der = error - prevError;
    //intergral
    totalError += error; 

    double lateralMotorPower = (error * kP + der * kD + totalError * kI) * 80;//+ totalError * kI

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

void turnPID (int desiredValue){

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

    double lateralMotorPower = (error * turnkP + der * turnkD) * 80; //+ totalError * turnkI

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

  //rollerSide = false;
  //bothSides = false;
  //skills = true;

  if (rollerSide == true){
    firingCata = true;
    drivePID(-750);
    wait (0.2, sec);
    turnPID(-45);
    turnPID(-45);
    wait(0.2, sec);
    drivePID(-200);
    wait(0.1, sec);
    fireCata();
    Intake.spin(forward, 100, vex::velocityUnits::pct);
    wait(1, sec);
    firingCata = true;
    Intake.spin(reverse, 100, vex::velocityUnits::pct);
    wait(1, sec);
    turnPID(43);
    turnPID(43);
    wait(0.2, sec);
    drivePID(1250);
    turnPID(15);
    drivePID(100);
    turnPID(0);
    wait(0.2, sec);
    Intake.spin(reverse, 100, vex::velocityUnits::pct);
    drivePID(200);
    wait(1,sec);
    Intake.stop();
    drivePID(-50);
  } else if (bothSides == true){
    firingCata = true;
    Intake.spin(reverse, 100, vex::velocityUnits::pct);
    drivePID(50);
    wait(0.1, sec);
    Intake.stop();
    drivePID(-100);
    wait(0.2, sec);
    turnPID(45);
    wait(0.2, sec);
    drivePID(-1000);
    wait(0.2, sec);
    turnPID(-45);
    wait(0.2, sec);
    drivePID(-100);
    fireCata();
    Intake.spin(forward, 100, vex::velocityUnits::pct);
    wait(1, sec);
    Intake.spin(reverse, 100, vex::velocityUnits::pct);
    firingCata = true;
    wait(1, sec);
    turnPID(-130);
    turnPID(-130);
    wait(0.1, sec);
    drivePID(1700);
    wait(0.2, sec);
    turnPID(-90);
    wait(0.2, sec);
    Intake.spin(reverse, 100, vex::velocityUnits::pct);
    drivePID(100);
  } else if (skills == true) {
    firingCata = true;
    Intake.spin(reverse, 100, vex::velocityUnits::pct);
    drivePID(50); //drive Forward and Spin Roller
    wait(1, sec);
    drivePID(20);
    wait(1, sec);
    drivePID(-100);
    wait(0.2, sec);
    drivePID(90);
    wait(1, sec);
    Intake.stop();
    drivePID(-385); //drive towards other roller
    wait(0.2, sec);
    //turnPID(90);
    //wait(0.2, sec);
    //Intake.spin(forward, 100, vex::velocityUnits::pct);
    //drivePID(200);
    //wait(0.2, sec);
    //turnPID(0);
    //turnPID(0);
    //Intake.stop();
    //wait(0.2, sec);
    //drivePID(-230);
    //wait(0.2, sec);
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

  } else {
    firingCata = true;
    drivePID(-750);
    wait (0.2, sec);
    turnPID(25);
    turnPID(25);
    wait(0.2, sec);
    drivePID(-100);
    wait(0.1, sec);
    fireCata();
    Intake.spin(forward, 100, vex::velocityUnits::pct);
    wait(1, sec);
    firingCata = true;
    Intake.spin(reverse, 100, vex::velocityUnits::pct);
    wait(1, sec);
    turnPID(-42);
    turnPID(-42);
    wait(0.2, sec);
    drivePID(950);
    turnPID(-15);
    drivePID(100);
    turnPID(-0);
    wait(0.2, sec);
    Intake.spin(reverse, 100, vex::velocityUnits::pct);
    drivePID(200);
    wait(1,sec);
    Intake.stop();
    drivePID(-50);
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

    //---------------------------------------Rollers---------------------------------------//
    if (Controller1.ButtonR1.pressing()){ //Spin Roller Slowly
      
      if (multiplier == 1) {
        multiplier = -1;
        wait(0.15, sec);
      } else {
        multiplier = 1;
        wait(0.15, sec);
      }
    }

    


    //---------------------------------------Expansion---------------------------------------//
    if (Controller1.ButtonB.pressing()){
      ExpansionPnuematics1.off();
      ExpansionPnuematics2.off();
      Controller1.rumble(rumbleShort);
    }

    if (Controller1.ButtonY.pressing()){
      ExpansionPnuematics1.on();
      ExpansionPnuematics2.on();
      Controller1.rumble(rumbleShort);
    }

    //---------------------------------------Catapult---------------------------------------//

    limitSwitchVar = LimitSwitch.pressing();//Variable for Limit Switch

    //Cata Stop
    if (limitSwitchVar == true && limitSwitchCan >= 20) {
      Controller1.rumble(rumbleLong);
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
