#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

// A global instance of brain used for printing to the V5 Brain screen
brain  Brain;

// VEXcode device constructors
triport Expander13 = triport(PORT13);
controller Controller1 = controller(primary);
motor RMMotor = motor(PORT10, ratio18_1, false);
motor RFMotor = motor(PORT9, ratio18_1, false);
motor RBMotor = motor(PORT21, ratio18_1, false);
motor LMMotor = motor(PORT1, ratio18_1, true);
motor LFMotor = motor(PORT2, ratio18_1, true);
motor LBMotor = motor(PORT14, ratio18_1, true);
motor Intake = motor(PORT8, ratio18_1, true);
motor Cata = motor(PORT15, ratio36_1, true);
led ExpansionPnuematics1 = led(Brain.ThreeWirePort.A);
led ExpansionPnuematics2 = led(Brain.ThreeWirePort.B);
limit LimitSwitch = limit(Brain.ThreeWirePort.C);
inertial Gyro = inertial(PORT3);
optical Color = optical(PORT17);
/*vex-vision-config:begin*/
signature VisionSensor__BLUE_GOAL = signature (1, -3529, -1927, -2728, 6343, 11565, 8954, 3, 0);
signature VisionSensor__RED_GOAL = signature (2, 9513, 10713, 10113, -2341, -1381, -1861, 2.5, 0);
vision VisionSensor = vision (PORT16, 150, VisionSensor__BLUE_GOAL, VisionSensor__RED_GOAL);
/*vex-vision-config:end*/
led CataPistons = led(Brain.ThreeWirePort.D);
encoder EncoderL = encoder(Brain.ThreeWirePort.E);
encoder EncoderR = encoder(Brain.ThreeWirePort.G);
encoder EncoderB = encoder(Expander13.A);

// VEXcode generated functions
// define variable for remote controller enable/disable
bool RemoteControlCodeEnabled = true;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void vexcodeInit( void ) {
  // nothing to initialize
}