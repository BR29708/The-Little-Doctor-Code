using namespace vex;

extern brain Brain;

using signature = vision::signature;

// VEXcode devices
extern controller Controller1;
extern motor RMMotor;
extern motor RFMotor;
extern motor RBMotor;
extern motor LMMotor;
extern motor LFMotor;
extern motor LBMotor;
extern motor Intake;
extern motor Cata;
extern led ExpansionPnuematics1;
extern led ExpansionPnuematics2;
extern limit LimitSwitch;
extern inertial Gyro;
extern optical Color;
extern signature VisionSensor__BLUE_GOAL;
extern signature VisionSensor__RED_GOAL;
extern signature VisionSensor__SIG_3;
extern signature VisionSensor__SIG_4;
extern signature VisionSensor__SIG_5;
extern signature VisionSensor__SIG_6;
extern signature VisionSensor__SIG_7;
extern vision VisionSensor;
extern led CataPistons;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void  vexcodeInit( void );