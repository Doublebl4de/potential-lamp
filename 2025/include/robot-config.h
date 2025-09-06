using namespace vex;

extern brain Brain;

// VEXcode devices
extern motor leftFrontDrive;
extern motor leftMiddleDrive;
extern motor leftBackDrive;
extern motor rightFrontDrive;
extern motor rightMiddleDrive;
extern motor rightBackDrive;
extern controller Controller1;
extern inertial gyroZeppeli;
extern motor backIntake;
extern motor middleIntake;
extern motor frontIntake;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void  vexcodeInit( void );
