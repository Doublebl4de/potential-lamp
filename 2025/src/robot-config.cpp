#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

// A global instance of brain used for printing to the V5 Brain screen
brain  Brain;

// VEXcode device constructors
motor leftFrontDrive = motor(PORT1, ratio6_1, true);
motor leftMiddleDrive = motor(PORT2, ratio6_1, true);
motor leftBackDrive = motor(PORT4, ratio6_1, true);
motor rightFrontDrive = motor(PORT10, ratio6_1, false);
motor rightMiddleDrive = motor(PORT9, ratio6_1, false);
motor rightBackDrive = motor(PORT8, ratio6_1, false);
controller Controller1 = controller(primary);
inertial gyroZeppeli = inertial(PORT19);
motor backIntake = motor(PORT17, ratio18_1, true);
motor middleIntake = motor(PORT7, ratio18_1, false);
motor frontIntake = motor(PORT5, ratio18_1, false);

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
