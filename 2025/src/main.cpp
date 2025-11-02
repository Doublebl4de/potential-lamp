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
// leftFrontDrive       motor         1              
// leftMiddleDrive      motor         2              
// leftBackDrive        motor         4              
// rightFrontDrive      motor         10             
// rightMiddleDrive     motor         9              
// rightBackDrive       motor         8              
// Controller1          controller                   
// gyroZeppeli          inertial      18             
// backIntake           motor         17             
// middleIntake         motor         7              
// frontIntake          motor         5              
// bar                  digital_out   A              
// stick                digital_out   B              
// ---- END VEXCODE CONFIGURED DEVICES ----


#include "vex.h"


using namespace vex;




// A global instance of competition
competition Competition;


// define your global instances of motors and other devices here


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
 gyroZeppeli.calibrate();


 // All activities that occur before the competition starts
 // Example: clearing encoders, setting servo positions, ...
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




void drive(double left,double right){
 leftFrontDrive.spin(forward, left, percent);
 leftMiddleDrive.spin(forward, left, percent);
 leftBackDrive.spin(forward, left, percent);
 rightFrontDrive.spin(forward, right, percent);
 rightMiddleDrive.spin(forward, right, percent);
 rightBackDrive.spin(forward, right, percent);
}
double clip_num(double input, double max, double min){
 if (input>max){
   return max;
 }
 else if (input < min) {
   return min;
 }
 return input;
}
double pi = 3.14159265358979323846264338327950288419716939937510582097494459230781640628620899862803482534211706798214808651328230664709384460955058223172;
void go(double dist, double power){
 leftBackDrive.setPosition(0,degrees);
 rightBackDrive.setPosition(0,degrees);
 // 3/4 gear ratio, wheel circumference is 2.75 inch * pi
 double totalDegrees = (dist*360*4)/(2.75*pi*3); //CHECKED
 double proportionalControllerConstant = 0.1;//abs(100/dist)
 int time = 0;
 while (true){
   double left_error = totalDegrees - leftBackDrive.position(degrees);
   Brain.Screen.clearScreen();
   Brain.Screen.clearLine();
   Brain.Screen.print(leftBackDrive.position(turns));
   Brain.Screen.print("    ");
   Brain.Screen.print(left_error);
  
   double right_error = totalDegrees - rightBackDrive.position(degrees);
   double left_output = clip_num(left_error*proportionalControllerConstant,power,-power);
   double right_output = clip_num(right_error*proportionalControllerConstant,power,-power);
   drive(left_output,right_output);
  


   if ((abs(left_error) < 2.0 and abs(right_error) < 2.0)){
     drive(0,0); //REDUNDANT
     time ++;
   }
   if (time >= 12){
     drive(0,0);
     break;
   }
   double left_average = ((leftFrontDrive.velocity(percent)+leftMiddleDrive.velocity(percent)+leftBackDrive.velocity(percent))/3);
   double right_average = ((rightFrontDrive.velocity(percent)+rightMiddleDrive.velocity(percent)+rightBackDrive.velocity(percent))/3);
   if (abs(left_average) < 2 and abs(right_average) < 2){
     time ++;
   wait(0.01, seconds); //IDK
   }
   /*
   if (time == 5){
     drive(0,0);
     break;
   }
   */
 }
}
void turn(double target, double power){
 int time = 0;
 double proportionalControllerConstant = 0.5;
 while(true){
   double error = target - gyroZeppeli.rotation();
   int output = clip_num(error * proportionalControllerConstant, power, -power);
   drive(output,-output);
   if(abs(leftBackDrive.velocity(percent)) < 2 and abs(rightBackDrive.velocity(percent)) < 2){
     time += 0;
   } else {
     time = 0;
   }
   if (abs(error) <= 2.5){
     time += 10;
   }
   if (time >= 40){
     drive(0,0);
     gyroZeppeli.setRotation(0, degrees);


     break;
   }
   Brain.Screen.clearScreen();
   Brain.Screen.clearLine();
   Brain.Screen.print(error);
   Brain.Screen.print(gyroZeppeli.rotation());
   wait(0.1,seconds);
 }
}
void quickTurn(double target, double power){
 int time = 0;
 double proportionalControllerConstant = 0.5;
 while(true){
   double error = target - gyroZeppeli.rotation();
   int output = clip_num(error * proportionalControllerConstant, power, -power);
   drive(output,-output);
   if(abs(leftBackDrive.velocity(percent)) < 2 and abs(rightBackDrive.velocity(percent)) < 2){
     time += 0;
   } else {
     time = 0;
   }
   if (abs(error) <= 5){
     time += 10;
   }
   if (time >= 20){
     drive(0,0);
     gyroZeppeli.setRotation(error, degrees);


     break;
   }
   Brain.Screen.clearScreen();
   Brain.Screen.clearLine();
   Brain.Screen.print(error);
   Brain.Screen.print(gyroZeppeli.rotation());
   wait(0.1,seconds);
 }
}
void rightSwing(double target, double power){
 int time = 0;
 double proportionalControllerConstant = 1.2;
 while(true){
   double error = target - gyroZeppeli.rotation();
   int output = clip_num(error * proportionalControllerConstant, power, -power);
   drive(0,-output);
   if(abs(rightBackDrive.velocity(percent)) < 7){
     time += 10;
   } else {
     time = 0;
   }
   if (abs(error) <= 2.5){
     time += 10;
   }
   if (time >= 80){
     drive(0,0);
     gyroZeppeli.setRotation(0, degrees);


     break;
   }
   Brain.Screen.clearScreen();
   Brain.Screen.clearLine();
   Brain.Screen.print(error);
   wait(0.1,seconds);
 }
}
void leftSwing(double target, double power){
 int time = 0;
 double proportionalControllerConstant = 0.7;
 while(true){
   double error = target - gyroZeppeli.rotation();
   int output = clip_num(error * proportionalControllerConstant, power, -power);
   drive(output,0);
   if(leftBackDrive.velocity(percent) == 0){
     time += 10;
   } else {
     time = 0;
   }
   if (abs(error) <= 2.5){
     time += 10;
   }
   if (time >= 80){
     drive(0,0);
     gyroZeppeli.setRotation(0, degrees);


     break;
   }
   Brain.Screen.clearScreen();
   Brain.Screen.clearLine();
   Brain.Screen.print(error);
   wait(0.1,seconds);
 }
}
void turnorest(double target, double power){
 int time = 0;
 double proportionalControllerConstant = 0.7;
 while(true){
   double error = target - gyroZeppeli.rotation();
   int output = clip_num(error * proportionalControllerConstant, power, -power);
   drive(output,-output);
   if(leftBackDrive.velocity(percent) == 0 and rightBackDrive.velocity(percent) == 0){
     time += 10;
   } else {
     time = 0;
   }
   if (abs(error) <= 2.5){
     time += 10;
   }
   if (time >= 80){
     drive(0,0);


     break;
   }
   Brain.Screen.clearScreen();
   Brain.Screen.clearLine();
   Brain.Screen.print(error);
   wait(0.1,seconds);
 }
}
void setturn(double target, double power,double reset){
 int time = 0;
 double proportionalControllerConstant = 0.7;
 while(true){
   double error = target - gyroZeppeli.rotation();
   int output = clip_num(error * proportionalControllerConstant, power, -power);
   drive(output,-output);
   if(leftBackDrive.velocity(percent) == 0 and rightBackDrive.velocity(percent) == 0){
     time += 10;
   } else {
     time = 0;
   }
   if (abs(error) <= 2.5){
     time += 10;
   }
   if (time >= 80){
     drive(0,0);
     gyroZeppeli.setRotation(reset, degrees);


     break;
   }
   Brain.Screen.clearScreen();
   Brain.Screen.clearLine();
   Brain.Screen.print(error);
   wait(0.1,seconds);
 }
}
void mintake(double power){
 frontIntake.spin(reverse,power,percent);
 middleIntake.spin(forward,power*0.75,percent);
}
void hintake(double power){
 frontIntake.spin(forward,power,percent);
 middleIntake.spin(forward,power,percent);
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


void test1(){
 //wait(4,sec);
 go(29.5,100);
 turn(-90,100);
 middleIntake.spin(forward,25,percent);
 frontIntake.spin(forward,25,percent);
 go(19,100);
 middleIntake.spin(forward,100,percent);
 frontIntake.spin(forward,100,percent);
 wait(1,sec);
 middleIntake.stop();
 frontIntake.stop();
 go(-19,100);
 turn(180,100);
 bar.set(true);
 wait(0.4,sec);
 middleIntake.spin(forward,15,percent);
 go(14,100);
 //quickTurn(0,100);
 wait(0.20,sec);
 go(-10,100);
 //frontIntake.spin(forward,10,percent);
 middleIntake.spin(forward,100,percent);
 wait(0.55,sec);
 middleIntake.spin(forward,15,percent);
 go(16,100);
 //quickTurn(0,100);
 wait(0.20,sec);
 go(-10,100);
 go(2,100);
 middleIntake.spin(forward,100,percent);
 wait(0.45,sec);
 middleIntake.spin(forward,15,percent);
 go(16,100);
 //quickTurn(0,100);
 wait(0.20,sec);
 bar.set(false);
 go(-10,100);
 go(2,100);
 middleIntake.spin(forward,100,percent);
 wait(0.30,sec);
 middleIntake.stop();
 frontIntake.stop();
 turn(0,100);
 turn(180,100);
 go(16,100);
 middleIntake.spin(forward,100,percent);
 frontIntake.spin(forward,100,percent);
  /*
 go(14,100);
 go(-10,100);
 middleIntake.spin(forward,100,percent);
 wait(1.5,sec);
 middleIntake.stop();
 go(10,100);
 go(-10,100);
 middleIntake.spin(forward,100,percent);
 wait(1.5,sec);
 middleIntake.stop();
 */


 /*
 go(-5,100);
 turn(180,100);
 go(24,100);
 middleIntake.spin(forward,100,percent);
 frontIntake.spin(forward,-100,percent);
 wait(4,sec);
 go(-8,100);
 turn(-45,100);
 middleIntake.spin(forward,100,percent);
 go(24,100);
 frontIntake.spin(forward,-100,percent);
 *///turn(90,100);
}


void pright(){
 stick.set(true);
 middleIntake.spin(forward,100,percent);
 //frontIntake.spin(forward,-100,percent);
 bar.set(true);
 wait(430,msec);
 go(30,80);
 wait(1.5,sec);
 go(3,100);
 wait(1,sec);
 go(3,100);
 wait(1.5,sec);
  go(-2,100);
 bar.set(false);
 go(-7,100);
 turn(-75,100);
 go(13.35,70);
 //frontIntake.spin(forward,100,percent);
 //wait(0.5,sec);
 frontIntake.spin(forward,-100,percent);
 middleIntake.spin(forward,-90,percent);


 go(-1.5,10);
}
void pleft(){
 stick.set(true);
 middleIntake.spin(forward,100,percent);
 //frontIntake.spin(forward,-100,percent);
 bar.set(true);
 wait(430,msec);
 go(30,82);
 wait(1.5,sec);
 go(3,100);
 wait(1,sec);
 go(3,100);
 wait(1.5,sec);
 //bar.set(false);
 go(-10,100);
 turn(73,100);
 go(14,70);
 stick.set(false);
 //frontIntake.spin(forward,100,percent);
 //wait(0.5,sec);
 middleIntake.spin(forward,-50,percent);
 wait(50,msec);
 frontIntake.spin(forward,-70,percent);
 middleIntake.spin(forward,70,percent);
 wait(3,sec);
 go(-2.5,10);
}
void autonomous(void) {
 pleft();
 // ..........................................................................
 // Insert autonomous user code here.
 // ..........................................................................
}
//ThisIsPascelCasing thisIsCamelCasing
bool muptake = false;
bool mdowntake = false;
bool huptake = false;
bool hdowntake = false;
bool barUp = false;
bool bardown = false;
bool ispressingbar = false;
bool stickUp = false;
bool stickdown = false;
bool ispressingstick = false;
bool reversedir = false;
bool reversepress = false;
void usercontrol(void) {
 // User control code here, inside the loop
 bar.set(false);
 gyroZeppeli.calibrate();
 while (1) {
   if (reversedir == false){
     drive(Controller1.Axis3.value(),Controller1.Axis2.value());
   }else if (reversedir == true){
     drive(-Controller1.Axis2.position(),-Controller1.Axis3.position());
   }
   if (Controller1.ButtonX.pressing() == true and reversepress == false){
     reversedir = not reversedir;
     reversepress = true;
   }else if (Controller1.ButtonX.pressing() == false){
     reversepress = false;
   }
  
   if (Controller1.ButtonL2.pressing()){
     middleIntake.spin(forward,-100,percent);
   }else if(Controller1.ButtonL1.pressing()){
     middleIntake.spin(forward,100,percent);
   }else{
     middleIntake.spin(forward,0,percent);
   }
   if (Controller1.ButtonR2.pressing()){
     frontIntake.spin(forward,-100,percent);
   }else if(Controller1.ButtonR1.pressing()){
     frontIntake.spin(forward,100,percent);
    
   }else{
     frontIntake.spin(forward,0,percent);
   }
   if (Controller1.ButtonUp.pressing()){
     if (ispressingbar == false){
       ispressingbar = true;
       barUp = not barUp;
     }
   }
   else if(Controller1.ButtonUp.pressing() == false){
     ispressingbar = false;
   }
   if (barUp == true and bardown == false){
     bardown = true;
     bar.set(true);


   }else if(barUp == false){
     bardown = false;
     bar.set(false);
   }
   if (Controller1.ButtonDown.pressing()){
     if (ispressingstick == false){
       ispressingstick = true;
       stickUp = not stickUp;
     }
   }
   else if(Controller1.ButtonDown.pressing() == false){
     ispressingstick = false;
   }
   if (stickUp == true and stickdown == false){
     stickdown = true;
     stick.set(true);


   }else if(stickUp == false){
     stickdown = false;
     stick.set(false);
   }
   Brain.Screen.clearScreen();
   Brain.Screen.clearLine();
   Brain.Screen.print(Controller1.Axis2.value());
   Brain.Screen.print(" ");
   Brain.Screen.print(rightMiddleDrive.value());
   wait(0.005,sec);
 }
  
   // This is the main execution loop for the user control program.
   // Each time through the loop your program should update motor + servo
   // values based on feedback from the joysticks.


   // ........................................................................
   // Insert user code here. This is where you use the joystick values to
   // update your motors, etc.
   // ........................................................................
  
   wait(20, msec); // Sleep the task for a short amount of time to
                   // prevent wasted resources.
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

