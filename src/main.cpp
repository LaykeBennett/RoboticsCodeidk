#include "main.h"
#include "lemlib/api.hpp"
#include "pros/misc.h"


// controller
pros::Controller controller(pros::E_CONTROLLER_MASTER);

// drive motors
pros::Motor lF(-8, pros::E_MOTOR_GEARSET_06); // left front motor. port 8, reversed
pros::Motor lB(-7, pros::E_MOTOR_GEARSET_06); // left Back motor. port 7, reversed
pros::Motor rF(11, pros::E_MOTOR_GEARSET_06); // right front motor. port 11
pros::Motor rB(5, pros::E_MOTOR_GEARSET_06); // right Back motor. port 5

// motor groups
pros::MotorGroup leftMotors({lF, lB}); // left motor group
pros::MotorGroup rightMotors({rF, rB}); // right motor group

// Inertial Sensor on port 11
pros::Imu imu(21);

//Declaring every other motor and Pistons
pros::Motor Intake(10);
pros::Motor Lift(3);
pros::Motor Slapper(2);
pros::Motor Lift2(-19);
pros::ADIDigitalOut WingPistons('A');
bool WingBool = false;
// LED's
pros::ADILED LedStrip('B', 64);
pros::ADILED LedStrip2('D',64);


// drivetrain settings
lemlib::Drivetrain drivetrain(&leftMotors, // left motor group
                              &rightMotors, // right motor group
                              12.25, // 12.25 inch track width
                              lemlib::Omniwheel::OLD_325, // using old 3.25" omnis
                              360, // drivetrain rpm is 360
                              2 // chase power is 2. If we had traction wheels, it would have been 8
);

// lateral motion controller
lemlib::ControllerSettings linearController(10, // proportional gain (kP)
                                            30, // derivative gain (kD)
                                            1, // small error range, in inches
                                            100, // small error range timeout, in milliseconds
                                            3, // large error range, in inches
                                            500, // large error range timeout, in milliseconds
                                            20 // maximum acceleration (slew)
);

// angular motion controller
lemlib::ControllerSettings angularController(2, // proportional gain (kP)
                                             10, // derivative gain (kD)
                                             1, // small error range, in degrees
                                             100, // small error range timeout, in milliseconds
                                             3, // large error range, in degrees
                                             500, // large error range timeout, in milliseconds
                                             20 // maximum acceleration (slew)
);

// sensors for odometry
// note that in this example we use internal motor encoders, so we don't pass vertical tracking wheels
lemlib::OdomSensors sensors(nullptr, // vertical tracking wheel 1, set to nullptr as we don't have one
                            nullptr, // vertical tracking wheel 2, set to nullptr as we don't have one
                            nullptr, // horizontal tracking wheel 1
                            nullptr, // horizontal tracking wheel 2, set to nullptr as we don't have a second one
                            &imu // inertial sensor
);

// create the chassis
lemlib::Chassis chassis(drivetrain, linearController, angularController, sensors);


void Right_Side(){
    chassis.setPose(40,-54,0);
    LedStrip.set_all(0xC36400);
    LedStrip2.set_all(0xC36400);
    LedStrip.update();
    LedStrip2.update();
    Lift = 127;
    Lift2= 127;
    pros::delay(1000);
    Intake=0;
    pros::delay(300);
    Lift= -127;
    Lift2= -127;
    chassis.turnTo(60,-40, 500,true, 100);
    chassis.waitUntilDone();
    chassis.moveToOld(60,-40, 1000, true, true,110);
    chassis.waitUntilDone();
    chassis.turnTo(60,-30, 500, true, 100);
    chassis.waitUntilDone();
    Intake=0;
    chassis.moveToOld(60,-30, 1000, true, true, 127);
    chassis.waitUntilDone();
    chassis.moveToOld(60,-44,1000, false, true, 100);
    chassis.waitUntilDone();
    chassis.linearSettings.kP= 100;
    chassis.moveToOld(60,-30,1000, true, true, 127);
    chassis.waitUntilDone();
    chassis.linearSettings.kP= 10;
    chassis.turnTo(41, -44, 500, true, 100, true);
}
void Right_Side_4_Ball(){
    ///////////////////////////
    //
    //
    // 4 BALL AUTONOMOUS FOR RIGHT SIDE
    //
    //
    ///////////////////////////


    chassis.setPose(40,-54,0); // Sets Position To Our Starting Point
    WingPistons.set_value(true); // Pushes the matchload Triball off to the side
    pros::delay(15); // Waits 15 Miliseconds
    WingPistons.set_value(false); // Retracts wings
    chassis.turnTo(10,-6,1000, true,100); //Turns till the first Triball
    chassis.waitUntilDone(); // Waits till the previous Command is Finished
    Intake = 127; // Sets Intake to max Speed to intake Triball
    chassis.moveToOld(10,-6,3000, true, 100); // Moves to the first triball
    chassis.waitUntilDone(); // Waits till the previous Command is Finished
    chassis.turnTo(44,-6,1000, true,100); // Turns Towards the Goal
    chassis.waitUntilDone(); // Waits till the previous Command is Finished
    WingPistons.set_value(true); // Pushes out wings to push in the triballs better.
    Intake = -127; //Sets Intake to Max Speed In reverse to outtake Triball
    chassis.moveToOld(44,-6,1500,true,100); // Pushes in the Two triballs in the center
    chassis.waitUntilDone(); // Waits till the Previous Command is Finished
    WingPistons.set_value(false); // Retracts wings
    chassis.moveToOld(33,-6,3000,false); // Go backwards from the goal 
    chassis.waitUntilDone(); // Waits till the Previous Command is Finished
    chassis.turnTo(12,-20,1000, true,100); // Turns to the 3rd triball
    chassis.waitUntilDone(); // Waits Till the Previous Command is Finished
    Intake = 127; // Sets Intake to Max Speed to intake the Triball
    chassis.moveToOld(12,-20,3000,true,100); // Moves to the 3rd Triball
    chassis.waitUntilDone(); // Waits till the Previous Command is Finished
    chassis.turnTo(40,-13,1000, true,100); // Turns to wards the goal
    chassis.waitUntilDone(); // Waits till the previous command is finished
    Intake = -127; // Sets Intake to Max Speed in rever to outtake the Triball
    chassis.moveToOld(40,-13,3000, true, 100); // Moves Towards the goal to push in the 3rd triball
    chassis.waitUntilDone(); // waits till the previous command is Finished
    chassis.moveToOld(30,-13,3000, false, 100); // Moves backwards from the goal
    chassis.waitUntilDone(); // Waits till the previous commend is finished
    chassis.turnTo(40,-54,1000, true,100); // Turns back to where we started roughly
    chassis.waitUntilDone(); // Waits till the previous command is finished
    chassis.moveToOld(40,-54,3000, true, 100); // Moves back to where we started roughtly
    chassis.waitUntilDone(); // Waits till the previous command is finished
    chassis.turnTo(59,-43,1000, true,100); // Turns to  the edge of the match load bar
    chassis.waitUntilDone(); // Waits till the previous command is finished
    chassis.moveToOld(59,-43,3000, true,100); // Moves to the edge of the match load bar.
    chassis.waitUntilDone(); // Waits till the previous command is finished
    chassis.turnTo(59,-31,1000, true, 100); // Turns towards the goal
    chassis.waitUntilDone(); // waits till the previous command is finished
    chassis.moveToOld(59,-43,3000,true, 100); // Runs into the goal to push the preload triball into the goal
    chassis.waitUntilDone(); // waits till the the previous command is finished
    chassis.moveToOld(59,-31,3000, false, 100); // Backs up from the Goal
    chassis.waitUntilDone(); // waits till the previous command is finished
    chassis.moveToOld(59,-43,3000, true, 100); // Runs back into the goal to make sure the triball is in
    chassis.waitUntilDone(); // Waits till the previous command is finished
    chassis.moveToOld(59,-31,3000, false, 100); // Moves backwards off the goal for the end of autonomous


}
void Tune_Linear_Pid(){
    chassis.setPose(0,0,0);
    chassis.moveToOld(0, 10, 1000, 100);
}
void Tune_Angular_Pid(){
    chassis.setPose(0,0,0);
    chassis.turnTo(30,0,3000, true, 100);

}
void AutonSkills2(){
    chassis.setPose(40,-54,0);
    chassis.turnTo(60,-40, 500,true, 100);
    chassis.waitUntilDone();
    chassis.moveToOld(60,-40, 1000, true, true,110);
    chassis.waitUntilDone();
    chassis.turnTo(60,-30, 500, true, 100);
    chassis.waitUntilDone();
    chassis.moveToOld(60,-30, 1000, true, true, 127);
    chassis.waitUntilDone();
    chassis.moveToOld(60,-44,1000, false, true, 100);
    chassis.waitUntilDone();
    chassis.turnTo(-20,-21.5, 1000, true, 100);
    chassis.waitUntilDone();
    Lift= 127;
    Lift2= 127;
    pros::delay(500);
    Lift.brake();
    Lift2.brake();
    Slapper=127;
    pros::delay(29000);
    Slapper=0;
    Lift=-127;
    Lift2=-127;
    pros::delay(750);
    Lift=0;
    Lift2=0;
    

}
void AutonSkills(){
    chassis.setPose(40,-54,0);
    LedStrip.set_all(0xC36400);
    LedStrip2.set_all(0xC36400);
    LedStrip.update();
    LedStrip2.update();
    Lift = 127;
    Lift2= 127;
    pros::delay(1000);
    Intake=0;
    pros::delay(300);
    Lift= -127;
    Lift2= -127;
    chassis.turnTo(60,-40, 500,true, 100);
    chassis.waitUntilDone();
    chassis.moveToOld(60,-40, 1000, true, true,110);
    chassis.waitUntilDone();
    chassis.turnTo(60,-30, 500, true, 100);
    chassis.waitUntilDone();
    Intake=0;
    chassis.moveToOld(60,-30, 1000, true, true, 127);
    chassis.waitUntilDone();
    chassis.moveToOld(60,-44,1000, false, true, 100);
    chassis.waitUntilDone();
    chassis.linearSettings.kP= 100;
    chassis.moveToOld(60,-30,1000, true, true, 127);
    chassis.waitUntilDone();
    chassis.linearSettings.kP= 10;
    chassis.moveToOld(60,-44,1000, false, true, 100);
    chassis.waitUntilDone();
    chassis.turnTo(-20,-21.5, 1000, true, 100);
    chassis.waitUntilDone();
    Intake=0;
    Lift= 127;
    Lift2= 127;
    Slapper=127;
    pros::delay(30000);
    Slapper=0;
    Lift=-127;
    Lift2=-127;
    chassis.turnTo(36,-60,500,true,100,true);
    chassis.waitUntilDone();
    chassis.moveToOld(36,-60,10000,true, true, 127);
    chassis.waitUntilDone();
    chassis.turnTo(-34, -58, 1000, true, 100, true);
    chassis.waitUntilDone();
    chassis.moveToOld(-34,-58, 7500, true, true, 127);
    chassis.waitUntilDone();
    chassis.turnTo(-58,-43,750, true, 100, true);
    chassis.waitUntilDone();
    chassis.moveToOld(-58,-43,3000, true, true, 127);
    chassis.waitUntilDone();
    chassis.turnTo(-58,-31, 500, true, 100, true);
    chassis.waitUntilDone();
    chassis.linearSettings.kP=100;
    chassis.moveToOld(-58,-31, 3000, true, true,127);
    chassis.waitUntilDone();
    chassis.linearSettings.kP=10;
    chassis.moveToOld(-58,-43, 3000, false, true, 127);
    chassis.waitUntilDone();
    chassis.turnTo(-13,-30, 500, true, 100, true);
    chassis.waitUntilDone();
    chassis.moveToOld(-13,-30,2500,true, true, 127);
    chassis.waitUntilDone();
    chassis.turnTo(-13, 0, 500,true, 100, true);
    chassis.moveToOld(-13,0, 2000, true, true, 127);
    chassis.waitUntilDone();
    chassis.turnTo(-39, 0, 500, true, 100, true);
    chassis.waitUntilDone();
    WingPistons.set_value(true);
    chassis.linearSettings.kP= 100;
    chassis.moveToOld(-39,0, 1500,true, true, 127);
    chassis.waitUntilDone();
    chassis.linearSettings.kP=10;
    chassis.moveToOld(-13, 0, 1500, false, true,127);
    chassis.waitUntilDone();
    chassis.linearSettings.kP=100;
    chassis.moveToOld(-39,0,1500, true, true,127);
    chassis.waitUntilDone();
    chassis.linearSettings.kP=10;
    chassis.moveToOld(-13,0 , 1500, false,true, 127);
    chassis.waitUntilDone();
    chassis.linearSettings.kP=100;
    chassis.moveToOld(-39,0,1500, true, true,127);
    chassis.waitUntilDone();
    chassis.linearSettings.kP=10;
    chassis.moveToOld(-13,0, 1500, false, true, 127);
    chassis.waitUntilDone();



}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
    pros::lcd::initialize(); // initialize brain screen
    chassis.calibrate(); // calibrate sensors

    // the default rate is 50. however, if you need to change the rate, you
    // can do the following.
    // lemlib::bufferedStdout().setRate(...);
    // If you use bluetooth or a wired connection, you will want to have a rate of 10ms

    // for more information on how the formatting for the loggers
    // works, refer to the fmtlib docs

    // thread to for brain screen and position logging
    pros::Task screenTask([&]() {
        lemlib::Pose pose(0, 0, 0);
        while (true) {
            // print robot location to the brain screen
            pros::lcd::print(0, "X: %f", chassis.getPose().x); // x
            pros::lcd::print(1, "Y: %f", chassis.getPose().y); // y
            pros::lcd::print(2, "Theta: %f", chassis.getPose().theta); // heading
            // log position telemetry
            lemlib::telemetrySink()->info("Chassis pose: {}", chassis.getPose());
            // delay to save resources
            pros::delay(50);
        }
    });
}

/**
 * Runs while the robot is disabled
 */
void disabled() {}

/**
 * runs after initialize if the robot is connected to field control
 */
void competition_initialize() {}

// get a path used for pure pursuit
// this needs to be put outside a function
 // '.' replaced with "_" to make c++ happy

/**
 * Runs during auto
 *
 * This is an example autonomous routine which demonstrates a lot of the features LemLib has to offer
 */
void autonomous() {
    Right_Side();
}

/**
 * Runs in driver control
 */
void opcontrol() {
    // controller
    // loop to continuously update motors
    while (true) {
        LedStrip.set_all(0xC36400);
        LedStrip2.set_all(0xC36400);
        LedStrip.update();
        LedStrip2.update();
        // get joystick positions
        int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightX = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
        // move the chassis with arcade drive
        chassis.arcade(leftY, rightX);
        // Wing Code
        if(controller.get_digital_new_press(DIGITAL_X)){
            WingBool = !WingBool;
            WingPistons.set_value(WingBool);
        }
        //Intake Code
        if(controller.get_digital(DIGITAL_L2)){
            Intake = 127;
        } else if(controller.get_digital(DIGITAL_L1)){
            Intake = -127;
        }else{
            Intake = 0;
        }
        // Slapper Code
        if(controller.get_digital(DIGITAL_UP)){
            Slapper = 127;
		}else{
			Slapper= 0;
		}
        // Lift Code
        if(controller.get_digital(DIGITAL_R1)){
            Lift =127;
            Lift2= 127;
        } else if(controller.get_digital(DIGITAL_R2)){
            Lift= -127;
            Lift2= -127;
        }else{
            Lift= 0;
            Lift2=0;
        }

        // delay to save resources
        pros::delay(10);
    }
}
