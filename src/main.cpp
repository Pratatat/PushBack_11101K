#include "main.h"
#include <iostream> 

//Motor Definitions
pros::adi::DigitalOut aligner('A');
pros::adi::DigitalOut doinker_left('C');
pros::adi::DigitalOut doinker_right('B');
pros::Motor left_front_mtr(-1, pros::v5::MotorGears::blue, pros::v5::MotorUnits::degrees);
pros::Motor left_middle_mtr(-2, pros::v5::MotorGears::blue, pros::v5::MotorUnits::degrees);
pros::Motor left_back_mtr(-11, pros::v5::MotorGears::blue, pros::v5::MotorUnits::degrees);
pros::Motor right_front_mtr(5, pros::v5::MotorGears::blue, pros::v5::MotorUnits::degrees);
pros::Motor right_middle_mtr(4, pros::v5::MotorGears::blue, pros::v5::MotorUnits::degrees);
pros::Motor right_back_mtr(21, pros::v5::MotorGears::blue, pros::v5::MotorUnits::degrees);


Drive chassis( 
  //ZERO_TRACKER_NO_ODOM, ZERO_TRACKER_ODOM, TANK_ONE_ENCODER, TANK_ONE_ROTATION, TANK_TWO_ENCODER, TANK_TWO_ROTATION, HOLONOMIC_TWO_ENCODER, and HOLONOMIC_TWO_ROTATION
  TANK_TWO_ROTATION,
  //Left Motors:
  {left_front_mtr.get_port(), left_middle_mtr.get_port(), left_back_mtr.get_port()},
  //Right Motors:
  {right_front_mtr.get_port(), right_middle_mtr.get_port(), right_back_mtr.get_port()},
  //IMU Port:
  9,
  //Wheel diameter (4" omnis are actually closer to 4.125"):
  3.25,
  //External Gear Ratio
  0.75,
  //Gyro scale, this is what your gyro reads when you spin the robot 360 degrees.
  360,
  //Remaining inputs are for position tracking
  //If you are using ZERO_TRACKER_ODOM, you ONLY need to adjust the FORWARD TRACKER CENTER DISTANCE.
  //If you are using position tracking, this is the Forward Tracker port (the tracker which runs parallel to the direction of the chassis).
  //If this is an encoder, enter the port as an integer. Triport A will be a "1", Triport B will be a "2", etc.
  0,
  //Input the Forward Tracker diameter (reverse it to make the direction switch):
  -2,
  //Input Forward Tracker center distance (In.) (a positive distance corresponds to a tracker on the right side of the robot, negative is left.)
  //For a zero tracker tank drive with odom, put the positive distance from the center of the robot to the right side of the drive.
  0,
  //Input the Sideways Tracker Port, following the same steps as the Forward Tracker Port:
  10, 
  //Sideways tracker diameter (reverse to make the direction switch):
  2,
  //Sideways tracker center distance (positive distance is behind the center of the robot, negative is in front):
  3.3
);

Scoring_Mech scoring_mech(
  8,13,-12,17
  //{-9, 7},
  //bottom, middle, top, color sensor
);


Pneumatics pneumatics(
	{aligner, doinker_left, doinker_right}
);


void initialize() {
	chassis.initialize();
  scoring_mech.initialize(); 
	pneumatics.aligner_initialize();
  pneumatics.doinker_initialize();

  //pros::Task intake_task_3(Scoring_Mech::intake_detector_task);
  //pros::Task intake_task_1(Scoring_Mech::red_color_sort_task);
  //pros::Task intake_task_2(Scoring_Mech::blue_color_sort_task);
}


void competition_initialize() {}


void autonomous() {
  chassis.set_brake_mode('H');
  tank_odom_test();
}

void opcontrol(void) {
  chassis.set_brake_mode('C');
  //chassis.calculate();
  scoring_mech.driverControl = true;
  //pros::Task neutral_stake_task(Scoring_Mech::neutral_stake_task);
  pros::Task intake_task(Scoring_Mech::intake_task);
  pros::Task pneumatics_aligner_task(Pneumatics::aligner_task);
  //pros::Task pneumatics_doinker_task_1(Pneumatics::doinker_right_task);
  //pros::Task pneumatics_doinker_task_2(Pneumatics::doinker_left_task);
  while (true) {
    chassis.arcade_control_double_reversed();
    pros::delay(util::DELAY_TIME); 
  }
}

void disabled() {}




