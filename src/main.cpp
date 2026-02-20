#include "main.h"
#include <iostream> 

//Motor Definitions
<<<<<<< Updated upstream
pros::adi::DigitalOut matchloader('A');
pros::adi::DigitalOut intake_piston('B');
pros::adi::DigitalOut wing('C');
pros::Motor left_front_mtr(-1, pros::v5::MotorGears::blue, pros::v5::MotorUnits::degrees);
pros::Motor left_middle_mtr(-12, pros::v5::MotorGears::blue, pros::v5::MotorUnits::degrees);
pros::Motor left_back_mtr(-11, pros::v5::MotorGears::blue, pros::v5::MotorUnits::degrees);
pros::Motor right_front_mtr(10, pros::v5::MotorGears::blue, pros::v5::MotorUnits::degrees);
pros::Motor right_middle_mtr(19, pros::v5::MotorGears::blue, pros::v5::MotorUnits::degrees);
pros::Motor right_back_mtr(20, pros::v5::MotorGears::blue, pros::v5::MotorUnits::degrees);
=======
pros::adi::DigitalOut hood('A');
pros::adi::DigitalOut matchloader('B');
pros::adi::DigitalOut intake_piston('C');
pros::adi::DigitalOut wing('D');

pros::Motor left_front_mtr(-11, pros::v5::MotorGears::blue, pros::v5::MotorUnits::degrees);
pros::Motor left_middle_mtr(12, pros::v5::MotorGears::blue, pros::v5::MotorUnits::degrees);
pros::Motor left_back_mtr(-13, pros::v5::MotorGears::blue, pros::v5::MotorUnits::degrees);
pros::Motor right_front_mtr(20, pros::v5::MotorGears::blue, pros::v5::MotorUnits::degrees);
pros::Motor right_middle_mtr(-19, pros::v5::MotorGears::blue, pros::v5::MotorUnits::degrees);
pros::Motor right_back_mtr(18, pros::v5::MotorGears::blue, pros::v5::MotorUnits::degrees);
>>>>>>> Stashed changes


Drive chassis( 
  //ZERO_TRACKER_NO_ODOM, ZERO_TRACKER_ODOM, TANK_ONE_ENCODER, TANK_ONE_ROTATION, TANK_TWO_ENCODER, TANK_TWO_ROTATION, HOLONOMIC_TWO_ENCODER, and HOLONOMIC_TWO_ROTATION
  TANK_TWO_ROTATION,
  //Left Motors:
  {left_front_mtr.get_port(), left_middle_mtr.get_port(), left_back_mtr.get_port()},
  //Right Motors:
  {right_front_mtr.get_port(), right_middle_mtr.get_port(), right_back_mtr.get_port()},
  //IMU Port:
  15,
  //Wheel diameter (4" omnis are actually closer to 4.125"):
  3.25,
  //External Gear Ratio
  0.75,
  //Gyro scale, this is what your gyro reads when you spin the robot 360 degrees.
  358.2,
  //Remaining inputs are for position tracking
  //If you are using ZERO_TRACKER_ODOM, you ONLY need to adjust the FORWARD TRACKER CENTER DISTANCE.
  //If you are using position tracking, this is the Forward Tracker port (the tracker which runs parallel to the direction of the chassis).
  //If this is an encoder, enter the port as an integer. Triport A will be a "1", Triport B will be a "2", etc.
  8,
  //Input the Forward Tracker diameter (reverse it to make the direction switch):
  -1.985,
  //Input Forward Tracker center distance (In.) (a positive distance corresponds to a tracker on the right side of the robot, negative is left.)
  //For a zero tracker tank drive with odom, put the positive distance from the center of the robot to the right side of the drive.
  0.2,
  //Input the Sideways Tracker Port, following the same steps as the Forward Tracker Port:
  17, 
  //Sideways tracker diameter (reverse to make the direction switch):
  2.00,
  //Sideways tracker center distance (positive distance is behind the center of the robot, negative is in front):
  1.75
);

Scoring_Mech scoring_mech(
<<<<<<< Updated upstream
  9,-18,17
  //{-9, 7},
  //bottom, top, color sensor
=======
  -16,14,26
  // Bottom Intake, Top Intake, Color Sensor
>>>>>>> Stashed changes
);


Pneumatics pneumatics(
<<<<<<< Updated upstream
	{matchloader, intake_piston, wing}
=======
	{matchloader, intake_piston, wing, hood}
>>>>>>> Stashed changes
);


void initialize() {
	chassis.initialize();
  scoring_mech.initialize(); 
	pneumatics.matchloader_initialize();
  pneumatics.intakepiston_initialize();
  pneumatics.wing_initialize();
<<<<<<< Updated upstream
=======
  pneumatics.hood_initialize();
>>>>>>> Stashed changes

  //pros::Task intake_task_3(Scoring_Mech::intake_detector_task);
  //pros::Task intake_task_1(Scoring_Mech::red_color_sort_task);
  //pros::Task intake_task_2(Scoring_Mech::blue_color_sort_task);
}


void competition_initialize() {}


void autonomous() { 
  chassis.set_brake_mode('H');
<<<<<<< Updated upstream
  LeftSevenBall();
=======
  AWP();
>>>>>>> Stashed changes
}

void opcontrol(void) {
  chassis.set_brake_mode('C');
  //chassis.calculate();
  scoring_mech.driverControl = true;
  pros::Task intake_task(Scoring_Mech::intake_task);
  pros::Task pneumatics_matchloader_task(Pneumatics::matchloader_task);
  pros::Task pneumatics_intakepiston_task(Pneumatics::intakepiston_task);
  pros::Task pneumatics_wing_task(Pneumatics::wing_task);
<<<<<<< Updated upstream
=======
  pros::Task pneumatics_hood_task(Pneumatics::hood_task);
  
>>>>>>> Stashed changes

  std::string left_front,left_middle, left_back, right_front, right_middle, right_back;
  while (true) {
    chassis.arcade_control_double_reversed();
    pros::delay(util::DELAY_TIME); 
    /*
    left_front = std::to_string(left_front_mtr.get_temperature());
    left_middle = std::to_string(left_middle_mtr.get_temperature());
    left_back = std::to_string(left_back_mtr.get_temperature());
    right_front = std::to_string(right_front_mtr.get_temperature());
    right_middle = std::to_string(right_middle_mtr.get_temperature());
    right_back = std::to_string(right_back_mtr.get_temperature());

    pros::screen::draw_rect(0,0,480,240);
    pros::screen::set_pen(pros::Color::white);
    pros::screen::print(TEXT_LARGE, 50, 50, left_front.c_str());
    pros::screen::print(TEXT_LARGE, 50, 100, left_middle.c_str());
    pros::screen::print(TEXT_LARGE, 50, 150, left_back.c_str());
    pros::screen::print(TEXT_LARGE, 250, 50, right_front.c_str());
    pros::screen::print(TEXT_LARGE, 250, 100, right_middle.c_str());
    pros::screen::print(TEXT_LARGE, 250, 150, right_back.c_str());
    */
  }
}

void disabled() {}

<<<<<<< Updated upstream






/*

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
  6,
  //Input the Forward Tracker diameter (reverse it to make the direction switch):
  2,
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
  RightAWP();
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

*/
=======
void competition_initialize() {}
>>>>>>> Stashed changes
