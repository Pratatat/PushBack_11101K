#include "main.h"
#include "pros/adi.hpp"

/**
 * Resets the constants for auton movement.
 * Modify these to change the default behavior of functions like
 * drive_distance(). For explanations of the difference between
 * drive, heading, turning, and swinging, as well as the PID and
 * exit conditions, check the docs.
 */

void default_constants(){
  
  // Each constant set is in the form of (maxVoltage, kP, kI, kD, startI).
  chassis.set_drive_constants(8, 0.4325, 0, 0.91, 0);
  chassis.set_heading_constants(6, .23, 0, 1, 0);
  chassis.set_turn_constants(8, .27, 0, 1.89, 0);
  chassis.set_swing_constants(12, .3, .001, 2, 15);

  // Each exit condition set is in the form of (settle_error, settle_time, timeout).
  chassis.set_drive_exit_conditions(1.5, 75, 1500);
  chassis.set_turn_exit_conditions(2.5, 75, 1500);
  chassis.set_swing_exit_conditions(1, 300, 3000);
}

/**
 * Sets constants to be more effective for odom movements.
 * For functions like drive_to_point(), it's often better to have
 * a slower max_voltage and greater settle_error than you would otherwise.
 */

void odom_constants(){
  default_constants();
  chassis.heading_max_voltage = 10;
  chassis.drive_max_voltage = 7;
  chassis.drive_settle_error = 3;
}


/**
 * The expected behavior is to return to the start position.
 */

void drive_test(){
  chassis.drive_distance(6);
  chassis.drive_distance(12);
  chassis.drive_distance(18);
  chassis.drive_distance(-36);
}

/**
 * The expected behavior is to return to the start angle, after making a complete turn.
 */

void turn_test(){
  chassis.turn_to_angle(5);
  chassis.turn_to_angle(30);
  chassis.turn_to_angle(90);
  chassis.turn_to_angle(225);
  chassis.turn_to_angle(0);
}

/**
 * Should swing in a fun S shape.
 */

void swing_test(){
  chassis.left_swing_to_angle(90);
  chassis.right_swing_to_angle(0);
}

/**
 * A little of this, a little of that; it should end roughly where it started.
 */

void full_test(){
  chassis.drive_distance(24);
  chassis.turn_to_angle(-45);
  chassis.drive_distance(-36);
  chassis.right_swing_to_angle(-90);
  chassis.drive_distance(24);
  chassis.turn_to_angle(0);
}

/**
 * Doesn't drive the robot, but just prints coordinates to the Brain screen 
 * so you can check if they are accurate to life. Push the robot around and
 * see if the coordinates increase like you'd expect.
 */

/**
 * Should end in the same place it began, but the second movement
 * will be curved while the first is straight.
 */
void tank_odom_test(){
  default_constants();
  chassis.set_coordinates(0, 0, 0);
  //chassis.turn_to_point(24, 24,0,4);
  //chassis.drive_to_point(24, 24,4,0,1,200,2000);
  //chassis.turn_to_angle(90,9);
  
  //chassis.drive_to_point(0, 24);
  //chassis.drive_distance(48,1.6,1,200,10000,0.48,0,0.65,0);
  //printf("Drive: X: %f, Y: %f, Heading: %f\n", chassis.get_X_position(), chassis.get_Y_position(), chassis.get_absolute_heading());
  

  
  chassis.turn_to_point(0, 48,0,8);
  printf("turn: X: %f, Y: %f, Heading: %f\n", chassis.get_X_position(), chassis.get_Y_position(), chassis.get_absolute_heading());

chassis.drive_to_point(0, 48,8,2);
  printf("Drive: X: %f, Y: %f, Heading: %f\n", chassis.get_X_position(), chassis.get_Y_position(), chassis.get_absolute_heading());
  
  chassis.turn_to_point(48, 48,0,8);
  printf("Turn: X: %f, Y: %f, Heading: %f\n", chassis.get_X_position(), chassis.get_Y_position(), chassis.get_absolute_heading());
  pros::delay(100);
  
  chassis.drive_to_point(48, 48,8,2);
  printf("Drive: X: %f, Y: %f, Heading: %f\n", chassis.get_X_position(), chassis.get_Y_position(), chassis.get_absolute_heading());
  pros::delay(100);
  chassis.turn_to_point(48, 0,0,8);
  printf("Turn: X: %f, Y: %f, Heading: %f\n", chassis.get_X_position(), chassis.get_Y_position(), chassis.get_absolute_heading());
  pros::delay(100);
  chassis.drive_to_point(48, 0,8,2);
  printf("Drive: X: %f, Y: %f, Heading: %f\n", chassis.get_X_position(), chassis.get_Y_position(), chassis.get_absolute_heading());
  pros::delay(100);
  chassis.turn_to_point(0, 0,0,8);
  printf("Turn: X: %f, Y: %f, Heading: %f\n", chassis.get_X_position(), chassis.get_Y_position(), chassis.get_absolute_heading());
  pros::delay(100);
  chassis.drive_to_point(0, 0,8,2);
  printf("Drive: X: %f, Y: %f, Heading: %f\n", chassis.get_X_position(), chassis.get_Y_position(), chassis.get_absolute_heading());
  pros::delay(100);
  chassis.turn_to_point(0, 48,0,8);
  printf("Turn: X: %f, Y: %f, Heading: %f\n", chassis.get_X_position(), chassis.get_Y_position(), chassis.get_absolute_heading());
  pros::delay(100);

  
}

void odom_offset_test(){
  odom_constants();
  chassis.set_coordinates(0, 0, 0);
  printf("forward: %.2f",chassis.get_ForwardTracker_position());
  printf("Sideways: %.2f",chassis.get_SidewaysTracker_position());
  chassis.turn_to_angle(90);
  printf("X: %f, Y: %f, Heading: %f\n", chassis.get_X_position(), chassis.get_Y_position(), chassis.get_absolute_heading());
  printf("forward: %.2f",chassis.get_ForwardTracker_position());
  printf("Sideways: %.2f",chassis.get_SidewaysTracker_position());
}

/**
 * Drives in a square while making a full turn in the process. Should
 * end where it started.
 */
void RightAWP(){
  default_constants();
  chassis.set_coordinates(13.73, 0.6, 180);
  //scoring_mech.top_goal_intake(50);
  chassis.drive_to_point(13.73,33);
  scoring_mech.top_goal_intake(600);
  //pneumatics.intakepiston_v(1);
  //pros::delay(200);
  chassis.turn_to_point(-1,22.5,0,8);
  scoring_mech.intake_move(600);
  pros::Task awp_task2(Pneumatics::awp_task2);
  chassis.drive_to_point(-1,22.5,4.5,2);
  
  //pros::delay(350);
  chassis.turn_to_point(-24,4.5,0,8,2.5,10,800);
  chassis.drive_to_point(-23.5,4.5,8,1.5);
  scoring_mech.intake_move(0);
  pneumatics.matchloader_v(0);
  chassis.turn_to_point(-23.5,-48,0,8);

  scoring_mech.top_goal_intake(75);
  pros::Task intae(Scoring_Mech::intake_autontask);
  chassis.drive_to_point(-23.5,23);
  scoring_mech.top_goal_intake(600);
  pneumatics.matchloader_v(1);
  pros::delay(340);
  //chassis.turn_to_point(-23.25,-48,0,8);
  chassis.drive_to_point(-23.25,-13,5.5,2);
  scoring_mech.intake_move(600);
  pros::delay(10);
  chassis.drive_to_point(-23.25,0);
  pneumatics.matchloader_v(0);
  scoring_mech.intake_move(600);
  chassis.turn_to_point(47,24,0,10);
  scoring_mech.intake_move(450);
  pros::Task antijam(Scoring_Mech::anti_jam_auton);
  chassis.drive_to_point(47,24);
  scoring_mech.intake_move(600);
  chassis.turn_to_point(70.3,5,0,8,2.5,10,800);
  chassis.drive_to_point(70.3,5,8,1);
  chassis.turn_to_point(70.3,-48,0,10);
  scoring_mech.top_goal_intake(50);
  pros::Task intae2(Scoring_Mech::intake_autontask);
  chassis.drive_to_point(70.3,22.5);
  scoring_mech.top_goal_intake(600);
  pneumatics.matchloader_v(1);
}




void LeftSevenBall(){
  default_constants();
  chassis.set_coordinates(13.73, 1.65, 0);
  chassis.turn_to_point(-2,28,0,8);
  scoring_mech.intake_move(600);
  pros::Task awp_task2(Pneumatics::awp_task2);
  chassis.drive_to_point(-2,28,5.5,2);
  
  //pros::delay(350);
  chassis.turn_to_point(-23.4,5,0,8);
  chassis.drive_to_point(-23.4,5,8,1);
  scoring_mech.intake_move(0);
  pneumatics.matchloader_v(0);
  chassis.turn_to_point(-23.4,-48,0,8);
  pros::Task intae(Scoring_Mech::intake_autontask);
  chassis.drive_to_point(-23.4,24,8,1.5);
  scoring_mech.top_goal_intake(600);
  pneumatics.matchloader_v(1);
  pros::delay(150);
  chassis.turn_to_point(-23.25,-16,1,8,2.5,10,800);
  chassis.drive_to_point(-23.25,-16,6.5,2);
  scoring_mech.intake_move(600);
  pros::delay(80);
  pros::Task intae2(Scoring_Mech::intake_autontask2);
  chassis.drive_to_point(-23.4,23,8,1);
  pneumatics.matchloader_v(0);
  scoring_mech.top_goal_intake(600);
  pros::delay(1850);
  
  chassis.drive_distance(12,12,1.5,30,700,0.4325, 0, 0.91, 0);
  scoring_mech.top_goal_intake(0);
  chassis.drive_distance(-20,12,1.5,30,700,0.4325, 0, 0.91, 0);
  /*
  default_constants();
  chassis.set_coordinates(13.73, 0.6, 180);
  //scoring_mech.top_goal_intake(50);
  chassis.drive_to_point(13.73,33);
  scoring_mech.top_goal_intake(600);
  //pneumatics.intakepiston_v(1);
  //pros::delay(200);
  chassis.turn_to_point(-1,22.5,0,8);
  scoring_mech.intake_move(600);
  pros::Task awp_task2(Pneumatics::awp_task2);
  chassis.drive_to_point(-1,22.5,4.5,2);
  
  //pros::delay(350);
  chassis.turn_to_point(-23.5,4.5,0,8);
  chassis.drive_to_point(-23.5,4.5,8,1);
  scoring_mech.intake_move(0);
  pneumatics.matchloader_v(0);
  chassis.turn_to_point(-23.5,-48,0,8);
  pros::Task intae(Scoring_Mech::intake_autontask);
  chassis.drive_to_point(-23.5,23.25);
  scoring_mech.top_goal_intake(600);
  pneumatics.matchloader_v(1);
  pros::delay(350);
  chassis.turn_to_point(-23.25,-15,1,8);
  chassis.drive_to_point(-23.25,-15,4.5,2);
  scoring_mech.intake_move(600);
  pros::delay(80);
  chassis.drive_to_point(-23.25,23);
  pneumatics.matchloader_v(0);
  scoring_mech.top_goal_intake(600);
  pros::delay(2100);
  
  chassis.drive_distance(10,12);
  scoring_mech.top_goal_intake(0);
  chassis.drive_distance(-15,12);
*/
}

void RightSevenBall(){
  default_constants();
  chassis.set_coordinates(-13.73, 0.6, 0);
  chassis.turn_to_point(2,28);
  scoring_mech.intake_move(600);
  pros::Task awp_task2(Pneumatics::awp_task);
  chassis.drive_to_point(2,28,4.5,2);
  pros::delay(350);
  chassis.turn_to_point(24,5,2.5,8,2.5,75,1500);
  chassis.drive_to_point(24,5,5,0.5);
  scoring_mech.intake_move(0);
  pneumatics.matchloader_v(0);
  
  
  chassis.turn_to_point(24,-48,0,8);


  
  pros::Task intae(Scoring_Mech::intake_autontask);
  chassis.drive_to_point(24,22.5);
  scoring_mech.top_goal_intake(600);
  pneumatics.matchloader_v(1);
  pros::delay(650);
  chassis.drive_to_point(23.5,-13,4.5,2);
  scoring_mech.intake_move(450);
  pros::delay(100);
  
  chassis.drive_to_point(23.5,22.5,6,2);
  scoring_mech.top_goal_intake(600);
  pneumatics.matchloader_v(0);
  pros::delay(2200);
  chassis.drive_distance(10,12);
  scoring_mech.top_goal_intake(0);
  chassis.drive_distance(-15,12);


}
void RightRush(){
  default_constants();
  chassis.set_coordinates(-13.73, 0.6, 0);
  chassis.turn_to_point(2,28);
  scoring_mech.intake_move(600);
  pros::Task awp_task2(Pneumatics::awp_task);
  chassis.drive_to_point(2,28,4.5,2);
  pros::delay(350);
  pros::Task antijag(Scoring_Mech::anti_jam_auton);
  chassis.turn_to_point(18,38);
  pneumatics.matchloader_v(0);
  chassis.drive_to_point(18,38,4.5,2);
  pros::delay(100);
  chassis.drive_to_point(2,28);
  
  
  chassis.turn_to_point(24,5,2.5,8,2.5,75,1500);
  chassis.drive_to_point(24,5,5,0.5);
  scoring_mech.intake_move(0);
  pneumatics.matchloader_v(0);
  
  
  chassis.turn_to_point(24,-48,0,8);


  
  pros::Task intae(Scoring_Mech::intake_autontask);
  chassis.drive_to_point(24,22.5);
  scoring_mech.top_goal_intake(600);
  pneumatics.matchloader_v(1);
  pros::delay(650);
  chassis.drive_to_point(23.5,-13,4.5,2);
  scoring_mech.intake_move(450);
  pros::delay(100);
  
  chassis.drive_to_point(23.5,22.5,6,2);
  scoring_mech.top_goal_intake(600);
  pneumatics.matchloader_v(0);
  pros::delay(2200);
  chassis.drive_distance(10,12);
  scoring_mech.top_goal_intake(0);
  chassis.drive_distance(-15,12);
  
}

void LeftRush(){
  default_constants();
  chassis.set_coordinates(13.73, 1.6, 0);
  chassis.turn_to_point(-2,28);
  scoring_mech.intake_move(600);
  pros::Task awp_task2(Pneumatics::awp_task);
  chassis.drive_to_point(-2,28,4.5,2);
  pros::delay(350);
  pros::Task antijag(Scoring_Mech::anti_jam_auton);
  chassis.turn_to_point(-18,38);
  pneumatics.matchloader_v(0);
  chassis.drive_to_point(-18,38,4.5,2);
  pros::delay(100);
  chassis.drive_to_point(-2,28);
  
  
  chassis.turn_to_point(-24,5,2.5,8,2.5,75,1500);
  chassis.drive_to_point(-24,5,5,0.5);
  scoring_mech.intake_move(0);
  pneumatics.matchloader_v(0);
  
  
  chassis.turn_to_point(-24,-48,0,8);


  
  pros::Task intae(Scoring_Mech::intake_autontask);
  chassis.drive_to_point(-24,22.5);
  scoring_mech.top_goal_intake(600);
  pneumatics.matchloader_v(1);
  pros::delay(650);
  chassis.drive_to_point(-23.5,-13,4.5,2);
  scoring_mech.intake_move(450);
  pros::delay(100);
  
  chassis.drive_to_point(-23.5,22.5,6,2);
  scoring_mech.top_goal_intake(600);
  pneumatics.matchloader_v(0);
  pros::delay(2200);
  chassis.drive_distance(10,12);
  scoring_mech.top_goal_intake(0);
  chassis.drive_distance(-15,12);
}

void auton_setup() {
  chassis.set_brake_mode('C');
  chassis.set_coordinates(0, 0, 0);
  std::string x_str, y_str, heading_str;
  while (1){ 
    x_str = std::to_string(chassis.get_X_position());
    y_str = std::to_string(chassis.get_Y_position());
    heading_str = std::to_string(chassis.get_absolute_heading());
    pros::screen::draw_rect(0,0,480,240);
    pros::screen::set_pen(pros::Color::white);
    pros::screen::print(TEXT_LARGE, 50, 50, x_str.c_str());
    pros::screen::print(TEXT_LARGE, 50, 125, y_str.c_str());
    pros::screen::print(TEXT_LARGE, 50, 175, heading_str.c_str()); 
  }
  /*
  odom_constants();
  chassis.set_brake_mode('C');
  chassis.set_coordinates(0, 0, 0);
  pros::delay(500);
  chassis.calculate();
*/
}