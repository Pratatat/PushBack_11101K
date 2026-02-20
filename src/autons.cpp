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
  
<<<<<<< Updated upstream
  //pros::delay(350);
  chassis.turn_to_point(-24,4.5,0,8,2.5,10,800);
  chassis.drive_to_point(-23.5,4.5,8,1.5);
=======

  pros::Task intae(Scoring_Mech::intake_autontask);
  chassis.drive_to_point(23.6,25,5,0.5,3,30,1000);
  chassis.drive_distance(-13, 12,5,30,500,0.75, 0.0000, 3.75, 2);
  pneumatics.matchloader_v(0);
  chassis.set_coordinates(23.6, 19.6, chassis.get_absolute_heading()+1);
  
  
  chassis.turn_to_point(-4.5,25.5,1,8,2.5,50,800);
  
  scoring_mech.intake_move(600);
  pros::Task intae2(Scoring_Mech::intake_autontask4);
  pros::Task awp_task(Pneumatics::awp_task);
  
  chassis.drive_to_point(-4.5,25.5,6,1,3,30,1000);
  pros::delay(140);

/*
  chassis.turn_to_point(11,35,1,8,4,30,700);
  pneumatics.matchloader_v(0);
  scoring_mech.intake_move(600);
  chassis.drive_to_point(11,35,6,1,3,30,1000);
  chassis.turn_to_point(18,38,1,8,4,30,700);
  chassis.drive_to_point(18,38,3.5,0.5,3,30,700);
  pros::delay(300);
  chassis.drive_to_point(4,25.5,5.5,4,3,30,1000);

*/

  chassis.turn_to_point(-13.5,35.5,7,8,2.5,50,700);
  pneumatics.matchloader_v(0);
  scoring_mech.top_goal_intake(0);
  chassis.drive_to_point(-13.5,35.5,8,1,3,30,800);
  
  scoring_mech.top_goal_intake(-100);
  chassis.drive_distance(-6, 8,5,30,310,0.75, 0.0000, 3.75, 2);
  pros::delay(850);
/*
  scoring_mech.top_goal_intake(-75);
  chassis.drive_distance(-4.5, 8,5,30,310,0.75, 0.0000, 3.75, 2);
  pros::delay(90);
  scoring_mech.top_goal_intake(0);
  pros::delay(20);
  scoring_mech.top_goal_intake(-75);
  pros::delay(220);
  scoring_mech.top_goal_intake(0);
  pros::delay(40);
  scoring_mech.top_goal_intake(-85);  
  pros::delay(220);
  scoring_mech.top_goal_intake(0);
   pros::delay(40);
   scoring_mech.top_goal_intake(-85);
  pros::delay(440);
 
  */
  scoring_mech.intake_move(600);
  chassis.drive_to_point(-4.5,25.5,6.25,1,3.5,30,900);

  chassis.turn_to_point(-46.75,24.75,1,8,2.5,50,900);
  pros::Task awp_task2(Pneumatics::awp_task8);
  chassis.drive_to_point(-46.75,24.75,6.25,1,3,50,1500);
  pros::delay(250);
  chassis.turn_to_point(-72,-4,1,8,2.5,50,800);
  scoring_mech.top_goal_intake(0);
  pros::Task awp_task3(Pneumatics::awp_task4); 
  
  chassis.drive_to_point(-35.2,37,6.25,0,3,50,1000);
  scoring_mech.mid_intake_move(600);
  chassis.drive_distance(-10,12,5,25,300,0.75, 0.0000, 3.75, 2);
  pros::delay(500);
   scoring_mech.mid_intake_move(400);
   pros::delay(500);
  pneumatics.matchloader_v(0);
  scoring_mech.mid_intake_move(0);
  pneumatics.intakepiston_v(0);
  chassis.drive_distance(19,12,5,25,300,0.75, 0.0000, 3.75, 2);
  //pneumatics.descore_v(1);
  pros::delay(300);
  chassis.drive_distance(-20,5.5,5,25,300,0.75, 0.0000, 3.75, 2);
  scoring_mech.top_goal_intake(0);

}

void RightFour(){
  default_constants();
  chassis.set_coordinates(-7, -1.5, 90);
  chassis.drive_to_point(20.65,-1.5,6.5,0,3,50,800);
  pneumatics.matchloader_v(1);
  scoring_mech.intake_move(600);
  chassis.turn_to_point(24.7,-24,1,8,2.5,30,700);
  
  chassis.drive_to_point(23.6,-15.35,5,0,3,50,850);
  chassis.drive_to_point(23.6,4,5.5,6,5.5,30,700);
  

  pros::Task intae(Scoring_Mech::intake_autontask);
  chassis.drive_to_point(23.75,25,6,0.5,3,30,800);
  chassis.drive_distance(-12, 7,5,30,600,0.75, 0.0000, 3.75, 2);
  pneumatics.matchloader_v(0);
  chassis.set_coordinates(23.6, 19.6, 181.5);

  chassis.turn_to_point(34.25,10,1,8,5,50,400);
  chassis.drive_to_point(34.25,10,7,1,8,30,600);
  chassis.turn_to_point(50,-96,1,8,5,50,400);
  chassis.drive_to_point(14.75,35.5,10.5,0,1,100,800);
  //pros::delay(500);
  chassis.turn_to_point(-65,-60,1,8,5,30,500);
  
  /**/
}

void RightSevenElim(){
  // 7 Ball
  default_constants();
  chassis.set_coordinates(-9, -3.24, 25.5);
  scoring_mech.intake_move(600);
  pros::Task awp_task(Pneumatics::awp_task5);
  chassis.drive_to_point(4,27,6,0,2,50,850);
  
  pros::delay(50);
  chassis.turn_to_point(25.2,2,1,8,2.5,30,600);
  pneumatics.matchloader_v(0);


  
  chassis.drive_to_point(25.2,2,6,0,2,30,700);
>>>>>>> Stashed changes
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
<<<<<<< Updated upstream
  pros::delay(500);
  chassis.calculate();
*/
}
=======
  std::string x_str, y_str, heading_str;
  chassis.turn_to_angle(-27);
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
}

void set_coordinates_with_ds(){
  double y_coordinate = 0;
  double x_coordinate = 0; 
  if (chassis.get_Y_position() > 0){
    y_coordinate = 71 - (chassis.distance_from_nearest_object_v() - 9.249 + 7.5);
  }
  else {
    y_coordinate = -71 + (chassis.distance_from_nearest_object_v() - 9.249 + 7.5);
  } 
  if (chassis.get_X_position() > 0){
    x_coordinate = 71 - (chassis.distance_from_nearest_object_h() - 0.95 + 6.3125);
  }
  else {
    x_coordinate = -71 + (chassis.distance_from_nearest_object_h() - 0.95 + 6.3125);
  } 
  chassis.set_coordinates(x_coordinate, y_coordinate, chassis.get_absolute_heading());
}

void set_X_coordinate_with_ds(){
  double x_coordinate = 0;
   if (chassis.get_X_position() > 0){
    x_coordinate = 71 - (chassis.distance_from_nearest_object_h() - 0.95 + 6.3125);
  }
  else {
    x_coordinate = -71 + (chassis.distance_from_nearest_object_h() - 0.95 + 6.3125);
  }  
  chassis.set_coordinates(x_coordinate, chassis.get_Y_position(), chassis.get_absolute_heading());
}

void set_Y_coordinate_with_ds(){
  double y_coordinate = 0;
  if (chassis.get_Y_position() > 0){
    y_coordinate = 71 - (chassis.distance_from_nearest_object_v() - 9.249 + 7.5);
  }
  else {
    y_coordinate = -71 + (chassis.distance_from_nearest_object_v() - 9.249 + 7.5);
  } 
  chassis.set_coordinates(chassis.get_X_position(), y_coordinate, chassis.get_absolute_heading());
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////




void skills(){ 
  
  // constants set up 
  chassis.set_turn_exit_conditions(2, 75, 950);
  chassis.set_swing_exit_conditions(1, 75, 1750);
  default_constants();
  chassis.heading_max_voltage = 2;
  
  // set coordinates, start intake
  chassis.set_coordinates(-11.75, -47.25, 337);
  scoring_mech.intake_move(600);

  // drive to 4 balls
  chassis.set_drive_exit_conditions(1.5, 1, 1300);
  chassis.drive_distance(19, 337, 10, 0.75);
  chassis.set_drive_exit_conditions(1.5, 10, 2500);
  chassis.drive_distance(1, 337, 10, 0.75);
  pros::delay(150);
  scoring_mech.intake_move(0);
  chassis.set_drive_exit_conditions(1.5, 10, 1000);

  // drive forward, align to mid goal
  chassis.drive_distance(10, 337, 6, 0.75);
  chassis.drive_distance(3, 337, 6, 0.75);
  chassis.set_turn_exit_conditions(2, 75, 1000);
  chassis.turn_to_angle(225, 5);
  chassis.set_turn_exit_conditions(2, 75, 2500);
  // drive to mid goal 
  scoring_mech.intake_move(100);
  chassis.drive_to_point(-10.25, -10.75, 10, 0.5, 1.5, 75, 900);
  chassis.set_drive_exit_conditions(1.5, 75, 2500);
  chassis.set_swing_exit_conditions(1, 75, 1750);
  //score 2 balls on mid goal
  ////scoring_mech.intake_move(-200);
  ////pros::delay(200);
  pneumatics.intakepiston_v(1);
  scoring_mech.mid_intake_move_skills(400, -100);
  pros::delay(700);
  pneumatics.intakepiston_v(0);

  // clear balls out of intake
  scoring_mech.top_goal_intake(600);

  // drive to front of loader #1
  chassis.drive_to_point(-46, -48, 6, 0.5, 1.5, 75, 2400);
  scoring_mech.top_goal_intake(0);
  chassis.set_turn_exit_conditions(2, 75, 1300);
  // turn to loader #1
  chassis.turn_to_angle(180);
  chassis.set_turn_exit_conditions(2, 75, 1000);
  // drive into & grab balls from loader #1
  pneumatics.matchloader_v(1);
  pros::delay(250);
  chassis.set_swing_exit_conditions(1, 75, 1750);
  scoring_mech.intake_move(600);
  chassis.drive_to_point(-46, -61, 7, 0.75, 1.5, 75, 1650);
  pros::delay(250);
  // drive out
  //scoring_mech.intake_move(0);
  chassis.set_turn_exit_conditions(2, 75, 1150);
  chassis.drive_to_point(-46, -51, 10, 1.5, 1.5, 75, 750);
  
  pneumatics.matchloader_v(0);

  // turn & drive to alley #1 
  chassis.turn_to_angle(145);
  chassis.drive_distance(-27, 10);
  scoring_mech.top_goal_intake(0);
  // angle to alley #1
  chassis.set_turn_exit_conditions(2, 75, 700);
  chassis.turn_to_angle(183);
  chassis.set_turn_exit_conditions(2, 75, 1500);
  //drive down alley #1
  chassis.drive_to_point(-56, 15, 7.5, 0.5, 1.5, 10, 1350);
  chassis.set_swing_exit_conditions(1, 75, 1750);

  // swing into goal
  chassis.set_swing_exit_conditions(1, 25, 2000);
  chassis.right_swing_to_angle(0, 1.65);
  chassis.set_swing_exit_conditions(1, 75, 4000);
  // score on goals
  scoring_mech.top_goal_intake(600);
  chassis.drive_distance(-5, 10, 1.5, 75, 1000, 0.75, 0.0000, 3.75, 2);
  chassis.drive_distance(-1.3, 10, 1.5, 75, 450, 0.75, 0.0000, 3.75, 2);

  // set coordinates & reset conditions
  set_coordinates_with_ds();
  chassis.set_turn_exit_conditions(2, 75, 950);
  chassis.set_coordinates(-47.5, 28.5, chassis.get_absolute_heading());
  chassis.set_swing_exit_conditions(1, 75, 1750);
  
  // align & drive into loader #2  
  pneumatics.matchloader_v(1);
  chassis.turn_to_angle(0);
  scoring_mech.top_goal_intake(0);
  scoring_mech.intake_move(-600);
  pros::delay(75);
  scoring_mech.intake_move(600);
  chassis.drive_to_point(-48.25, 61, 4, 2, 1.5, 75, 2400);
  // grab balls from loader #2
  chassis.set_turn_exit_conditions(2, 75, 950);
  pros::delay(350);
  scoring_mech.top_goal_intake(0);
  chassis.set_swing_exit_conditions(1, 75, 1750); 
  chassis.set_turn_exit_conditions(2, 75, 500);

  // align and drive into goal 
  chassis.turn_to_angle(0);
  chassis.set_turn_exit_conditions(2, 75, 2500);
  chassis.drive_to_point(-47.5, 25, 5, 2, 1.5, 75, 1000);
  // score on goal 
  scoring_mech.top_goal_intake(600);
  pros::delay(1500);
  
  
  // reset conditions
  chassis.drive_timeout = 1000;
  chassis.turn_timeout = 800;
  chassis.swing_timeout = 800;
  set_coordinates_with_ds();
  chassis.set_turn_exit_conditions(2, 75, 950);
  chassis.set_swing_exit_conditions(1, 75, 1750);

  // drive out of goal
  scoring_mech.top_goal_intake(600);
  chassis.drive_distance(6, 10);

  // turn to left of park zone
  chassis.turn_to_angle(40);
  pneumatics.matchloader_v(0);
  chassis.drive_distance(35.5, 10);
  // clear balls from intake
  scoring_mech.intake_move(600);
  // turn to park zone 
  chassis.left_swing_to_angle(74, 4);
  chassis.drive_distance(5.5, 10);
  // matchloader down & clear balls from park zone
  pneumatics.matchloader_v(1);
  chassis.set_turn_exit_conditions(2, 75, 950);
  chassis.left_swing_to_angle(80, 4);
  chassis.drive_distance(7.75, 12, 2, 0.5, 1000, 0.75, 0.0000, 3.75, 2);
  pneumatics.matchloader_v(0);
  chassis.drive_distance(30, 6.5, 2, 0.5, 800, 0.75, 0.0000, 3.75, 2);
  chassis.set_swing_exit_conditions(1, 75, 1750);
  chassis.drive_timeout = 2500;
  chassis.turn_to_angle(87);

  // intake all balls from park zone
  chassis.drive_distance(25, 5.5, 2, 0.5, 2500, 0.75, 0.0000, 3.75, 2);
  chassis.drive_distance(17, 7.5, 2, 0.5, 2500, 0.75, 0.0000, 3.75, 2);
  // grab corner ball
  chassis.right_swing_to_angle(45,3);
  scoring_mech.intake_move(600);

  // align robot to wall and reset position & conditions
  chassis.drive_distance(5.5, 5, 2, 0.5, 400, 0.75, 0.0000, 3.75, 2);
  chassis.set_coordinates(36.5, chassis.get_Y_position(), chassis.get_absolute_heading());
  scoring_mech.intake_move(0);
  chassis.set_turn_exit_conditions(2, 75, 950);
  chassis.swing_timeout = 1000;
  chassis.swing_settle_error = 1; 
  chassis.right_swing_to_angle(180, 1.5);
  chassis.drive_distance(-9, 8, 2, 0.5, 550, 0.75, 0.0000, 3.75, 2);
  chassis.set_swing_exit_conditions(1, 75, 1750);
  chassis.set_coordinates(chassis.get_X_position(), 63.5, 180);
  

  // drive forward & reset position
  chassis.drive_distance(6, 5, 2, 0.5, 550, 0.75, 0.0000, 3.75, 2);
  //pneumatics.mid_descore_v(1);
  set_X_coordinate_with_ds(); 
  chassis.set_turn_exit_conditions(2, 75, 950);

  // turn to 4 balls and drive into them
  scoring_mech.intake_move(600);
  chassis.turn_to_point(21, 22.5);
  chassis.drive_to_point(20, 22.5, 5, 1.5, 1.5, 75, 1800);
  //pneumatics.mid_descore_v(0);
  chassis.set_swing_exit_conditions(1, 75, 1750);
  scoring_mech.intake_move(600);

  // angle to mid goal put matchloader down  
  chassis.turn_to_angle(42.5, 6);
  pneumatics.matchloader_v(1);
  // drive into mid goal
  chassis.drive_to_point(9.5, 9.5, 4, 1 , 1.5, 75, 1000);
  chassis.set_turn_exit_conditions(2, 75, 200);
  chassis.turn_to_angle(42.5, 6);
  chassis.set_turn_exit_conditions(2, 75, 950);
  //chassis.drive_distance(-1.25, 5, 0.25, 0.5, 150, 0.75, 0.0000, 3.75, 2);
  chassis.drive_distance(-0.5, 5, 0.25, 0.5, 150, 0.75, 0.0000, 3.75, 2);
  scoring_mech.intake_move(0);
  // score balls on mid goal
  scoring_mech.intake_move(-300);
  pros::delay(400);
  pneumatics.intakepiston_v(1);
  int counter1 = 0; 
  int counter2 = 0;

  scoring_mech.mid_intake_move_skills(300, -50);
  pros::delay(800);
  while (counter1 < 10){
    scoring_mech.mid_intake_move_skills(500, -50);
    pros::delay(220);
    scoring_mech.mid_intake_move_skills(0, 0);
    pros::delay(50);    
    counter1 = counter1 + 1;  
  }
  while (counter1 < 14){
    scoring_mech.mid_intake_move_skills(500, -50);
    pros::delay(150);
    scoring_mech.mid_intake_move_skills(0, 0);
    pros::delay(50);    
    counter1 = counter1 + 1;  
  }

  //scoring_mech.mid_intake_move(400, -150);

  /*
  scoring_mech.mid_intake_move(400, -50);
  pros::delay(5000);
   scoring_mech.mid_intake_move(0, 0);
  pros::delay(50);
  */
  /*
  while (counter1 < 18){
    scoring_mech.mid_intake_move(400, -30);
    pros::delay(170);
    scoring_mech.mid_intake_move(0, 0);
    pros::delay(50);    
    counter1 = counter1 + 1; 
  }
  */
  

  //scoring_mech.mid_intake_move(400, -400);
  
  /* garunteed 6 balls
  while (counter1 < 10){
    scoring_mech.mid_intake_move(150, -50);
    pros::delay(200);
    scoring_mech.mid_intake_move(0, 0);
    pros::delay(50);    
    counter1 = counter1 + 1; 
  }
  */
  

  /*
  chassis.drive_distance(1.5, 10, 2, 0.5, 350, 0.75, 0.0000, 3.75, 2);
  scoring_mech.mid_intake_move(-300, -300);
  pros::delay(50);
  scoring_mech.mid_intake_move(300, 100);
  chassis.drive_distance(-1.5, 10, 2, 0.5, 350, 0.75, 0.0000, 3.75, 2);
  */
  
  pros::delay(150); 
  

  // MID GOAL SCORED

  // reset conditions 
  chassis.set_swing_exit_conditions(1, 75, 1750);
  chassis.set_turn_exit_conditions(2, 75, 950);
  
  // small drive & turn to front of loader #3
  chassis.drive_distance(3, 2, 2, 0.5, 400, 0.75, 0.0000, 3.75, 2);
  pneumatics.intakepiston_v(0);
  chassis.turn_to_point(47, 47);
  // clear balls out of intake
  scoring_mech.top_goal_intake(600);
  // drive to front of loader #3
  chassis.drive_to_point(47, 47, 5.5, 0.5, 1.5, 75, 2250);
  scoring_mech.top_goal_intake(0);
  scoring_mech.intake_move(600);
  pneumatics.intakepiston_v(0);
  chassis.set_turn_exit_conditions(2, 75, 800);
  // turn to loader #3
  chassis.turn_to_angle(0);
  chassis.set_turn_exit_conditions(2, 75, 2500);
  pneumatics.matchloader_v(1);
  pros::delay(250);
  chassis.set_swing_exit_conditions(1, 75, 1750);
  scoring_mech.top_goal_intake(0);
  scoring_mech.intake_move(600);
  // drive into loader #3 & grab balls
  chassis.drive_to_point(47, 60, 5, 1.5, 1.5, 75, 1200);
  chassis.set_turn_exit_conditions(2, 75, 950);
  pros::delay(950);

  // reset conditions
  chassis.set_drive_exit_conditions(1.5, 75, 2500);
  chassis.set_turn_exit_conditions(2, 75, 2500);
  chassis.set_swing_exit_conditions(1, 75, 4000); 
  chassis.set_swing_exit_conditions(1, 75, 1750);

  
  // drive out of loader #3
  chassis.drive_distance(-6, 10, 4, 75, 900, 0.75, 0.0000, 3.75, 2); 
  //pros::delay(200);
  scoring_mech.intake_move(0);
  pneumatics.matchloader_v(0);

  // turn & drive to alley way
  chassis.turn_to_angle(325);
  chassis.drive_distance(-25, 10);
  // align to & drive down alley #2
  chassis.turn_to_angle(0);
  chassis.drive_to_point(55.5, -17.5, 6.5, 0.5, 1.5, 75, 1500);
  
  // swing into goal & reset conditions
  chassis.set_swing_exit_conditions(1, 75, 1250);
  chassis.right_swing_to_angle(180, 1.65);
  chassis.set_swing_exit_conditions(1, 75, 4000);
  chassis.set_swing_exit_conditions(1, 75, 1750);
  chassis.set_turn_exit_conditions(2, 75, 950);
  // score balls into goal
  scoring_mech.top_goal_intake(600);
  chassis.drive_distance(-5, 10, 2, 75, 450, 0.75, 0.0000, 3.75, 2); 
  pneumatics.matchloader_v(1);
  pros::delay(1250);
  scoring_mech.top_goal_intake(0);
  scoring_mech.intake_move(600);

  
  // reset position & conditions
  set_coordinates_with_ds();
  chassis.set_turn_exit_conditions(2, 75, 950);
  chassis.set_turn_exit_conditions(2, 75, 250);

  // align to loader #4
  chassis.turn_to_angle(180);
  chassis.set_turn_exit_conditions(2, 75, 2500);  // drive into loader #4
  chassis.drive_to_point(49.25, -61, 4.75, 1.5, 1.5, 75, 1700);
  // turn & drive into goal
  chassis.set_turn_exit_conditions(2, 75, 200);
  chassis.turn_to_angle(180);
  chassis.set_turn_exit_conditions(2, 75, 2500);
  chassis.drive_to_point(47.25, -27, 5.5, 2, 1.5, 75, 1150);
  // score balls into goal
  //scoring_mech.top_goal_intake(-600);
  //pros::delay(150);
  scoring_mech.top_goal_intake(600);
  pros::delay(2000);
  scoring_mech.top_goal_intake(0);
  
  // reset conditions
  chassis.set_swing_exit_conditions(1, 75, 1750);
  chassis.set_turn_exit_conditions(2, 75, 950);
  chassis.set_turn_exit_conditions(2, 25, 700);

  // turn to left of park zone
  chassis.turn_to_angle(215);
  pneumatics.matchloader_v(0);
  // drive to left of park zone
  chassis.drive_distance(35.5, 12, 2, 10, 1450, 0.75, 0.0000, 3.75, 2);
  scoring_mech.top_goal_intake(600);
  chassis.set_swing_exit_conditions(2, 25, 800);
  // turn to park zone
  chassis.left_swing_to_angle(262, 3);
  chassis.drive_distance(1.75, 10, 2, 10, 300, 0.75, 0.0000, 3.75, 2);
  // drive into park zone and clear balls
  pneumatics.matchloader_v(1);
  scoring_mech.top_goal_intake(0);
  scoring_mech.intake_move(600);
  chassis.drive_distance(8.5, 10, 2, 10, 350, 0.75, 0.0000, 3.75, 2);
  pneumatics.matchloader_v(0);
  chassis.drive_distance(28, 10);
  chassis.drive_distance(2, 10);
  chassis.turn_to_angle(290);
  
   
}
>>>>>>> Stashed changes
