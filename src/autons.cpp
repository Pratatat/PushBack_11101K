#include "main.h"
#include "pros/adi.hpp"
#include "pros/rtos.hpp"

void default_constants(){
  // (maxVoltage, kP, kI, kD, startI)
  chassis.set_drive_constants(12, 0.75, 0.0000, 3.75, 2);
  chassis.set_heading_constants(12, 0.27, 0.0001, 1.83, 5);
   chassis.set_turn_constants(12, 0.27, 0.0001, 1.83, 5);
  chassis.set_swing_constants(12, 0.56, 0.00001, 3.65, 5);

  // (settle_error, settle_time, timeout)
  chassis.set_drive_exit_conditions(1.5, 50, 2500);
  chassis.set_turn_exit_conditions(2, 50, 2500);
  chassis.set_swing_exit_conditions(1, 75, 4000);
}
/*
// Old Robot Constants
void default_constants(){
  // (maxVoltage, kP, kI, kD, startI)
  chassis.set_drive_constants(12, 1, 0.00005, 8.65, 5);
  chassis.set_heading_constants(12, 0.270, 0.00032, 2.29, 5);
   chassis.set_turn_constants(12, 0.270, 0.00032, 2.29, 5);
  chassis.set_swing_constants(12, 0.50, 0.00001, 3.65, 5);

  // (settle_error, settle_time, timeout)
  chassis.set_drive_exit_conditions(1, 50, 2500);
  chassis.set_turn_exit_conditions(1.5, 50, 2500);
  chassis.set_swing_exit_conditions(1, 75, 4000);
}
 */

void odom_constants(){
  default_constants();
  chassis.heading_max_voltage = 10;
  chassis.drive_max_voltage = 12;
  chassis.drive_settle_error = 3;
}


void drive_test(){
  default_constants();
  chassis.heading_max_voltage=0;
  chassis.set_coordinates(0,0,0);
  chassis.print_odom_vals();
  //chassis.drive_distance(48);
  chassis.drive_to_point(0,48);
  chassis.print_odom_vals();
  pros::delay(900);
  chassis.set_brake_mode('C');
}


void turn_test(){
  std::string left_front,left_middle, left_back;
  default_constants();
  chassis.set_coordinates(0, 0, 0);
  pros::delay(4000);
  chassis.turn_to_angle(90);
  left_front = std::to_string(chassis.get_X_position());
  left_middle = std::to_string(chassis.get_Y_position());
  left_back = std::to_string(chassis.get_absolute_heading());
  pros::screen::draw_rect(0,0,480,240);
    pros::screen::set_pen(pros::Color::white);
    pros::screen::print(TEXT_LARGE, 50, 50, left_front.c_str());
    pros::screen::print(TEXT_LARGE, 50, 100, left_middle.c_str());
    pros::screen::print(TEXT_LARGE, 50, 150, left_back.c_str());
}


void swing_test(){
  std::string left_front,left_middle, left_back;
  default_constants();
  chassis.set_coordinates(0, 0, 0);
  chassis.right_swing_to_angle(182,4.5);
  left_front = std::to_string(chassis.get_X_position());
  left_middle = std::to_string(chassis.get_Y_position());
  left_back = std::to_string(chassis.get_absolute_heading());
  pros::screen::draw_rect(0,0,480,240);
    pros::screen::set_pen(pros::Color::white);
    pros::screen::print(TEXT_LARGE, 50, 50, left_front.c_str());
    pros::screen::print(TEXT_LARGE, 50, 100, left_middle.c_str());
    pros::screen::print(TEXT_LARGE, 50, 150, left_back.c_str());
}

void tank_odom_test(){
  default_constants();
  chassis.set_coordinates(0, 0, 0);
  
  chassis.turn_max_voltage = 6;
  chassis.drive_max_voltage = 6;
  
  //chassis.turn_to_point(24, 24,0,4);
  //chassis.drive_to_point(24, 24,4,0,1,200,2000);
  //chassis.turn_to_angle(90,9);
  
  //chassis.drive_to_point(0, 24);
  //chassis.drive_distance(48,1.6,1,200,10000,0.48,0,0.65,0);
  //printf("Drive: X: %f, Y: %f, Heading: %f\n", chassis.get_X_position(), chassis.get_Y_position(), chassis.get_absolute_heading());
  

  
  chassis.turn_to_point(0, 48);
  printf("turn: X: %f, Y: %f, Heading: %f\n", chassis.get_X_position(), chassis.get_Y_position(), chassis.get_absolute_heading());

  chassis.drive_to_point(0, 48);
  printf("Drive: X: %f, Y: %f, Heading: %f\n", chassis.get_X_position(), chassis.get_Y_position(), chassis.get_absolute_heading());
  
  chassis.turn_to_point(48, 48);
  printf("Turn: X: %f, Y: %f, Heading: %f\n", chassis.get_X_position(), chassis.get_Y_position(), chassis.get_absolute_heading());
 // pros::delay(100);
  
  chassis.drive_to_point(48, 48);
  printf("Drive: X: %f, Y: %f, Heading: %f\n", chassis.get_X_position(), chassis.get_Y_position(), chassis.get_absolute_heading());
 // pros::delay(100);
  chassis.turn_to_point(48, 0);
  printf("Turn: X: %f, Y: %f, Heading: %f\n", chassis.get_X_position(), chassis.get_Y_position(), chassis.get_absolute_heading());
 // pros::delay(100);
  chassis.drive_to_point(48, 0);
  printf("Drive: X: %f, Y: %f, Heading: %f\n", chassis.get_X_position(), chassis.get_Y_position(), chassis.get_absolute_heading());
 // pros::delay(100);
  chassis.turn_to_point(0, 0);
  printf("Turn: X: %f, Y: %f, Heading: %f\n", chassis.get_X_position(), chassis.get_Y_position(), chassis.get_absolute_heading());
  //pros::delay(100);
  chassis.drive_to_point(0, 0);
  printf("Drive: X: %f, Y: %f, Heading: %f\n", chassis.get_X_position(), chassis.get_Y_position(), chassis.get_absolute_heading());
  //pros::delay(100);
  chassis.turn_to_point(0, 48);
  printf("Turn: X: %f, Y: %f, Heading: %f\n", chassis.get_X_position(), chassis.get_Y_position(), chassis.get_absolute_heading());
  //pros::delay(100);

  
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


void LeftSeven(){
  // 4+3 
  default_constants();
  chassis.set_coordinates(7, -1.5, 270);
  chassis.drive_max_voltage=8;
  int start_time = pros::millis();
  chassis.drive_to_point(-21,-1.5,5.5,0,3,50,1000);
  pneumatics.matchloader_v(1);
  scoring_mech.intake_move(600);
  chassis.turn_to_point(-23,-48,1,8,2.5,50,750);
  
  chassis.drive_to_point(-23,-15,5.5,0,3,50,850);

  
  chassis.drive_to_point(-24.75,4,6.5,0,5,30,700);
  pros::Task intae(Scoring_Mech::intake_autontask);
  chassis.drive_to_point(-24.9,26,6.5,3,3,30,1000);
  chassis.drive_distance(-9.75, 12,5,30,600,0.75, 0.0000, 3.75, 2);
  //pros::delay(350);
  pneumatics.matchloader_v(0);
  chassis.set_coordinates(-23.6, 19.6, chassis.get_absolute_heading()-0.5);
  chassis.turn_to_point(4.5,25,1,8,2.5,50,700);
  pros::Task awp_task(Pneumatics::awp_task);
  scoring_mech.intake_move(600);
  
  chassis.drive_to_point(3,24.75,5.5,1,3,30,1000);
  pros::delay(150);

  // EXTRA BALL
  
  chassis.turn_to_point(-11,38,1,8,4,30,700);
  pneumatics.matchloader_v(0);
  chassis.drive_to_point(-11,38,6,1,3,30,1000);
  chassis.turn_to_point(-18,40,1,8,4,30,700);
  pneumatics.matchloader_v(0);
  chassis.drive_to_point(-18,40,3.5,0.5,3,30,700);
  pros::delay(300);
  chassis.drive_to_point(3,25,5.5,4,3,30,1000);
  
  
  scoring_mech.mid_intake_move(0);
  chassis.turn_to_point(-24,-10,1,8,2.5,50,700);
  pneumatics.matchloader_v(0);
  pros::Task awp_task3(Pneumatics::awp_task6);
  chassis.drive_to_point(15.5,40.5,7,1,2,30,900);
  scoring_mech.mid_intake_move(600);
  chassis.turn_to_point(-13.5,14,1,8,4,30,450);
  chassis.drive_distance(-4.5, 8,5,30,310,0.75, 0.0000, 3.75, 2);
  pros::delay(500);
  scoring_mech.mid_intake_move(350);
  pros::delay(450);
  
  

  /*
  // MID DESCORE
  chassis.drive_distance(12, 12,5,30,850,0.75, 0.0000, 3.75, 2);
  pneumatics.descore_v(1);
  pros::delay(300);
  scoring_mech.mid_intake_move(0);
  pneumatics.intakepiston_v(0);
  chassis.drive_distance(-18, 3,5,30,850,0.75, 0.0000, 3.75, 2);
  chassis.drive_distance(-67, 2,2,120,3850,0.75, 0.0000, 3.75, 2);
  */
  
  
  
  
  pneumatics.intakepiston_v(0);
  chassis.drive_to_point(-13,14,8,1,4,30,900);

  chassis.turn_to_angle(168, 12, 8, 25, 380, 0.27, 0.00032, 2.29, 5);
  
  chassis.drive_distance(-25, 12,6,25,700,0.75, 0.0000, 3.75, 2);
  chassis.turn_to_angle(210, 12, 8, 25, 380, 0.27, 0.0001, 1.83, 5);
  
  /**/
}

void LeftFour(){
  default_constants();
  chassis.set_coordinates(7, -1.5, 270);
  chassis.drive_max_voltage=8;
  int start_time = pros::millis();
  chassis.drive_to_point(-21,-1.5,6.5,0,3,50,1000);
  pneumatics.matchloader_v(1);
  chassis.turn_to_point(-23,-48,1,8,2.5,50,900);
  
  scoring_mech.intake_move(600);
  
  chassis.drive_to_point(-23,-15,5,0,3.5,50,900);

  pros::delay(50);
  chassis.drive_to_point(-24.5,4,8,1,5,30,1000);
  pros::Task intae(Scoring_Mech::intake_autontask3);
  chassis.drive_to_point(-24.5,26,8,3,3.5,30,1000);
  pros::delay(250);
  pneumatics.matchloader_v(0);
  chassis.set_coordinates(-23.6, 19.6, chassis.get_absolute_heading()-0.5);
  chassis.turn_to_point(-14.7,10,1,8,5,30,500);
  chassis.drive_to_point(-14.7,10,7,1,5,30,600);
  chassis.turn_to_point(-9,-96,1,8,5,30,500);
  int end_time = pros::millis();
  int total_time = (end_time - start_time);
  pros::screen::print(TEXT_LARGE, 50, 150, std::to_string(total_time).c_str());
  chassis.drive_to_point(-13,38,10.5,1,1,30,800);
  
  chassis.turn_to_point(-65,-60,1,8,5,30,500);
  
  /**/
}

void LeftSevenElim(){
  // 7 Ball
  default_constants();
  chassis.set_coordinates(9, -3.24, 334.5);
  scoring_mech.intake_move(600);
  pros::Task awp_task(Pneumatics::awp_task5);
  chassis.drive_to_point(-4,27,6,0,2,50,850);
  
  pros::delay(50);
  chassis.turn_to_point(-26.2,2,1,8,2.5,30,600);
  pneumatics.matchloader_v(0);
  chassis.drive_to_point(-26.2,2,6,0,2,30,800);
  scoring_mech.intake_move(0);
  
  chassis.turn_to_point(-26.2,-18,1,8,2.5,30,600);
  scoring_mech.intake_move(600);
  pneumatics.matchloader_v(1);
  chassis.drive_to_point(-26.2,-18,6.75,1,3,30,1050);

  chassis.drive_to_point(-26.7,6,5.5,6,6,30,650);

  pros::Task intae(Scoring_Mech::intake_autontask);
  chassis.drive_to_point(-26.7,25,5,0.5,3,30,800);
  chassis.drive_distance(-13.5, 12,5,30,780,0.75, 0.0000, 3.75, 2);
  pros::delay(200);
  pneumatics.matchloader_v(0);
  chassis.set_coordinates(-23.6, 19.6, chassis.get_absolute_heading()-0.5);
  chassis.turn_to_point(-14.7,10,1,8,5,30,500);
  chassis.drive_to_point(-14.7,10,7,1,5,30,600);
  chassis.turn_to_point(-9,-96,1,8,5,30,500);
  chassis.drive_to_point(-13,38,10.5,1,1,30,800);
  
  chassis.turn_to_point(-65,-60,1,8,5,30,500);
  chassis.turn_to_point(-75,-60,1,8,5,30,3500);
}

void AWP(){
  default_constants();
  chassis.set_coordinates(-7, -1.5, 90);
  chassis.drive_to_point(20.55,-1.5,6,0,2,50,1000);
  pneumatics.matchloader_v(1);
  scoring_mech.intake_move(600);
  chassis.turn_to_point(24.7,-24,1,8,2.5,30,700);
  scoring_mech.intake_move(600);
  chassis.drive_to_point(23.6,-14.85,4.7,0,3,50,850);

  chassis.drive_to_point(23.8,4,5.5,4,5.5,30,800);
  

  pros::Task intae(Scoring_Mech::intake_autontask);
  chassis.drive_to_point(23.8,25,5,0.5,3,30,1000);
  chassis.drive_distance(-13, 12,5,30,680,0.75, 0.0000, 3.75, 2);
  pneumatics.matchloader_v(0);
  chassis.set_coordinates(23.6, 19.6, chassis.get_absolute_heading()+1.5);
  chassis.turn_to_point(-4.5,25.5,1,8,2.5,50,900);
  
  scoring_mech.intake_move(600);
  pros::Task awp_task(Pneumatics::awp_task);
  chassis.drive_to_point(-4.5,25.5,6,1,3,30,1000);
  pros::delay(150);
  pneumatics.matchloader_v(0);
  chassis.turn_to_point(-47.75,24.75,1,8,2.5,50,900);
  pros::Task awp_task2(Pneumatics::awp_task3);
  chassis.drive_to_point(-47.75,24.75,6.25,1,3,50,1500);
  pros::delay(150);
  chassis.turn_to_point(-72,-3.5,1,8,2.5,50,800);
  scoring_mech.top_intake_move(90);
  
  pros::Task awp_task3(Pneumatics::awp_task4); 

  chassis.drive_to_point(-34.45,37.25,6.25,1,3,50,1000);
  scoring_mech.mid_intake_move(600);
  chassis.drive_distance(-8.5,9.2,5,25,300,0.75, 0.0000, 3.75, 2);
  pneumatics.matchloader_v(0);
  pros::delay(440);
  scoring_mech.mid_intake_move(0);
  pneumatics.intakepiston_v(0);
  chassis.drive_to_point(-47,25.5,5.5,0,4,50,1000);
  chassis.turn_to_point(-69,0,1,8,3,50,800);
  scoring_mech.top_goal_intake(0);
  chassis.drive_to_point(-69,0,6.25,1,3,50,950);
  pneumatics.matchloader_v(1);
  
  chassis.turn_to_point(-68.5,-48,1,8,2.5,50,900);
  scoring_mech.intake_move(600);
  
  chassis.drive_to_point(-70,-16.5,5.5,0.5,3,50,1000);
  
  pros::delay(60);
  chassis.drive_to_point(-71, 6,6.25,1.5,4,50,1000);
  //chassis.turn_to_point(-67.4,-48,1,8,2.5,50,900);
  pros::Task intae2(Scoring_Mech::intake_autontask);
  chassis.drive_to_point(-71,24,6.5,0.5,1.5,50,1000);
  chassis.drive_distance(-28,12,5,25,600,0.75, 0.0000, 3.75, 2);
  
}


void RightSeven(){
  // 4+3
  default_constants();
  chassis.set_coordinates(-7, -1.5, 90);
  int start_time = pros::millis();
  chassis.drive_to_point(20.45,-1.5,6,0,2,50,900);
  
  pneumatics.matchloader_v(1);
  scoring_mech.intake_move(600);
  chassis.turn_to_point(24.4,-24,1,8,2.5,30,900);
  scoring_mech.intake_move(600);
  chassis.drive_to_point(23.6,-14.85,5,0,3,50,850);

  chassis.drive_to_point(23.6,4,5.5,6,5.5,30,800);
  

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

  chassis.turn_to_point(-13.25,35.5,7,8,2.5,50,700);
  pneumatics.matchloader_v(0);
  scoring_mech.top_goal_intake(0);
  chassis.drive_to_point(-13.25,35.5,8,1,3,30,800);
  
  scoring_mech.top_goal_intake(-100);
  chassis.drive_distance(-6.5, 8,5,30,310,0.75, 0.0000, 3.75, 2);
  pros::delay(850);
  
  scoring_mech.top_goal_intake(0);
  chassis.drive_to_point(12.6,14,7,1,3.5,30,900);
  chassis.turn_to_point(10,90,1,8,2.5,50,800);
  int end_time = pros::millis();
  chassis.set_brake_mode('C');
   chassis.drive_to_point(14.75,36,9.5,0,3,10,1000);
   int total_time = (end_time - start_time);
   pros::screen::print(TEXT_LARGE, 50, 150, std::to_string(total_time).c_str());
   chassis.turn_to_point(-25,60,1,8,5,50,600);
}


void RightAWP(){
  // 4+3+3
  default_constants();
  chassis.set_coordinates(-7, -1.5, 90);
  int start_time = pros::millis();
  chassis.drive_to_point(20.45,-1.5,6,0,2,50,900);
  
  pneumatics.matchloader_v(1);
  scoring_mech.intake_move(600);
  chassis.turn_to_point(24.4,-24,1,8,2.5,30,900);
  scoring_mech.intake_move(600);
  chassis.drive_to_point(23.6,-14.85,5,0,3,50,850);

  chassis.drive_to_point(23.6,4,5.5,6,5.5,30,800);
  

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
  scoring_mech.intake_move(0);
  pneumatics.matchloader_v(1);
  chassis.turn_to_point(25.9,-18,1,8,2.5,30,600);
  scoring_mech.intake_move(600);
  chassis.drive_to_point(25.7,-18,6,1,3,30,1150);
  pros::delay(150);
  chassis.drive_to_point(25.9,6,5.5,6,6,30,650);

  pros::Task intae(Scoring_Mech::intake_autontask);
  chassis.drive_to_point(25.9,25,5,0.5,3,30,800);
  chassis.drive_distance(-13.5, 12,5,30,780,0.75, 0.0000, 3.75, 2);
  pros::delay(150);
  pneumatics.matchloader_v(0);
  chassis.set_coordinates(23.6, 19.6, 181.5);

  chassis.turn_to_point(34.25,10,1,8,5,50,400);
  chassis.drive_to_point(34.25,10,7,1,8,30,600);
  chassis.turn_to_point(50,-96,1,8,5,50,400);
  chassis.drive_to_point(14.75,35.5,10.5,0,1,100,800);
  //pros::delay(500);
  chassis.turn_to_point(-25,-60,1,8,5,30,500);
  chassis.turn_to_point(-35,-60,1,8,5,30,3500);

}


void right_auton_setup() {
  default_constants();
  chassis.set_brake_mode('C');
  chassis.set_coordinates(0, 0, 0);
  std::string x_str, y_str, heading_str;
  chassis.turn_to_angle(27);
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

void left_auton_setup() {
  default_constants();
  chassis.set_brake_mode('C');
  chassis.set_coordinates(0, 0, 0);
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
