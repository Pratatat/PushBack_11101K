#include "main.h"
#include "pros/adi.hpp"

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


void LeftSevenAWP(){
  default_constants();
  chassis.set_coordinates(7, -1.5, 270);
  chassis.drive_max_voltage=8;
  chassis.drive_to_point(-22.5,-1.5,5.5,0,3,50,1000);
  pneumatics.matchloader_v(1);
  chassis.turn_to_point(-23,-48,1,8,2.5,50,900);
  scoring_mech.intake_move(600);
  
  chassis.drive_to_point(-23,-15,5,0,3,50,900);

  pros::delay(100);
  chassis.drive_to_point(-24,4,6,1,3.5,30,1000);
  pros::Task intae(Scoring_Mech::intake_autontask);
  chassis.drive_to_point(-24.1,24,6,3,2.5,30,1000);
  pros::delay(300);
  pneumatics.matchloader_v(0);
  chassis.set_coordinates(-23.6, 19.6, 180);
  pros::delay(250);
  chassis.turn_to_point(4.5,24.5,1,8,2.5,50,900);
  pros::Task awp_task(Pneumatics::awp_task5);
  scoring_mech.intake_move(600);
  
  chassis.drive_to_point(4.5,24.5,5.5,1,3,30,1000);
  pros::delay(250);
  pneumatics.matchloader_v(0);
  scoring_mech.mid_intake_move(0);
  chassis.turn_to_point(-24,-8,1,8,2.5,50,900);
  pros::Task awp_task3(Pneumatics::awp_task4);
  chassis.drive_to_point(15,39,6.25,1,2,30,1000);
  scoring_mech.mid_intake_move(600);
  pros::delay(900);
  scoring_mech.mid_intake_move(0);
  pneumatics.intakepiston_v(0);
  chassis.drive_to_point(-15,14,6.25,1,3.5,30,1000);
  chassis.turn_to_angle(180, 12, 5, 25, 400, 0.27, 0.00032, 2.29, 5);

  chassis.drive_distance(-25, 12,5,30,850,0.75, 0.0000, 3.75, 2);
  pros::delay(500);
  chassis.turn_to_angle(220, 12, 5, 25, 400, 0.27, 0.0001, 1.83, 5);

  
  /**/
}

void LeftFour(){
  default_constants();
  chassis.set_coordinates(7, -1.5, 270);
  chassis.drive_max_voltage=8;
  chassis.drive_to_point(-22.5,-1.5,6.5,0,3,50,1000);
  pneumatics.matchloader_v(1);
  chassis.turn_to_point(-24.75,-24,1,8,2.5,50,900);
  
  scoring_mech.intake_move(600);
  
  chassis.drive_to_point(-23,-15,5,0,3.5,50,900);

  pros::delay(100);
  chassis.drive_to_point(-24,4,8,1,3.5,30,1000);
  pros::Task intae(Scoring_Mech::intake_autontask3);
  chassis.drive_to_point(-24,24,8,3,2.5,30,1000);
  pros::delay(450);
  pneumatics.matchloader_v(0);
  chassis.set_coordinates(-23.6, 19.6, 180);
  chassis.turn_to_point(-15.5,10,1,8,5,30,500);
  chassis.drive_to_point(-15.5,10,7,1,5,30,600);
  chassis.turn_to_point(-15.5,-96,1,8,5,30,500);

  chassis.drive_distance(-27.5, 12,8,25,600,0.75, 0.0000, 3.75, 2);
  pros::delay(500);
  chassis.turn_to_angle(220, 12, 12, 25, 300, 0.27, 0.0001, 1.83, 5);
  
  /**/
}


void AWP(){
  default_constants();
  chassis.set_coordinates(-7, -1.5, 90);
  chassis.drive_max_voltage=8;
  chassis.drive_to_point(21,-1.5,6,0,3,50,1000);
  pneumatics.matchloader_v(1);
  scoring_mech.intake_move(600);
  chassis.turn_to_point(25,-24,1,8,2.5,50,900);
  
  chassis.drive_to_point(24.25,-14.85,5.5,0,3,50,900);
  chassis.drive_to_point(24,4,6,1,3.5,30,1000);
  
  pros::Task intae(Scoring_Mech::intake_autontask);
  chassis.drive_to_point(24,24,6.5,3,2.5,30,1000);
  chassis.drive_distance(-15, 12,5,30,850,0.75, 0.0000, 3.75, 2);
  pros::delay(25);
  pneumatics.matchloader_v(0);
  chassis.set_coordinates(23.6, 19.6, 180);
  
  
  chassis.turn_to_point(-4.5,25.1,1,8,2.5,50,900);
  
  scoring_mech.intake_move(600);
  pros::Task awp_task(Pneumatics::awp_task);
  chassis.drive_to_point(-4.5,25.1,6,1,3,30,1000);
  pros::delay(250);
  pneumatics.matchloader_v(0);
  chassis.turn_to_point(-48,23.7,1,8,2.5,50,900);
  pros::Task awp_task2(Pneumatics::awp_task3);
  chassis.drive_to_point(-48,23.7,6.25,1,3,50,2000);
  pros::delay(250);
  
  chassis.turn_to_point(-72,-8,1,8,2.5,50,900);
  scoring_mech.mid_intake_move(0);
  pros::Task awp_task3(Pneumatics::awp_task4); 
  
  chassis.drive_to_point(-35,39,6.25,1,3,50,1000);
  scoring_mech.mid_intake_move(400);
  pros::delay(700);
  scoring_mech.mid_intake_move(0);
  pneumatics.intakepiston_v(0);
  chassis.drive_to_point(-47,23.7,5.5,0,3,50,1000);
  chassis.turn_to_point(-70.75,0,1,8,2.5,50,900);
  chassis.drive_to_point(-70.75,0,6.25,1,3,50,1000);
  pneumatics.matchloader_v(1);
  
  chassis.turn_to_point(-68.8,-48,1,8,2.5,50,900);
  scoring_mech.intake_move(600);
  chassis.drive_to_point(-71,-16.5,6.25,1,3,50,1000);
  
  pros::delay(100);
  chassis.drive_to_point(-72, 6,6.25,1,4,50,1000);
  chassis.turn_to_point(-67.5,-48,1,8,2.5,50,900);
  pros::Task intae2(Scoring_Mech::intake_autontask);
  chassis.drive_to_point(-72.5,24,6.5,6,1.5,50,1000);
  chassis.drive_distance(-28,12,3,25,600,0.75, 0.0000, 3.75, 2);
 
}


void RightSevenAWP(){
  default_constants();
  chassis.set_coordinates(-7, -1.5, 90);
  chassis.drive_max_voltage=8;
  chassis.drive_to_point(21,-1.5,6,0,3,50,1000);
  pneumatics.matchloader_v(1);
  scoring_mech.intake_move(600);
  chassis.turn_to_point(25,-24,1,8,2.5,50,900);
  
  chassis.drive_to_point(24.25,-14.85,5.5,0,3,50,900);
  chassis.drive_to_point(24,4,6,1,3.5,30,1000);
  
  pros::Task intae(Scoring_Mech::intake_autontask);
  chassis.drive_to_point(24,24,6.5,3,2.5,30,1000);
  chassis.drive_distance(-15, 12,5,30,850,0.75, 0.0000, 3.75, 2);
  chassis.set_coordinates(23.6, 19.6, 180);
  pros::delay(25);
  pneumatics.matchloader_v(0);
  
  
  chassis.turn_to_point(-4.5,25.1,1,8,2.5,50,900);
  
  scoring_mech.intake_move(600);
  pros::Task awp_task(Pneumatics::awp_task);
  chassis.drive_to_point(-4.5,25.1,6,1,3,30,1000);
  pros::delay(250);
  chassis.turn_to_point(-13.25,35.6,1,8,2.5,50,900);
  pneumatics.matchloader_v(0);
  chassis.drive_to_point(-13.25,35.6,6,1,3,30,1000);
  scoring_mech.top_goal_intake(-150);
  pros::delay(600);
  chassis.drive_to_point(15,14,6.25,1,3.5,30,1000);
  scoring_mech.top_goal_intake(0);
  chassis.turn_to_angle(1.5, 12, 5, 25, 400, 0.27, 0.00032, 2.29, 5);
  chassis.drive_distance(26, 12,5,30,850,0.75, 0.0000, 3.75, 2);
  pros::delay(500);
  chassis.turn_to_angle(-30, 12, 5, 25, 400, 0.27, 0.0001, 1.83, 5);
}

void RightFour(){
  default_constants();
  chassis.set_coordinates(-7, -1.5, 90);
  chassis.drive_max_voltage=8;
  chassis.drive_to_point(21,-1.5,6.5,0,3,50,1000);
  pneumatics.matchloader_v(1);
  
  
  chassis.turn_to_point(25,-24,1,8,2.5,50,900);
  
  scoring_mech.intake_move(600);
  chassis.drive_to_point(24.25,-15.25,4.8,0,3,50,900);
  chassis.drive_to_point(24,4,8,1,3.5,30,900);
  
  pros::Task intae(Scoring_Mech::intake_autontask3);
  chassis.drive_to_point(24,26,8,3,2.5,30,1000);
  pneumatics.matchloader_v(0);
  pros::delay(420);
  chassis.set_coordinates(23.6, 19.6, 180);
  chassis.turn_to_point(33.8,10,1,8,5,50,600);
  chassis.drive_to_point(33.8,10,7,1,8,30,700);
  chassis.turn_to_point(38,-96,1,8,5,50,600);
  chassis.drive_distance(-27.5, 12,8,25,600,0.75, 0.0000, 3.75, 2);
  pros::delay(500);
  chassis.turn_to_angle(220, 12, 12, 25, 300, 0.27, 0.0001, 1.83, 5);
  
  /**/
}

void LeftSevenQual(){
  //-setting up-//
  default_constants();  
  //turn PID: 0.27, 0.00032, 2.29
  //drive PID: 1, 0.00005, 8.65
  chassis.heading_max_voltage = 2;
  chassis.set_drive_exit_conditions(1.5, 40, 770);
  chassis.set_turn_exit_conditions(1.5, 40, 640);
  chassis.set_coordinates(-17.75, -49.25, 345.1);

  //-head towards the first 3 balls-//
  scoring_mech.intake_move(600);
  chassis.drive_to_point(-22, -32.5, 12, 2, 1.5, 5, 1000);
  pneumatics.matchloader_v(1);
  chassis.drive_to_point(-24, -23);

  //-head towards the goal-//
  chassis.turn_to_point(-48.5, -37, 2, 10, 1.5, 40, 550);
  chassis.drive_to_point(-48.5, -37, 10, 0.5, 1.5, 45, 770);
  chassis.turn_to_angle(176, 12, 1.5, 30, 390, 0.27, 0.00032, 2.29, 5);
  chassis.drive_distance(-11, 180, 12, 2);
  scoring_mech.top_goal_intake(600);
  chassis.set_coordinates(-48, -28, chassis.get_absolute_heading()); 
  pros::delay(700);
  scoring_mech.top_goal_intake(0);
  scoring_mech.intake_move(600);

  //-head towards matchloader-//
  chassis.drive_to_point(-47, -59, 5.25, 0.75, 1.5, 50, 1350);
  pros::delay(250);

  //-scoring the 3 balls-//
  chassis.drive_to_point(-48, -26, 7, 0.8, 1.5, 50, 1350);
  scoring_mech.top_goal_intake(600);
  chassis.set_coordinates(-48, -28, chassis.get_absolute_heading());
  pros::delay(750);
  pneumatics.matchloader_v(0);
  scoring_mech.intake_move(0);
  chassis.drive_settle_time = 450; chassis.drive_distance(3); chassis.drive_settle_time = 900;

  //-set up for wing push-//
  pneumatics.wing_v(1);
  chassis.set_swing_constants(5, 0.50, 0.00001, 3.65, 5);
  chassis.set_swing_exit_conditions(1.5, 35, 1250);
  chassis.swing_settle_time = 15;
  chassis.drive_settle_time = 15; chassis.drive_timeout = 570;
  chassis.turn_settle_time = 15;  chassis.turn_timeout = 400;
  
  chassis.right_swing_to_angle(7, 2);
  chassis.turn_to_point(-48, -3); 
  chassis.drive_distance(25.5);
  pneumatics.wing_v(0);
  chassis.left_swing_to_angle(0, 4);
  chassis.drive_distance(10, 12);
  chassis.turn_to_angle(35);
 /* chassis.drive_settle_time = 10;
  chassis.drive_distance(23, 320, 9, 4.5);
  chassis.drive_distance(9);
  /*pneumatics.wing_v(0);
  chassis.drive_distance(30, 0, 12, 5);
  chassis.set_turn_constants(12, 0.270, 0.00032, 2.29, 5);
  //chassis.drive_to_point(-37.8, -24, 8, 1.5);8?*/

}

void LeftSevenElim(){
  default_constants();
  chassis.set_coordinates(13.73, 0.9, 0);
  chassis.turn_to_point(0,24.8,1,8,2.5,75,800);
  scoring_mech.intake_move(600);
  pros::Task awp_task2(Pneumatics::awp_task);
  chassis.drive_to_point(0,24.8,5.5,2);
  
  chassis.turn_to_point(-23,5,1,8,2.5,75,800);
  chassis.drive_to_point(-23,5,5,2);
  
  scoring_mech.intake_move(0);
  pneumatics.matchloader_v(1);
  chassis.turn_to_point(-23,-48,1,8,2.5,10,800);
  scoring_mech.intake_move(600);

   pros::Task intae(Scoring_Mech::intake_autontask);
  chassis.drive_to_point(-23.45,23,8,0.5);
  scoring_mech.top_goal_intake(600);
  pneumatics.matchloader_v(1);
  pros::delay(350);
  scoring_mech.top_goal_intake(0);
  chassis.turn_to_point(-23.45,-16,1,8,2.5,10,800);
  scoring_mech.intake_move(600);
  chassis.drive_to_point(-23.45,-16,4.35,2);
  pros::delay(125);
  pros::Task intae2(Scoring_Mech::intake_autontask2);
  chassis.drive_to_point(-23.2,23,8,1);
  pneumatics.matchloader_v(0);
  scoring_mech.top_goal_intake(600);
  pros::delay(750);
  

  chassis.drive_to_point(-23.2,7);
  pneumatics.wing_v(1);
  chassis.turn_to_point(0,-13.5,1.5,8,2.5,10,800);
  chassis.drive_distance(-18,12,1.5,50,600,0.4325, 0, 0.91, 0);
  chassis.turn_to_angle(181,10,2,65,550,0.46, 0, 5, 0);
  pneumatics.wing_v(0);
  chassis.drive_distance(-28,12,1.5,50,600,0.4325, 0, 0.91, 0);

  

}

void RightSevenBall(){
  default_constants();
  chassis.set_coordinates(-13.73, 0.6, 0);
  chassis.turn_to_point(0,25,1,8,2.5,75,800);
  scoring_mech.intake_move(600);
  pros::Task awp_task2(Pneumatics::awp_task);
  chassis.drive_to_point(0,25,7,2);
  pros::delay(350);
  chassis.turn_to_point(24.4,5,1.5,8,2.5,75,800);
  
  chassis.drive_to_point(24.4,5,5,2);
  pneumatics.matchloader_v(0);
  scoring_mech.intake_move(0);

  pneumatics.matchloader_v(1);
  chassis.turn_to_point(24.8,-48,1,8,2.5,75,800);
  scoring_mech.intake_move(600);
  chassis.drive_to_point(24.75,-16,4.5,0.5);
  
  pros::delay(75);
  pros::Task intae2(Scoring_Mech::intake_autontask2);
  chassis.drive_to_point(24.55,23,6,2);
  scoring_mech.top_goal_intake(600);
  pneumatics.matchloader_v(0);
  pros::delay(1250);
  

  chassis.drive_to_point(24,7);
  pneumatics.wing_v(1);
  chassis.turn_to_point(48,-13.5,1,8,2.5,75,800);
  chassis.drive_distance(-14,12,1.5,50,600,0.4325, 0, 0.91, 0);
  chassis.turn_to_angle(182,10,2,65,550,0.46, 0, 5, 0);
  pneumatics.wing_v(0);
  chassis.drive_distance(-28,12,1.5,50,600,0.4325, 0, 0.91, 0);
  chassis.set_brake_mode('B');
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
}