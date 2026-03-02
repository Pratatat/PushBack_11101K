
#include "main.h"
#include "pros/adi.hpp"
#include "pros/rtos.hpp"

void default_constants(){
  // (maxVoltage, kP, kI, kD, startI)
  chassis.set_drive_constants(12, 0.65, 0.0001, 4.45, 2);
  chassis.set_heading_constants(12, 0.28, 0.00015, 1.9, 5); // same as turn
   chassis.set_turn_constants(12, 0.28, 0.00015, 1.9, 5);
  chassis.set_swing_constants(12, 0.56, 0.00001, 3.65, 5);

  // (settle_error, settle_time, timeout)
  chassis.set_drive_exit_conditions(1, 50, 2500);
  chassis.set_turn_exit_conditions(1, 50, 2500);
  chassis.set_swing_exit_conditions(1, 50, 4000);
}

void skills_constants()
{
   // (maxVoltage, kP, kI, kD, startI)
  chassis.set_drive_constants(12, 0.65, 0.0001, 4.48, 2.5);
  chassis.set_heading_constants(12, 0.28, 0.000, 1.95, 4); // same as turn
  chassis.set_turn_constants(12, 0.28, 0.000, 1.95, 4);
  chassis.set_swing_constants(12, 0.56, 0.00001, 3.65, 5);

  // (settle_error, settle_time, timeout)
  chassis.set_drive_exit_conditions(1, 50, 2500);
  chassis.set_turn_exit_conditions(1, 50, 2500);
  chassis.set_swing_exit_conditions(1, 50, 4000);
}

void odom_constants(){
  default_constants();
  chassis.heading_max_voltage = 10;
  chassis.drive_max_voltage = 12;
  chassis.drive_settle_error = 3;
}


void drive_test(){
  chassis.drive_timeout = 1000;
  chassis.turn_timeout = 800;
  chassis.turn_settle_time = 30;
  chassis.drive_settle_time = 30;
  chassis.drive_settle_error - 2;
  chassis.turn_settle_error - 1.5;
  std::string left_front,left_middle, left_back;
  skills_constants();
  chassis.heading_max_voltage=0;
  chassis.set_coordinates(0,0,0);
  pros::delay(900);
  //chassis.drive_distance(24);
  chassis.drive_to_point(0, 24, 10, 0);
  pros::delay(100);
  std::cout << "After Drive1:" << chassis.get_X_position() << " " << 
    chassis.get_Y_position() << " " << chassis.get_absolute_heading() << std::endl;

  chassis.turn_to_point(24, 24, 1, 10);
  pros::delay(100);
  std::cout << "After Turn1:" << chassis.get_X_position() << " " << 
    chassis.get_Y_position() << " " << chassis.get_absolute_heading() << std::endl;
  
    chassis.drive_to_point(24, 24, 10, 0);
  pros::delay(100);
  std::cout << "After Drive1:" << chassis.get_X_position() << " " << 
    chassis.get_Y_position() << " " << chassis.get_absolute_heading() << std::endl;

  chassis.turn_to_point(24, 0, 1, 10);
  pros::delay(100);
  std::cout << "After Turn1:" << chassis.get_X_position() << " " << 
    chassis.get_Y_position() << " " << chassis.get_absolute_heading() << std::endl;
  
  chassis.drive_to_point(24, 0, 10, 0);
  pros::delay(100);
  std::cout << "After Drive1:" << chassis.get_X_position() << " " << 
    chassis.get_Y_position() << " " << chassis.get_absolute_heading() << std::endl;

  chassis.turn_to_point(0, 0, 1, 10);
  pros::delay(100);
  std::cout << "After Turn1:" << chassis.get_X_position() << " " << 
    chassis.get_Y_position() << " " << chassis.get_absolute_heading() << std::endl;
    
  chassis.drive_to_point(0, 0, 10, 0);
  pros::delay(100);
  std::cout << "After Drive1:" << chassis.get_X_position() << " " << 
    chassis.get_Y_position() << " " << chassis.get_absolute_heading() << std::endl;

  chassis.turn_to_point(0, 24, 1, 10);
  pros::delay(100);
  std::cout << "After Turn1:" << chassis.get_X_position() << " " << 
    chassis.get_Y_position() << " " << chassis.get_absolute_heading() << std::endl;

  

  left_front = std::to_string(chassis.get_X_position());
  left_middle = std::to_string(chassis.get_Y_position());
  left_back = std::to_string(chassis.get_absolute_heading());
  pros::screen::draw_rect(0,0,480,240);
    pros::screen::set_pen(pros::Color::white);
    pros::screen::print(TEXT_LARGE, 50, 50, left_front.c_str());
    pros::screen::print(TEXT_LARGE, 50, 100, left_middle.c_str());
    pros::screen::print(TEXT_LARGE, 50, 150, left_back.c_str());
}


void turn_test(){
  std::string left_front,left_middle, left_back;
  default_constants();
  chassis.set_coordinates(0, 0, 0);
  pros::delay(1000);
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
 chassis.set_coordinates(7, -0.5, 270);
 chassis.drive_max_voltage=8;
  chassis.drive_to_point(-21,-0.5,5.5,0,3,50,1000);
 pneumatics.matchloader_v(1);
 scoring_mech.intake_move(600);
 chassis.turn_to_point(-23,-48,1,8,2.5,50,750);
  chassis.drive_to_point(-23,-15,5.5,0,3,50,900);


  chassis.drive_to_point(-24.75,4,6.5,0,5,30,700);
 pros::Task intae(Scoring_Mech::intake_autontask);
 chassis.drive_to_point(-24.9,26,6.5,3,3,30,1000);
 chassis.drive_distance(-12, 12,5,30,800,0.75, 0.0000, 3.75, 2);
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
 chassis.set_coordinates(7, -0.5, 270);
 chassis.drive_max_voltage=8;
  chassis.drive_to_point(-21,-0.5,5.5,0,3,50,1000);
 pneumatics.matchloader_v(1);
 scoring_mech.intake_move(600);
 chassis.turn_to_point(-23,-48,1,8,2.5,50,750);
  chassis.drive_to_point(-23,-15,5.5,0,3,50,900);


  chassis.drive_to_point(-24.75,4,6.5,0,5,30,700);
 pros::Task intae(Scoring_Mech::intake_autontask);
 chassis.drive_to_point(-24.9,26,6.5,3,3,30,1000);
  pros::delay(250);
 //pros::delay(350);
 pneumatics.matchloader_v(0);
 chassis.set_coordinates(-23.6, 19.6, chassis.get_absolute_heading()-0.5);
  chassis.turn_to_point(-14.7,10,1,8,5,30,500);
  chassis.drive_to_point(-14.7,10,7,1,5,30,600);
  chassis.turn_to_point(-9,-96,1,8,5,30,500);
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
 chassis.set_coordinates(-6.5, -1.5, 90);
 chassis.drive_to_point(21.35,-1.5,9,0,2,50,1000);
 pneumatics.matchloader_v(1);
 pneumatics.intakepiston_v(1);
 scoring_mech.intake_move(600);
 chassis.turn_to_point(24.9,-24,1,8,2.5,30,700);
 scoring_mech.intake_move(600);
 chassis.drive_to_point(24,-15,6,0,3,50,800);


 chassis.drive_to_point(24.2,4,6,4,5.5,30,850);


 pros::Task intae(Scoring_Mech::intake_autontask);
 chassis.drive_to_point(24.2,25,6,0.5,3,30,1000);
 chassis.drive_distance(-13, 12,5,30,680,0.75, 0.0000, 3.75, 2);
 pneumatics.matchloader_v(0);
 chassis.set_coordinates(23.6, 19.6, chassis.get_absolute_heading()+1.5);
  chassis.turn_to_point(-48.75,24.75,1,8,2.5,50,900);
 pneumatics.hood_v(0);
 pros::Task awp(Pneumatics::awp_task_new);
 chassis.drive_to_point(-48.75,24.75,7,1,3,50,1500);
 pneumatics.intakepiston_v(1);
 chassis.turn_to_point(-69.5,5,1,8,3,50,800);
 scoring_mech.top_goal_intake(0);
 pneumatics.matchloader_v(0);
 chassis.drive_to_point(-69.5,5,7,1,3,50,950);
 chassis.turn_to_point(-68.5,-48,1,8,2.5,50,900);
 pros::Task intae2(Scoring_Mech::intake_autontask);
 chassis.drive_to_point(-70,25,6,0.5,3,30,1000);
 chassis.drive_distance(-13, 12,5,30,680,0.75, 0.0000, 3.75, 2);
 chassis.drive_to_point(-69.5,4.5,7,1,5,10,700);
 pneumatics.matchloader_v(1);
 chassis.turn_to_point(-68.5,-48,1,8,2.5,20,900);
 scoring_mech.intake_move(600);
 pneumatics.intakepiston_v(0);
 chassis.drive_to_point(-70,-16.25,6,0.5,3,20,1000);
 pros::delay(90);
 chassis.drive_to_point(-69.5,-3,7,1,3,20,850);
 chassis.turn_to_point(-96,-30,1,8,2.5,20,900);
 scoring_mech.intake_move(600);
 chassis.drive_to_point(-34.45,37.25,7,1,3,50,1500);
 pneumatics.hood_v(1);
 pneumatics.matchloader_v(0);
  /*
 scoring_mech.intake_move(600);
 pros::Task awp_task(Pneumatics::awp_task);
 chassis.drive_to_point(-4.5,25.5,7,1,3,30,1000);
 pros::delay(150);
 pneumatics.matchloader_v(0);
 chassis.turn_to_point(-48.75,24.75,1,8,2.5,50,900);
 pros::Task awp_task2(Pneumatics::awp_task3);
 chassis.drive_to_point(-48.75,24.75,7,1,3,50,1500);
 pros::delay(150);
 chassis.turn_to_point(-72,-3.5,1,8,2.5,50,800);
 scoring_mech.top_intake_move(90);
  //pros::Task awp_task3(Pneumatics::awp_task4);


 chassis.drive_to_point(-34.45,37.25,6.25,1,3,50,1000);
 scoring_mech.mid_intake_move(600);
 chassis.drive_distance(-5,9.2,5,25,300,0.75, 0.0000, 3.75, 2);
 pneumatics.matchloader_v(0);
 pros::delay(800);
 scoring_mech.mid_intake_move(0);
 pneumatics.intakepiston_v(1);
 chassis.drive_to_point(-47,25.5,7,0,4,50,1000);
 chassis.turn_to_point(-69.5,0,1,8,3,50,800);
 scoring_mech.top_goal_intake(0);
 chassis.drive_to_point(-69.5,0,7,1,3,50,950);
 pneumatics.matchloader_v(1);
  chassis.turn_to_point(-68.5,-48,1,8,2.5,50,900);
 scoring_mech.intake_move(600);
  chassis.drive_to_point(-70,-16.5,6,0.5,3,50,1000);
  pros::delay(60);
 chassis.drive_to_point(-71, 6,7,1.5,4,50,1000);
 //chassis.turn_to_point(-67.4,-48,1,8,2.5,50,900);
 pros::Task intae2(Scoring_Mech::intake_autontask);
 chassis.drive_to_point(-71,24,7,0.5,1.5,50,1000);
 chassis.drive_distance(-28,12,5,25,600,0.75, 0.0000, 3.75, 2);
 */
}



void RightSeven(){
 // 4+3
 default_constants();
 chassis.set_coordinates(-6.5, -1.5, 90);
 chassis.drive_to_point(21.35,-1.5,9,0,2,50,1000);
 pneumatics.matchloader_v(1);
 pneumatics.intakepiston_v(1);
 scoring_mech.intake_move(600);
 chassis.turn_to_point(24.9,-24,1,8,2.5,30,700);
 scoring_mech.intake_move(600);
 chassis.drive_to_point(24,-15,6,0,3,50,800);


 chassis.drive_to_point(24.2,4,6,4,5.5,30,850);


 pros::Task intae(Scoring_Mech::intake_autontask);
 chassis.drive_to_point(24.2,25,6,0.5,3,30,1000);
 chassis.drive_distance(-13, 12,5,30,680,0.75, 0.0000, 3.75, 2);
 pneumatics.matchloader_v(0);
 chassis.set_coordinates(23.6, 19.6, chassis.get_absolute_heading()+1.5);
 
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
 //chassis.drive_distance(-6.5, 8,5,30,310,0.75, 0.0000, 3.75, 2);
 pros::delay(850);
  scoring_mech.top_goal_intake(0);
 chassis.drive_to_point(11,14,7,1,2,30,1000);
 chassis.turn_to_point(10,90,1,8,2.5,50,800);
 int end_time = pros::millis();
 chassis.set_brake_mode('C');
  chassis.drive_to_point(14.75,36,9.5,0,3,10,1000);
  chassis.turn_to_point(-25,60,1,8,5,50,600);
}



void RightAWP(){
  // 4+3+3
 default_constants();
 chassis.set_coordinates(-6.5, -1.5, 90);
 chassis.drive_to_point(21.35,-1.5,9,0,2,50,1000);
 pneumatics.matchloader_v(1);
 pneumatics.intakepiston_v(1);
 scoring_mech.intake_move(600);
 chassis.turn_to_point(24.9,-24,1,8,2.5,30,700);
 scoring_mech.intake_move(600);
 chassis.drive_to_point(24,-15,6,0,3,50,800);


 chassis.drive_to_point(24.2,4,6,4,5.5,30,850);


 pros::Task intae(Scoring_Mech::intake_autontask);
 chassis.drive_to_point(24.2,25,6,0.5,3,30,1000);
 chassis.drive_distance(-13, 12,5,30,680,0.75, 0.0000, 3.75, 2);
 pneumatics.matchloader_v(0);
 chassis.set_coordinates(23.6, 19.6, chassis.get_absolute_heading()+1.5);
 
  
  
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
 chassis.set_coordinates(-6.5, -1.5, 90);
 chassis.drive_to_point(21.35,-1.5,9,0,2,50,1000);
 pneumatics.matchloader_v(1);
 pneumatics.intakepiston_v(1);
 scoring_mech.intake_move(600);
 chassis.turn_to_point(24.9,-24,1,8,2.5,30,700);
 scoring_mech.intake_move(600);
 chassis.drive_to_point(24,-15,6,0,3,50,800);


 chassis.drive_to_point(24.2,4,6,4,5.5,30,850);


 pros::Task intae(Scoring_Mech::intake_autontask);
 chassis.drive_to_point(24.2,25,6,0.5,3,30,1000);
 chassis.drive_distance(-12, 12,5,30,600,0.75, 0.0000, 3.75, 2);
 pneumatics.matchloader_v(0);
 chassis.set_coordinates(23.6, 19.6, chassis.get_absolute_heading()+1.5);

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

void auton_setup() {
 default_constants();
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
  chassis.set_turn_exit_conditions(1.5, 20, 800);
  chassis.set_swing_exit_conditions(1, 20, 1750);
  chassis.set_drive_exit_conditions(1.5, 20, 1000);
  skills_constants();
  chassis.heading_max_voltage = 2;
  chassis.turn_max_voltage = 10;
  chassis.drive_max_voltage = 10;
  
  
  // set coordinates, start intake
  chassis.set_coordinates(-11.81, -46.75, 337);
  scoring_mech.intake_move(600);

  // drive to 4 balls
  chassis.drive_distance(27, 337, 10, 0.75);
  chassis.set_turn_exit_conditions(1.5, 50, 850);
  chassis.turn_to_angle(220, 5);
  
  // drive back and score balls
  chassis.drive_distance(-15.5, 222, 10, 0.75);
  pneumatics.intakepiston_v(0);
  pros::delay(100);
  scoring_mech.mid_goal_score(500, 200);
  pros::delay(400);
  chassis.turn_to_point(-46, -45, 1, 10, 2.5, 30, 400);
  pneumatics.hood_v(0);
  chassis.drive_to_point(-46, -45, 8, 0.5, 2.5, 30, 1800);

  chassis.turn_to_point(-47.5, -61, 1, 8, 1.5, 30, 450);
  pneumatics.matchloader_v(1);
  pros::delay(250);
  scoring_mech.intake_move(600);
  pneumatics.intakepiston_v(1);
  chassis.drive_to_point(-47.5, -62, 10, 0.5, 2.5, 30, 1700);

  chassis.drive_to_point(-47, -51, 10, 0.5, 2.5, 30, 700);
  chassis.turn_to_angle(145);
  chassis.drive_distance(-26, 10);
  pneumatics.matchloader_v(0);

  chassis.turn_to_angle(185);
  chassis.drive_to_point(-58.25, 17, 9, 1, 1.5, 10, 1200);
  chassis.set_swing_exit_conditions(1, 25, 2000);
  chassis.right_swing_to_angle(0, 1.65);

  scoring_mech.intake_move(600);
  pneumatics.hood_v(1);
  chassis.drive_distance(-5, 10, 2.5, 1, 800, 0.65, 0.0001, 4.48, 2.5); //default timeout is already 1000 - for driving while scoring

  // set coordinates & reset conditions
  
  
  // align & drive into loader #2  
  chassis.set_coordinates(-47.5, 28.5, chassis.get_absolute_heading());
  scoring_mech.intake_move(0);
  pneumatics.matchloader_v(1);
  chassis.drive_distance(10, 8, 2.5, 1, 550, 0.65, 0.0001, 4.48, 2.5);
  chassis.turn_to_point(-47.5, 72, 1, 10, 1.5, 2, 450);
  scoring_mech.intake_move(600);
  chassis.drive_to_point(-47.5, 61, 4, 2.5, 1.5, 75, 2400);
  
  // grab balls from loader #2
  scoring_mech.top_goal_intake(0);
  pros::delay(350);
  
  chassis.drive_distance(-10, 10, 2.5, 1, 550, 0.75, 0.0000, 3.75, 2);
  // align and drive into goal 
  chassis.turn_to_angle(0, 10, 1.5, 2, 330, 0.28, 0, 1.95, 4);
  chassis.drive_to_point(-47.5, 25, 5, 2, 1.5, 75, 1000);


  // score on goal 
  scoring_mech.top_goal_intake(600);
  pneumatics.hood_v(1);
  chassis.drive_distance(-5, 10, 1.5, 75, 1500, 0.75, 0.0000, 3.75, 2);

  
  /*pneumatics.hood_v(1);
  chassis.set_coordinates(-71+chassis.distance_from_nearest_object_h(), 71-chassis.distance_from_nearest_object_v(), chassis.get_absolute_heading());

  // drive out of goal
  chassis.drive_distance(6, 10);

  // turn to left of park zone
  chassis.turn_to_angle(35);
  pneumatics.matchloader_v(0);
  //chassis.drive_distance(35.5, 10);
  // clear balls from intake
  // turn to park zone 

  chassis.drive_until(11.5, 6);
  // clear balls from intake
  scoring_mech.intake_move(600);
  // turn to park zone 
  chassis.set_swing_exit_conditions(1, 25, 500);
  chassis.left_swing_to_angle(74, 4);
  chassis.set_swing_exit_conditions(1, 25, 900);


  chassis.drive_distance(10, 10);
  // matchloader down & clear balls from park zone
  pneumatics.matchloader_v(1);

  chassis.set_turn_exit_conditions(2, 75, 950);
  chassis.left_swing_to_angle(80, 4);
  chassis.drive_distance(12, 12, 2, 0.5, 200, 0.75, 0.0000, 3.75, 2);
  pneumatics.matchloader_v(0);
  chassis.drive_distance(30, 8  , 2, 0.5, 800, 0.75, 0.0000, 3.75, 2);
  chassis.set_swing_exit_conditions(1, 75, 1750);
  
  chassis.turn_to_angle(87);

  // intake all balls from park zone
  chassis.drive_distance(25, 5.5, 2, 0.5, 2500, 0.75, 0.0000, 3.75, 2);
  chassis.drive_distance(17, 7.5, 2, 0.5, 2500, 0.75, 0.0000, 3.75, 2);
  // grab corner ball
  chassis.right_swing_to_angle(45,3);
  scoring_mech.intake_move(600);


  // align robot to wall and reset position & conditions
  chassis.drive_distance(5.5, 5, 2, 0.5, 400, 0.75, 0.0000, 3.75, 2);
  //chassis.set_coordinates(36.5, chassis.get_Y_position(), chassis.get_absolute_heading());
  scoring_mech.intake_move(0);
  chassis.set_turn_exit_conditions(2, 75, 950);
  chassis.swing_timeout = 1000;
  chassis.swing_settle_error = 1; 
  chassis.left_swing_to_angle(0, 3.5);
  chassis.set_coordinates(chassis.get_X_position(), 71- chassis.distance_from_nearest_object_v(), chassis.get_absolute_heading());
  chassis.turn_to_angle(20,10);

  
   

  
  chassis.right_swing_to_angle(180, 1.5);
  chassis.drive_distance(-25, 12, 2, 0.5, 1000, 0.75, 0.0000, 3.75, 2);
  chassis.set_swing_exit_conditions(1, 75, 1750);
  //chassis.set_coordinates(chassis.get_X_position(), 63.5, 180);
  
  
  // drive forward & reset position
  chassis.drive_distance(6, 5, 2, 0.5, 550, 0.75, 0.0000, 3.75, 2);
  //pneumatics.mid_descore_v(1);
  chassis.set_coordinates(71- chassis.distance_from_nearest_object_h(),chassis.get_Y_position(), chassis.get_absolute_heading());
  
  pros::delay(200);
  std::string x_str, y_str, heading_str;

    x_str = std::to_string(chassis.get_X_position());
    y_str = std::to_string(chassis.get_Y_position());
    heading_str = std::to_string(chassis.get_absolute_heading());
    pros::screen::draw_rect(0,0,480,240);
    pros::screen::set_pen(pros::Color::white);
    pros::screen::print(TEXT_LARGE, 50, 50, x_str.c_str());
    pros::screen::print(TEXT_LARGE, 50, 125, y_str.c_str());
    pros::screen::print(TEXT_LARGE, 50, 175, heading_str.c_str()); 
 
  
  // turn to 4 balls and drive into them
  scoring_mech.intake_move(600);
  chassis.turn_to_point(22, 22.5);
  chassis.drive_to_point(22, 22.5, 5, 1.5, 1.5, 75, 1800);

  chassis.turn_to_angle(45, 6);
  pneumatics.matchloader_v(1);
  // drive into mid goal
  chassis.drive_to_point(15, 15, 4, 1 , 1.5, 75, 1000);
  scoring_mech.mid_goal_score(500, 100);

  /*
  









  chassis.set_drive_exit_conditions(1.5, 10, 2500);
  //pros::delay(150);
  //scoring_mech.intake_move(0);
  chassis.set_drive_exit_conditions(1.5, 10, 1000);

  // drive forward, align to mid goal
  chassis.drive_distance(10, 337, 6, 0.75);
  chassis.drive_distance(-3, 337, 6, 0.75);
  chassis.set_turn_exit_conditions(1, 75, 1000);
  chassis.turn_to_angle(220, 5);
  chassis.set_turn_exit_conditions(1, 75, 2500);
  // drive to mid goal 
  chassis.drive_to_point(-13, -14, 10, 0.5, 1.5, 75, 900);
  

  pros::delay(1000);

  chassis.set_drive_exit_conditions(1.5, 75, 2500);
  chassis.set_swing_exit_conditions(1, 75, 1750);
  //score 2 balls on mid goal 
  scoring_mech.mid_intake_move(175);
  pros::delay(1000);
  scoring_mech.mid_intake_move(0);

  pneumatics.hood_v(0);

  pneumatics.intakepiston_v(1);
  

  // clear balls out of intake
  //scoring_mech.intake_move(600);

  
  // drive to front of loader #1
  chassis.turn_to_point(-38, -50, 10, 0.5, 2.5, 2, 400);
  chassis.drive_to_point(-38, -50, 6, 0.5, 2.5, 2, 1600);

  
  


  
  chassis.set_turn_exit_conditions(2, 2, 400);
  // turn to loader #1
  chassis.turn_to_angle(180);
  chassis.turn_to_point(-44.5, -61, 6, 0.5, 1.5, 2, 100);
  chassis.set_turn_exit_conditions(2, 75, 1000);
  // drive into & grab balls from loader #1
  pros::delay(250);
  pneumatics.matchloader_v(1);


  
  pros::delay(250);
  chassis.set_swing_exit_conditions(1, 75, 1750);
  scoring_mech.intake_move(600);
  chassis.drive_to_point(-44.5, -63, 7, 0.75, 1.5, 75, 1750);
  pros::delay(250);
  // drive out
  //scoring_mech.intake_move(0);
  chassis.set_turn_exit_conditions(2, 75, 1150);
  chassis.drive_to_point(-44.5, -51, 10, 0.5, 1.5, 75, 750);
  

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
  chassis.drive_to_point(-56, 12, 7.5, 0.5, 1.5, 10, 1350);
  chassis.set_swing_exit_conditions(1, 75, 1750);

  // swing into goal
  chassis.set_swing_exit_conditions(1, 25, 2000);

  chassis.right_swing_to_angle(0, 1.7);
  chassis.set_swing_exit_conditions(1, 75, 4000);

  //std::string x_str, y_str, heading_str;

    x_str = std::to_string(chassis.get_X_position());
    y_str = std::to_string(chassis.get_Y_position());
    heading_str = std::to_string(chassis.get_absolute_heading());
    pros::screen::draw_rect(0,0,480,240);
    pros::screen::set_pen(pros::Color::white);
    pros::screen::print(TEXT_LARGE, 50, 50, x_str.c_str());
    pros::screen::print(TEXT_LARGE, 50, 125, y_str.c_str());
    pros::screen::print(TEXT_LARGE, 50, 175, heading_str.c_str()); 

  
  
  // score on goals
  scoring_mech.top_goal_intake(600);
  pneumatics.hood_v(1);
  chassis.drive_distance(-5, 10, 1.5, 75, 1000, 0.75, 0.0000, 3.75, 2);
  chassis.drive_distance(-1.3, 10, 1.5, 75, 450, 0.75, 0.0000, 3.75, 2);
  
  /*

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
*/
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
  /*
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
  
   */
}

void distance_sensor_test(){
  chassis.set_coordinates(-71+chassis.distance_from_nearest_object_h(), 71-chassis.distance_from_nearest_object_v(), 270);
    
    std::string x_str, y_str, heading_str;
     x_str = std::to_string(chassis.get_X_position());
     y_str = std::to_string(chassis.get_Y_position());
    heading_str = std::to_string(chassis.get_absolute_heading());
    pros::screen::draw_rect(0,0,480,240);
    pros::screen::set_pen(pros::Color::white);
    pros::screen::print(TEXT_LARGE, 50, 50, x_str.c_str());
    pros::screen::print(TEXT_LARGE, 50, 125, y_str.c_str());
    pros::screen::print(TEXT_LARGE, 50, 175, heading_str.c_str());
    
  chassis.drive_until(10, 2);
}