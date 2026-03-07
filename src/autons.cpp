
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
  chassis.drive_to_point(-23,-15,6,0,3,50,950);


  chassis.drive_to_point(-24.75,4,6.5,0,5,30,700);
  pros::Task intake(Scoring_Mech::intake_autontask);
  chassis.drive_to_point(-24.9,26,5,3,3,30,1000);
  chassis.drive_distance(-12, 12,5,30,250,0.75, 0.0000, 3.75, 2);
  //pros::delay(350);
  pneumatics.matchloader_v(0);
  chassis.set_coordinates(-24, 18.5, chassis.get_absolute_heading()-0.5);
  chassis.turn_to_point(0,24,1,7,2.5,50,700);
  scoring_mech.intake_move(600);
  pros::Task awp_task(Pneumatics::awp_task);

  chassis.drive_to_point(3,24,5.5,1,3,30,1000);
  pros::delay(150);
  pneumatics.matchloader_v(0);
  chassis.turn_to_point(-24,-2,1,8,2.5,50,700);
  pneumatics.intakepiston_v(0);
  chassis.drive_to_point(13.75,37.75,7,0.5,2,30,900);
  scoring_mech.mid_intake_move(300);
  // chassis.turn_to_point(-13.5,14,1,8,4,30,450);
  pros::delay(650);
  
  pneumatics.intakepiston_v(0);
  chassis.drive_to_point(-11.75,17,8,1,2,30,900);

  chassis.turn_to_angle(176, 8, 1, 2, 450, 0.28, 0.00015, 1.9, 5);
  chassis.drive_distance(-22, 10);
  chassis.turn_to_point(-65,-60,1,8,5,30,500);
  /**/
}

void LeftNine(){
  // 4+3+2
  default_constants();
  chassis.set_coordinates(7, -0.5, 270);
  chassis.drive_max_voltage=8;
  chassis.drive_to_point(-21,-0.5,5.5,0,3,50,1000);
  pneumatics.matchloader_v(1);
  scoring_mech.intake_move(600);
  chassis.turn_to_point(-23,-48,1,8,2.5,50,750);
  chassis.drive_to_point(-23,-15,6,0,3,50,950);


  chassis.drive_to_point(-24.75,4,6.5,0,5,30,700);
  pros::Task intake(Scoring_Mech::intake_autontask);
  chassis.drive_to_point(-24.9,26,5,3,3,30,1000);
  chassis.drive_distance(-12, 12,5,30,250,0.75, 0.0000, 3.75, 2);
  //pros::delay(350);
  pneumatics.matchloader_v(0);
  chassis.set_coordinates(-23.6, 19.6, chassis.get_absolute_heading()-0.5);
  chassis.turn_to_point(4.5,25,1,8,2.5,50,700);
  pros::Task awp_task(Pneumatics::awp_task);
  scoring_mech.intake_move(600);

  chassis.drive_to_point(3,24.75,5.5,1,3,30,1000);
  pros::delay(150);

  // EXTRA BALL
  pneumatics.matchloader_v(0);
  chassis.turn_to_point(-11,38,1,8,4,30,700);
  chassis.drive_to_point(-11,38,6,1,3,30,1000);
  chassis.turn_to_point(-20,40,1,8,4,30,700);
  chassis.drive_to_point(-20,40,3.5,0.5,3,30,650);
  //pneumatics.matchloader_v(1);
  pros::delay(350);
  chassis.turn_to_point(-30,48,1,8,4,30,700);
  //scoring in the middle
  
 chassis.drive_to_point(3,24.75,6,1,3,30,1200);
  pros::delay(150);
  pneumatics.matchloader_v(0);
  chassis.turn_to_point(-24,-2,1,8,2.5,50,700);
  pneumatics.intakepiston_v(0);
  chassis.drive_to_point(13.5,38.25,7,0.5,2,30,900);
  scoring_mech.mid_intake_move(320);
  pros::delay(650);



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




  chassis.drive_to_point(-11.5,17,8,1,2,30,900);

  chassis.turn_to_angle(178.5, 8, 1, 2, 450, 0.28, 0.00015, 1.9, 5);
  chassis.drive_distance(-22, 10);
  chassis.turn_to_point(-65,-60,1,8,5,30,500);

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
  chassis.drive_to_point(-23,-15,5.5,0,3,50,945);


  chassis.drive_to_point(-24.75,4,6.5,0,5,30,700);
 pros::Task intae(Scoring_Mech::intake_autontask);
 chassis.drive_to_point(-24.9,26,6,1,3,30,1000);
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
  chassis.set_coordinates(11.25, -1.26, 334.6);
  scoring_mech.intake_move(600);
  pros::Task awp_task(Pneumatics::awp_task5);
  chassis.drive_to_point(-4,27,6,0,2,50,850);
  pneumatics.intakepiston_v(1);
  pros::delay(50);
  chassis.turn_to_point(-24.5,1,1,8,2.5,30,600);
  pneumatics.matchloader_v(0);
  chassis.drive_to_point(-24.5,1,6,0,2,30,900);
  scoring_mech.intake_move(0);
  
  chassis.turn_to_point(-24.7,-18,1,7,2,30,750);
  scoring_mech.intake_move(600);
  pneumatics.matchloader_v(1);
  chassis.drive_to_point(-24.7,-18,5.75,1,3,30,1050);

  chassis.drive_to_point(-25.3,6,5.5,6,6,30,650);

  pros::Task intae(Scoring_Mech::intake_autontask);
  chassis.drive_to_point(-25.3,25,5,0.5,3,30,800);
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
 chassis.set_coordinates(-7.5, -1.5, 90);
 chassis.drive_to_point(21.35,-1.5,9,0,2,50,1000);
 pneumatics.matchloader_v(1);
 pneumatics.intakepiston_v(1);
 scoring_mech.intake_move(600);
 chassis.turn_to_point(24.9,-24,1,8,2.5,30,700);
 scoring_mech.intake_move(600);
 chassis.drive_to_point(24,-16,5,0,3,50,850);


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
 chassis.turn_to_point(-70.5,5,1,8,3,50,800);//
 scoring_mech.top_goal_intake(0);
 pneumatics.matchloader_v(0);
 pneumatics.hood_v(0);
 chassis.drive_to_point(-71,5,7,1,3,50,950);//
 chassis.turn_to_point(-68.5,-48,1,8,2.5,50,900);
 pros::Task intae2(Scoring_Mech::intake_autontask);
 chassis.drive_to_point(-71,25,6,0.5,3,30,1000);
 chassis.drive_distance(-13, 12,5,30,680,0.75, 0.0000, 3.75, 2);
 chassis.drive_to_point(-70,7.5,7,1,5,10,700);
 pneumatics.matchloader_v(1);
 chassis.turn_to_point(-70.75,-48,1,8,2.5,20,900); //-69.25
 scoring_mech.intake_move(600);
 pneumatics.intakepiston_v(0);
 chassis.drive_to_point(-70.75,-16.25,5,0.5,3,20,1070);
 pros::delay(110);
 chassis.drive_to_point(-70.75,0,7,1,3,20,850);
 chassis.turn_to_point(-96,-24,1,8,2.5,20,900);
 scoring_mech.mid_intake_move_skills(600, 300);
 pneumatics.hood_v(0);
 chassis.drive_to_point(-33.45,37.25,7,1,3,50,1500);
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
 chassis.set_coordinates(-7.5, -1.5, 90);
 chassis.drive_to_point(21.35,-1.5,9,0,2,50,1000);
 pneumatics.matchloader_v(1);
 pneumatics.intakepiston_v(1);
 scoring_mech.intake_move(600);
 chassis.turn_to_point(24.9,-24,1,8,2.5,30,700);
 scoring_mech.intake_move(600);
 chassis.drive_to_point(24,-16,5,0,3,50,900);


 chassis.drive_to_point(24.2,4,6,4,5.5,30,800);


 pros::Task intae(Scoring_Mech::intake_autontask);
 chassis.drive_to_point(24.2,25,6,0.5,3,30,1000);
 chassis.drive_distance(-13, 12,5,30,130,0.75, 0.0000, 3.75, 2);
 pneumatics.matchloader_v(0);
 chassis.set_coordinates(23.6, 19.6, chassis.get_absolute_heading()+1.5);
chassis.turn_to_point(-4,24.75,1,8,2.5,50,900);
 pneumatics.hood_v(0);
 scoring_mech.intake_move(0);
scoring_mech.bottom_intake_move(600);
 pros::Task awp(Pneumatics::awp_task);
 chassis.drive_to_point(-4,25.75,7,1,3,50,1500);


 chassis.turn_to_point(-24,48,7,8,2.5,50,700);
 pneumatics.matchloader_v(0);
 scoring_mech.top_goal_intake(0);
 chassis.drive_to_point(-14.5,38,8,1,3,30,800);
 pneumatics.intake_piston_bottom_v(1);
  scoring_mech.top_goal_intake(-300);
 //chassis.drive_distance(-6.5, 8,5,30,310,0.75, 0.0000, 3.75, 2);
 pros::delay(850);
  scoring_mech.top_goal_intake(0);
 chassis.drive_to_point(11,14,7,1,2,30,1000);
 chassis.turn_to_point(11,90,1,8,2.5,50,800);
 int end_time = pros::millis();
 chassis.set_brake_mode('C');
  chassis.drive_to_point(14.75,36,9.5,0,3,10,1000);
  chassis.turn_to_point(-25,60,1,6,5,50,600);
}



void RightAWP(){

 default_constants();
 chassis.set_coordinates(-7.5, -1.5, 90);
 chassis.drive_to_point(21.35,-1.5,9,0,2,50,1000);
 pneumatics.matchloader_v(1);
 pneumatics.intakepiston_v(1);
 scoring_mech.intake_move(600);
 chassis.turn_to_point(24.9,-24,1,8,2.5,30,700);
 scoring_mech.intake_move(600);
 chassis.drive_to_point(24,-16,5,0,3,50,900);


 chassis.drive_to_point(24.2,4,6,4,5.5,30,800);


 pros::Task intae(Scoring_Mech::intake_autontask);
 chassis.drive_to_point(24.2,25,6,0.5,3,30,1000);
 chassis.drive_distance(-13, 12,5,30,130,0.75, 0.0000, 3.75, 2);
 pneumatics.matchloader_v(0);
 chassis.set_coordinates(23.6, 19.6, chassis.get_absolute_heading()+1.5);
chassis.turn_to_point(-4,24.75,1,8,2.5,50,900);
 pneumatics.hood_v(0);
 scoring_mech.intake_move(0);
scoring_mech.bottom_intake_move(600);
 pros::Task awp(Pneumatics::awp_task);
 chassis.drive_to_point(-4,25.75,7,1,3,50,1500);


 chassis.turn_to_point(-24,48,7,8,2.5,50,700);
 pneumatics.matchloader_v(0);
 scoring_mech.top_goal_intake(0);
 chassis.drive_to_point(-14.5,38,8,1,3,30,800);
 pneumatics.intake_piston_bottom_v(1);
  scoring_mech.top_goal_intake(-300);
 //chassis.drive_distance(-6.5, 8,5,30,310,0.75, 0.0000, 3.75, 2);
 pros::delay(850);


  scoring_mech.top_goal_intake(0);
  scoring_mech.intake_move(600);
  chassis.drive_to_point(-4.5,25.5,6.25,1,3.5,30,900);
  pneumatics.intake_piston_bottom_v(0);
  chassis.turn_to_point(-48,24.75,1,8,2.5,50,900);
  pros::Task awp_task2(Pneumatics::awp_task8);
  chassis.drive_to_point(-48,24.75,6.25,1,3,50,1500);
  pros::delay(200);
  scoring_mech.intake_move(600);
  chassis.turn_to_point(-72,-4,1,8,2.5,50,800);
  pneumatics.intakepiston_v(0);
  chassis.drive_to_point(-35.2,37,6.25,0,3,50,1000);
  scoring_mech.mid_intake_move(600);
  pros::delay(500);
   scoring_mech.mid_intake_move(400);
   pros::delay(500);
   pneumatics.matchloader_v(0);
  scoring_mech.mid_intake_move(0);

}

void RightFour(){
 default_constants();
 chassis.set_coordinates(-7.5, -1.5, 90);
 chassis.drive_to_point(21.35,-1.5,9,0,2,50,1000);
 pneumatics.matchloader_v(1);
 pneumatics.intakepiston_v(1);
 scoring_mech.intake_move(600);
 chassis.turn_to_point(24.9,-24,1,8,2.5,30,700);
 scoring_mech.intake_move(600);
 chassis.drive_to_point(24,-16,5,0,3,50,900);


 chassis.drive_to_point(24.2,4,6,4,5.5,30,800);


 pros::Task intae(Scoring_Mech::intake_autontask);
 chassis.drive_to_point(24.2,25,6,0.5,3,30,1000);
 chassis.drive_distance(-13, 12,5,30,130,0.75, 0.0000, 3.75, 2);
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
  chassis.set_coordinates(-13.2, 1.92, 25.5);
  scoring_mech.intake_move(600);
  pros::Task awp_task(Pneumatics::awp_task5);
  chassis.drive_to_point(4,27,6,0,2,50,850);
  
  pros::delay(50);
  chassis.turn_to_point(25,2,1,8,2.5,30,600);
  pneumatics.matchloader_v(0);


  
  chassis.drive_to_point(25,2,6,0,2,30,770);
  scoring_mech.intake_move(0);
  pneumatics.matchloader_v(1);
  chassis.turn_to_point(25.2,-18,1,8,2.5,30,600);
  scoring_mech.intake_move(600);
  chassis.drive_to_point(25.2,-17,6,1,3,30,900);
  pros::delay(150);
  chassis.drive_to_point(25,6,5.5,6,6,30,650);

  pros::Task intae(Scoring_Mech::intake_autontask);
  chassis.drive_to_point(25,25,5,0.5,3,30,800);
  chassis.drive_distance(-13.5, 12,5,30,780,0.75, 0.0000, 3.75, 2);
  pros::delay(150);
  pneumatics.matchloader_v(0);
  chassis.set_coordinates(23.6, 19.6, chassis.get_absolute_heading()+1.5);

  chassis.turn_to_point(34.25,10,1,8,5,50,400);
  chassis.drive_to_point(34.25,10,7,1,8,30,600);
  chassis.turn_to_point(46,-96,1,8,5,50,400);
  chassis.drive_to_point(14.75,35,10.5,0,1,100,800);
  //pros::delay(500);
  chassis.turn_to_point(-25,-60,1,8,5,30,1500);

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
  chassis.set_coordinates(0, 0, 25.4);
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
  pros::millis();
  // set constants
  chassis.set_turn_exit_conditions(1.5, 0, 800);
  chassis.set_swing_exit_conditions(1, 20, 1750);
  chassis.set_drive_exit_conditions(1.5, 0, 1000);
  skills_constants();
  chassis.set_drive_constants(12, 0.65, 0.0001, 4.48, 2.5);
  chassis.set_heading_constants(12, 0.28, 0.000, 1.95, 4); // same as turn
  chassis.set_turn_constants(12, 0.28, 0.000, 1.95, 4);
  chassis.set_swing_constants(12, 0.56, 0.00001, 3.65, 5);
  chassis.heading_max_voltage = 2;
  chassis.turn_max_voltage = 10;
  chassis.drive_max_voltage = 10;
  
  
  // set coordinates, start intake
  chassis.set_coordinates(-11.81, -46.75, 337);
  scoring_mech.intake_move(600);

  // drive to 4 balls
  chassis.drive_distance(27.75, 337, 10, 0.75);
  chassis.set_turn_exit_conditions(1.5, 0, 850);
  chassis.turn_to_angle(220, 5);
  
  // drive back and score balls
  pneumatics.intakepiston_v(0);
  chassis.set_drive_exit_conditions(1.5, 0, 800);
  chassis.drive_distance(-15.5, 222, 10, 0.75);
  chassis.set_drive_exit_conditions(1.5, 0, 1000);
  
  // pros::delay(100);
  scoring_mech.mid_goal_score(500, 150);

  pros::delay(700); // wait for balls to score
  chassis.turn_to_point(-45.6, -45, 1, 10, 2.5, 0, 300); // was 400
  pneumatics.hood_v(0);
  chassis.drive_to_point(-45.6, -45, 8, 0.5, 2.5, 0, 1500); // was 1800
  
  // turn to loader
  chassis.turn_to_point(-46.75, -61, 1, 8, 1.5, 0, 300); // was 450
  pneumatics.matchloader_v(1);
  pros::delay(175); // was 250
  scoring_mech.intake_move(600);
  pneumatics.intakepiston_v(1);
  chassis.drive_to_point(-46.75, -62, 10, 0.5, 1.5, 0, 1950); // delay is to drive + pick up the blocks
  chassis.drive_distance(3, 10, 1, 0, 300, 0.65, 0.0001, 4.48, 2.5);


  double timeSecs = pros::millis()/ 1000.0;
  printf("\n run time %.2f sec\n",timeSecs);

  chassis.drive_to_point(-47, -51, 10, 0.5, 2.5, 0, 600);
  pneumatics.matchloader_v(0);
  chassis.set_turn_exit_conditions(1.5, 0, 650);
  chassis.turn_to_angle(145);
  chassis.drive_distance(-26, 10, 2.5, 0, 900, 0.65, 0.0001, 4.48, 2.5);
  scoring_mech.intake_move(0);
  

  chassis.turn_to_angle(185);
  chassis.drive_to_point(-58.25, 17, 9, 1, 1.5, 0, 1400); // drive down alley works dont touch
  chassis.set_swing_exit_conditions(1, 25, 1400);
  chassis.right_swing_to_angle(0, 1.65); // was 1.65

  // start scoring
  scoring_mech.intake_move(600);
  pneumatics.hood_v(1);
  chassis.left_swing_to_angle(0, 1.65);
  chassis.drive_distance(-5, 10, 2.5, 0, 1300, 0.65, 0.0001, 4.48, 2.5); //default timeout is already 1000 - for driving while scoring // was 1500

  // set coordinates & reset conditions
  chassis.set_coordinates(-47.5, 28.5, chassis.get_absolute_heading());
  
  // done scoring

  // drive into loader
  pneumatics.matchloader_v(1);
  chassis.drive_distance(5, 8, 2.5, 1, 500, 0.65, 0.0001, 4.48, 2.5);// was 900
  chassis.turn_to_point(-47.5, 72, 1, 10, 1.5, 0, 350);
  pneumatics.hood_v(0); 
  chassis.drive_to_point(-48.5, 61.5, 4, 2.5, 1.5, 0, 2400);
  chassis.drive_distance(3, 10, 1, 0, 300, 0.65, 0.0001, 4.48, 2.5);
  
  pros::delay(100);
  
  chassis.drive_distance(-10, 10, 2.5, 1, 550, 0.65, 0.0001, 4.48, 2.5);
  // align and drive into goal 
  chassis.turn_to_angle(0, 10, 1.5, 2, 330, 0.28, 0.000, 1.95, 4);
  chassis.drive_to_point(-47.5, 25, 8, 2, 1.5, 20, 700);

  // score on goal 
  scoring_mech.top_goal_intake(600);
  pneumatics.hood_v(1);
  chassis.drive_distance(-5, 10, 1.5, 0, 1700, 0.65, 0.0001, 4.48, 2.5);
  chassis.set_coordinates(-71+chassis.distance_from_nearest_object_h(), 71-chassis.distance_from_nearest_object_v(), chassis.get_absolute_heading());
  pneumatics.matchloader_v(0);
  timeSecs = pros::millis()/ 1000.0;
  printf("\n run time 2: %.2f sec\n",timeSecs);

  // drive out of goal
  chassis.drive_distance(6, 10, 1, 0, 900, 0.65, 0.0001, 4.48, 2.5);
  pneumatics.hood_v(0);
  chassis.drive_distance(-8, 7);
  pros::delay(150);
  chassis.drive_distance(7, 10);

  // turn to left of park zone
  chassis.turn_to_angle(35);
  chassis.drive_until(11.5, 8);
  // clear balls from intake
  scoring_mech.intake_move(600);
  // turn to park zone 
  chassis.set_swing_exit_conditions(1, 25, 500);
  chassis.left_swing_to_angle(74, 4);
  chassis.set_swing_exit_conditions(1, 25, 900);

  chassis.drive_distance(10, 10, 2.5, 1, 675, 0.65, 0.0001, 4.48, 2.5); // drive towards park zone
  // matchloader down & clear balls from park zone
  pneumatics.matchloader_v(1);

  chassis.set_turn_exit_conditions(2, 75, 950);
  chassis.left_swing_to_angle(80, 4);
  chassis.drive_distance(12, 12, 2, 0.5, 200, 0.65, 0.0001, 4.48, 2.5); // drive over the barrier
  pneumatics.matchloader_v(0);

  chassis.drive_distance(30, 8, 2, 0.5, 1200, 0.65, 0.0001, 4.48, 2.5); // drive through the park zone
  chassis.set_swing_exit_conditions(1, 20, 1750);
  
  chassis.turn_to_angle(87); // at this point we crossed the park zone

  // intake all balls from park zone
  chassis.drive_distance(25, 5.5, 2, 0.5, 2500,0.65, 0.0001, 4.48, 2.5);
  chassis.drive_distance(17, 7.5, 2, 0.5, 2500, 0.65, 0.0001, 4.48, 2.5);

  // grab corner ball
  //chassis.right_swing_to_angle(45,3);
  // scoring_mech.intake_move(600);

  // drive towards
  //chassis.drive_distance(5.5, 5, 2, 0.5, 400, 0.65, 0.0001, 4.48, 2.5);

  chassis.right_swing_to_angle(180, 1.5);
  chassis.drive_distance(-10, 4, 2, 0.5, 600, 0.65, 0.0001, 4.48, 2.5);
  chassis.set_coordinates(chassis.get_X_position(), 62.5, chassis.get_absolute_heading());
  
  timeSecs = pros::millis()/ 1000.0;
  printf("\n run time 3: %.2f sec\n",timeSecs);

  chassis.set_swing_exit_conditions(1, 75, 1750);
  //chassis.set_coordinates(chassis.get_X_position(), 63.5, 180);
    
  // drive forward & reset position
  chassis.drive_distance(6, 5, 2, 0.5, 550, 0.65, 0.0001, 4.48, 2.5);
  //pneumatics.mid_descore_v(1);
  chassis.set_coordinates(71- chassis.distance_from_nearest_object_h(),chassis.get_Y_position(), chassis.get_absolute_heading());
  
  pros::delay(100);
  
  // turn to 4 balls and drive into them
  scoring_mech.intake_move(600);
  chassis.set_turn_exit_conditions(1.5, 0, 400);
  chassis.turn_to_point(21, 20);
  chassis.set_turn_exit_conditions(1.5, 0, 800);
 
  
  chassis.drive_to_point(21, 18, 8, 1.5, 1.5, 20, 1200); // drive towards the stack of 4 // y = 20 later

  chassis.turn_to_angle(42, 6);

  pneumatics.matchloader_v(1);
  // drive into mid goal
  
  pneumatics.intakepiston_v(0);
  chassis.drive_distance(-10.5, 5, 2, 0.5, 550, 0.65, 0.0001, 4.48, 2.5);
  chassis.drive_distance(1, 5, 0.5, 0.5, 400, 0.65, 0.0001, 4.48, 2.5);

  scoring_mech.mid_goal_score(-400, -400);
  pros::delay(50);
  scoring_mech.mid_goal_score(500, 125);
  pros::delay(2000);
  scoring_mech.mid_goal_score(500, 75);
  pros::delay(250);
  scoring_mech.mid_goal_score(0,0);
  
  // done scoring
  timeSecs = pros::millis()/ 1000.0;
  printf("\n run time 4: %.2f sec\n",timeSecs);

  // done with scoring on middle

  // driving to matchloader
  chassis.turn_to_point(48, 48);
  
  chassis.drive_to_point(48, 48, 6, 1.5, 1.5, 20, 1800);
  pneumatics.hood_v(0);
  chassis.turn_to_point(48, 61);
  pneumatics.intakepiston_v(1);
  
  scoring_mech.intake_move(600);
  chassis.drive_to_point(48, 62, 8, 1.5, 1.5, 20, 1950);
  chassis.drive_distance(3, 10, 1, 0, 300, 0.65, 0.0001, 4.48, 2.5);

  // done with matchloading

  chassis.drive_distance(-8, 10, 8, 75, 600, 0.65, 0.0001, 4.48, 2.5); 
  //pros::delay(200);

  // turn & drive to alley way
  chassis.set_turn_exit_conditions(1.5, 0, 450);
  chassis.turn_to_angle(325);
  chassis.set_turn_exit_conditions(1.5, 0, 900);
  chassis.drive_distance(-25, 10);
  scoring_mech.intake_move(0);
  pneumatics.matchloader_v(0);
  // align to & drive down alley #2
  chassis.turn_to_angle(0);

  chassis.drive_to_point(61, -15.5, 8, 1.5, 1.5, 20, 1600);
  chassis.set_swing_exit_conditions(1, 10, 1800);
  chassis.set_turn_exit_conditions(1.5, 0, 200);
  chassis.turn_to_angle(5, 10);
  chassis.set_turn_exit_conditions(1.5, 0, 150);
  //chassis.turn_to_angle(3, 10);
  chassis.set_turn_exit_conditions(1.5, 0, 800);
  chassis.right_swing_to_angle(180, 1.65);

  scoring_mech.intake_move(600);
  pneumatics.hood_v(1);
  chassis.set_swing_exit_conditions(1, 10, 1200);
  chassis.drive_distance(-5, 10, 2, 75, 450, 0.65, 0.0001, 4.48, 2.5); 
  chassis.set_swing_exit_conditions(1, 10, 250);
  chassis.left_swing_to_angle(180, 3);
  chassis.set_swing_exit_conditions(1, 10, 1200);
  pneumatics.matchloader_v(1);
  pros::delay(700);
 
  chassis.set_coordinates(71-chassis.distance_from_nearest_object_h(), -71+chassis.distance_from_nearest_object_v(), chassis.get_absolute_heading());

  // reset position & conditions

  
  chassis.drive_distance(5, 10, 2, 0, 200, 0.65, 0.0001, 4.48, 2.5); 
  pneumatics.hood_v(0);
  chassis.set_turn_exit_conditions(1.5, 0, 300);
  chassis.turn_to_point(50.5,-72);
  chassis.set_turn_exit_conditions(1.5, 0, 800);
  chassis.drive_to_point(50.5, -68, 4.75, 1.5, 1.5, 75, 2100);  // match loading
  chassis.drive_distance(3, 10, 1, 0, 300, 0.65, 0.0001, 4.48, 2.5);
  
  chassis.drive_distance(-1.5, 10, 0, 75, 450, 0.65, 0.0001, 4.48, 2.5);
  chassis.set_turn_exit_conditions(1.5, 0, 400);
  chassis.turn_to_angle(183); 
  chassis.set_turn_exit_conditions(1.5, 0, 800);
  chassis.drive_to_point(47.75, -27, 5.5, 0, 1.5, 75, 1000);

  
  chassis.set_coordinates(71-chassis.distance_from_nearest_object_h(), -71+chassis.distance_from_nearest_object_v(), chassis.get_absolute_heading());
  pneumatics.hood_v(1);
  pros::delay(1500); // second scoring of the second long goal
  // done scoring
  //timeSecs = pros::millis()/ 1000.0;
  //printf("\n run time 5: %.2f sec\n",timeSecs);
  chassis.drive_distance(6, 10);
  pneumatics.hood_v(0);
  chassis.drive_distance(-8, 7);
  pros::delay(150);
  chassis.drive_distance(7, 10);

pneumatics.matchloader_v(0);
  // turn to left of park zone
  //chassis.turn_to_angle(183);
  chassis.turn_to_angle(215);
  chassis.drive_until(11.5, 8);
  
  // clear balls from intake
  scoring_mech.intake_move(600);
  // turn to park zone 
  chassis.set_swing_exit_conditions(1, 25, 500);
  chassis.left_swing_to_angle(254, 4);
  chassis.set_swing_exit_conditions(1, 25, 900);

  chassis.drive_distance(10, 10, 2.5, 1, 675, 0.65, 0.0001, 4.48, 2.5); // drive towards park zone
  // matchloader down & clear balls from park zone
  pneumatics.matchloader_v(1);

  chassis.set_turn_exit_conditions(2, 75, 950);
  chassis.left_swing_to_angle(260, 4);
  chassis.drive_distance(12, 12, 2, 0.5, 200, 0.65, 0.0001, 4.48, 2.5); // drive over the barrier
  // pneumatics.matchloader_v(0);

  chassis.drive_distance(30, 8, 2, 0.5, 1200, 0.65, 0.0001, 4.48, 2.5); // drive through the park zone
  pneumatics.matchloader_v(0);
  chassis.drive_until(43, 8);
  std::cout << "end distance: " << chassis.distance_from_nearest_object_v() << std::endl;
  chassis.set_swing_exit_conditions(1, 20, 1750);
  
  chassis.turn_to_angle(267); // at this point we crossed the park zone

 
  /*
  // drive towards second park zone
  chassis.drive_distance(5, 10, 2, 0, 450, 0.65, 0.0001, 4.48, 2.5); 
  chassis.set_turn_exit_conditions(1, 0, 600);
  chassis.turn_to_angle(215);
  pneumatics.matchloader_v(0);
  // drive to left of park zone
  chassis.drive_until(10.75, 10);
  pneumatics.hood_v(1); // open the hood so the blocks can come out
  scoring_mech.intake_move(600);
  chassis.set_swing_exit_conditions(1, 0, 800);
  chassis.left_swing_to_angle(260, 3);

  chassis.drive_distance(7, 3, 2, 0, 850, 0.65, 0.0001, 4.48, 2.5);
  chassis.drive_distance(-3.75, 10, 0.5, 0, 300, 0.65, 0.0001, 4.48, 2.5);
  pneumatics.matchloader_v(1);
  chassis.drive_distance(10, 12, 2, 0, 800, 0.65, 0.0001, 4.48, 2.5);
  //pneumatics.matchloader_v(0);
  chassis.drive_distance(36, 12, 2, 0, 500, 0.65, 0.0001, 4.48, 2.5);
  chassis.turn_to_angle(290);
  */
  //timeSecs = pros::millis()/ 1000.0;
  //printf("\n run time 6: %.2f sec\n",timeSecs);
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