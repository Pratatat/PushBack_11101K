#include "main.h"

class Pneumatics{
   public:
   pros::adi::DigitalOut matchloader;
   //pros::adi::DigitalOut climb;
   pros::adi::DigitalOut intake_piston;
   pros::adi::DigitalOut doinker_right;
   Pneumatics(pros::adi::DigitalOut matchloader_port, pros::adi::DigitalOut intakepiston_port, pros::adi::DigitalOut doinker_right_port);

   void matchloader_initialize();
   void matchloader_control();
   void matchloader_v(int value);

   void intakepiston_initialize();
   void intakepiston_control();
   void intakepiston_v(int value);

   static int matchloader_task();
   static int intakepiston_task();
   static int doinker_right_task();
   static int awp_task();
   static int awp_task2();
   static int awp_task3();
   
};