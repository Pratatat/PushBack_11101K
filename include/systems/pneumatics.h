#include "main.h"
class Pneumatics{
   public:
   pros::adi::DigitalOut matchloader;
   //pros::adi::DigitalOut climb;
   pros::adi::DigitalOut intake_piston;
   pros::adi::DigitalOut wing;
   Pneumatics(pros::adi::DigitalOut matchloader_port, pros::adi::DigitalOut intakepiston_port, pros::adi::DigitalOut wing_port);

   void matchloader_initialize();
   void matchloader_control();
   void matchloader_v(int value);

   void intakepiston_initialize();
   void intakepiston_control();
   void intakepiston_v(int value);

   void wing_initialize();
   void wing_control();
   void wing_v(int value);

   static int matchloader_task();
   static int intakepiston_task();
   static int wing_task();
   static int awp_task();
   static int awp_task2();
   static int awp_task3();
   
};