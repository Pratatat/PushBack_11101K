#include "main.h"
class Pneumatics{
   public:
   pros::adi::DigitalOut matchloader;
   pros::adi::DigitalOut descore;
   pros::adi::DigitalOut intake_piston;
   pros::adi::DigitalOut wing;
   Pneumatics(pros::adi::DigitalOut matchloader_port, pros::adi::DigitalOut intakepiston_port, pros::adi::DigitalOut wing_port,pros::adi::DigitalOut descore_port);

   void matchloader_initialize();
   void matchloader_control();
   void matchloader_v(int value);
   static int matchloader_task();
   
   void intakepiston_initialize();
   void intakepiston_control();
   void intakepiston_v(int value);
   static int intakepiston_task();

   void wing_initialize();
   void wing_control();
   void wing_v(int value);
   static int wing_task();

   void descore_initialize();
   void descore_control();
   void descore_v(int value);
   static int descore_task();
   
   

   static int awp_task();
   static int awp_task2();
   static int awp_task3();
   static int awp_task4();
   static int awp_task5();
   static int awp_task6();
   static int awp_task7();
    static int awp_task8();
};