#include "main.h"

class Pneumatics{
   public:
   pros::adi::DigitalOut aligner;
   //pros::adi::DigitalOut climb;
   pros::adi::DigitalOut doinker_left;
   pros::adi::DigitalOut doinker_right;
   Pneumatics(pros::adi::DigitalOut aligner_port, pros::adi::DigitalOut doinker_left_port, pros::adi::DigitalOut doinker_right_port);

   void aligner_initialize();
   void aligner_control();
   void aligner_v(int value);

   void doinker_initialize();
   void doinker_left_control();
   void doinker_right_control();
   void doinker_left_v(int value);
   void doinker_right_v(int value);

   static int aligner_task();
   static int doinker_left_task();
   static int doinker_right_task();
   
};