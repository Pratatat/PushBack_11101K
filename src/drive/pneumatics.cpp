#include "main.h"


Pneumatics::Pneumatics(pros::adi::DigitalOut matchloader_port, pros::adi::DigitalOut intakepiston_port, pros::adi::DigitalOut doinker_right_port)
   : matchloader(matchloader_port),
   intake_piston(intakepiston_port),
   doinker_right(doinker_right_port) {}


void Pneumatics::matchloader_control() {
  if (master.get_digital(DIGITAL_Y)){
       matchloader_down = !matchloader_down;
       matchloader.set_value(matchloader_down);
       while (master.get_digital(DIGITAL_Y)) {
           pros::delay(util::DELAY_TIME);
       }
   }
}


void Pneumatics::matchloader_initialize() {
   matchloader.set_value(0);
}


void Pneumatics::matchloader_v(int value) {
   matchloader.set_value(value);
}

int Pneumatics::matchloader_task() {
   while (true) {
      pneumatics.matchloader_control();
      pros::delay(10);
   } return 1;
}

int Pneumatics::awp_task() {
   pros::delay(600);
   pneumatics.matchloader_v(1);
   return 1;
}
int Pneumatics::awp_task2() {
   pros::delay(525);
   pneumatics.matchloader_v(1);
   return 1;
}
int Pneumatics::awp_task3() {
   pros::delay(750);
   pneumatics.matchloader_v(1);
   return 1;
}


void Pneumatics::intakepiston_initialize() {
   intake_piston.set_value(0);
}


void Pneumatics::intakepiston_v(int value) {
   intake_piston.set_value(value);
}


int Pneumatics::intakepiston_task() {
   while (true) {
      pneumatics.intakepiston_control();
      pros::delay(10);
   } return 1;
}

void Pneumatics::intakepiston_control() {
  if (master.get_digital(DIGITAL_B)){
       intakepiston_down = !intakepiston_down;
       intake_piston.set_value(intakepiston_down);
       while (master.get_digital(DIGITAL_B)) {
           pros::delay(util::DELAY_TIME);
       }
   }
}
