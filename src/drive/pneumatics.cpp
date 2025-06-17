#include "main.h"


Pneumatics::Pneumatics(pros::adi::DigitalOut clench_port, pros::adi::DigitalOut doinker_left_port, pros::adi::DigitalOut doinker_right_port)
   : clench(clench_port),
   doinker_left(doinker_left_port),
   doinker_right(doinker_right_port) {}


void Pneumatics::clench_control() {
  if (master.get_digital(DIGITAL_R1)){
       clench_down = !clench_down;
       clench.set_value(clench_down);
       while (master.get_digital(DIGITAL_R1)) {
           pros::delay(util::DELAY_TIME);
       }
   }
}


void Pneumatics::clench_initialize() {
   clench.set_value(0);
}


void Pneumatics::clench_v(int value) {
   clench.set_value(value);
}

int Pneumatics::clench_task() {
   while (true) {
      pneumatics.clench_control();
      pros::delay(10);
   } return 1;
}

void Pneumatics::doinker_left_control() {
   if (master.get_digital(DIGITAL_R2)){
       doinker_left_open = !doinker_left_open;
       doinker_left.set_value(doinker_left_open);
       while (master.get_digital(DIGITAL_R2)) {
         pros::delay(util::DELAY_TIME);
       }
   }
}

void Pneumatics::doinker_right_control() {
   if (master.get_digital(DIGITAL_X)){
      doinker_right_open = !doinker_right_open;
      doinker_right.set_value(doinker_right_open);
      while (master.get_digital(DIGITAL_X)) {
        pros::delay(util::DELAY_TIME);
      }
  }
}


void Pneumatics::doinker_initialize() {
   doinker_left.set_value(0);
   doinker_right.set_value(0);
}


void Pneumatics::doinker_left_v(int value) {
   doinker_left.set_value(value);
}

void Pneumatics::doinker_right_v(int value) {
   doinker_right.set_value(value);
}

int Pneumatics::doinker_left_task() {
   while (true) {
      pneumatics.doinker_left_control();
      pros::delay(10);
   } return 1;
}

int Pneumatics::doinker_right_task() {
   while (true) {
      pneumatics.doinker_right_control();
      pros::delay(10);
   } return 1;
}