#include "main.h"


Pneumatics::Pneumatics(pros::adi::DigitalOut aligner_port, pros::adi::DigitalOut doinker_left_port, pros::adi::DigitalOut doinker_right_port)
   : aligner(aligner_port),
   doinker_left(doinker_left_port),
   doinker_right(doinker_right_port) {}


void Pneumatics::aligner_control() {
  if (master.get_digital(DIGITAL_Y)){
       aligner_down = !aligner_down;
       aligner.set_value(aligner_down);
       while (master.get_digital(DIGITAL_Y)) {
           pros::delay(util::DELAY_TIME);
       }
   }
}


void Pneumatics::aligner_initialize() {
   aligner.set_value(0);
}


void Pneumatics::aligner_v(int value) {
   aligner.set_value(value);
}

int Pneumatics::aligner_task() {
   while (true) {
      pneumatics.aligner_control();
      pros::delay(10);
   } return 1;
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