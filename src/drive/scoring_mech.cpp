#include "main.h"
#include "algorithm"
using namespace std;

Scoring_Mech::Scoring_Mech(std::initializer_list<std::int8_t> neutral_stake_mtr_grp, int8_t neutral_stake_rot_grp, int8_t intake_mtr_grp, int8_t intake_color_sensor_grp) 
    : neutral_stake_mtr(neutral_stake_mtr_grp)
    , neutral_stake_rot(neutral_stake_rot_grp) 
    , intake_mtr(intake_mtr_grp)
    , color_sensor(intake_color_sensor_grp) {}


void Scoring_Mech::initialize() {
    //neutral_stake_rot.set_position(36000);
    neutral_stake_rot.set_position(33000);



    neutral_stake_mtr.move_velocity(0);
    neutral_stake_rot.set_data_rate(5);
    intake_mtr.set_encoder_units(pros::v5::MotorUnits::counts);
    intake_mtr.set_brake_mode(MOTOR_BRAKE_HOLD);
    color_sensor.set_integration_time(5);
    color_sensor.set_led_pwm(100);
    set_brake_mode('H');
}

void Scoring_Mech::neutral_stake_control() {
    if (master.get_digital(DIGITAL_A) && neutral_stake_position != 3) {
        neutral_stake_mtr.move_velocity(600);
        timeout = 0;
        while (neutral_stake_rot.get_position() > angle_positions[neutral_stake_position + 1] + up_thresholds[neutral_stake_position] and timeout < 3000) {
            pros::delay(5);
            timeout += 5;

        }
        /*neutral_stake_mtr.move_velocity(100);
        while (neutral_stake_rot.get_position() > angle_positions[neutral_stake_position + 1] + up_thresholds[neutral_stake_position] and timeout < 3000) {
            pros::delay(5);
            timeout += 5;
        }*/
        neutral_stake_position++;
        neutral_stake_mtr.move_velocity(0);
    } 
    else if (master.get_digital(DIGITAL_Y) && neutral_stake_position != 0) {
        neutral_stake_mtr.move_velocity(-600);
        timeout = 0;
        while (neutral_stake_rot.get_position() < angle_positions[neutral_stake_position - 1] - down_thresholds[neutral_stake_position-1] and timeout < 3000) {
            pros::delay(5);
            timeout += 5;
        }
        /*neutral_stake_mtr.move_velocity(-100);
        while (neutral_stake_rot.get_position() < angle_positions[neutral_stake_position - 1] - down_thresholds[neutral_stake_position-1] and timeout < 3000) {
            pros::delay(5);
            timeout += 5;
        }*/
        neutral_stake_position--;
        neutral_stake_mtr.move_velocity(0);
    } 
    else if (master.get_digital(DIGITAL_B) && neutral_stake_position == 1) {
        neutral_stake_mtr.move_velocity(600);
        while (neutral_stake_rot.get_position() > 26000) {
            pros::delay(5);
        } 
        about_to_score = 1;
    } else if (master.get_digital(DIGITAL_DOWN)) {
        neutral_stake_mtr.move_velocity(-600);
    }
    else {
        neutral_stake_mtr.move_velocity(0);
    }

    if (master.get_digital(DIGITAL_UP)) {
        neutral_stake_mtr.move_velocity(0);
        pros::delay(500);
        neutral_stake_position = 0;
        neutral_stake_rot.set_position(36000);
        pros::delay(500);
    }
}

int Scoring_Mech::neutral_stake_task() {
    while (true) {
        scoring_mech.neutral_stake_control();
        pros::delay(5);
    }
    return 1;
}


void Scoring_Mech::move1(double voltage) {
    neutral_stake_mtr.move_velocity(voltage);
}


void Scoring_Mech::set_brake_mode(char brake_type) {
   if (brake_type == 'H'){
      neutral_stake_mtr.move_velocity(0);
      neutral_stake_mtr.set_brake_mode_all(MOTOR_BRAKE_HOLD);
   }
   else if (brake_type == 'C'){
      neutral_stake_mtr.move_velocity(0);
      neutral_stake_mtr.set_brake_mode_all(MOTOR_BRAKE_COAST);
   } else {
      neutral_stake_mtr.move_velocity(0);
      neutral_stake_mtr.set_brake_mode_all(MOTOR_BRAKE_BRAKE);
   }
}

void Scoring_Mech::neutral_stake_setup() {
    neutral_stake_mtr.move_velocity(600);
    while (neutral_stake_rot.get_position() > angle_positions[1] + up_thresholds[0]) {
        pros::delay(5);
    }
    neutral_stake_mtr.move_velocity(0);
}

int Scoring_Mech::neutral_stake_setup_task() {
    scoring_mech.neutral_stake_setup();
    return 1;
}

void Scoring_Mech::neutral_stake_score() {
    neutral_stake_mtr.move_velocity(600);
    // + 7250
    //+ 750
    while(neutral_stake_rot.get_position() > angle_positions[2] + up_thresholds[2] + 750) {
        pros::delay(5);
    }
    neutral_stake_mtr.move_velocity(0);
}

int Scoring_Mech::neutral_stake_score_task() {
    scoring_mech.neutral_stake_score();
    return 1;
}


void Scoring_Mech::intake_control() {
    if (master.get_digital(DIGITAL_L1) && (current_outtaking == 0)){
        intake_mtr.move_velocity(600);
    } else if ((master.get_digital(DIGITAL_L2)) && (current_outtaking == 0)) {
        intake_mtr.move_velocity(-600);
    } else if (current_outtaking == 0){
        intake_mtr.move_velocity(0);
    }
}

int Scoring_Mech::intake_task() {
    while (true) {
        scoring_mech.intake_control();
        pros::delay(10);
    }
    return 1;
}

void Scoring_Mech::intake_move(double velocity) {
    intake_mtr.move_velocity(velocity);
    current_intaking = velocity;
}


// use intake rotations
void Scoring_Mech::red_color_sort() { 
    color_sensor.set_led_pwm(100); 
    pros::delay(250);
    color_sensor.set_integration_time(5);
    int current_rotation = 0;
    while (!driverControl) {
        if ((color_sensor.get_hue() <= 240 and color_sensor.get_hue() >= 200) && (color_sensor.get_saturation() <= 0.9 and color_sensor.get_saturation() >= 0.5) && color_sensor.get_proximity() >= 250) {
            current_rotation = intake_mtr.get_position();
            while (intake_mtr.get_position() - current_rotation < 525) {
                pros::delay(5);
                continue;
            } 
            current_outtaking = 1;
            intake_mtr.move_velocity(0);
            pros::delay(100);
            intake_mtr.move_velocity(600);
            current_outtaking = 0; 
        } 
        pros::delay(5);
    }
}

int Scoring_Mech::red_color_sort_task() {
    scoring_mech.red_color_sort();
    return 1;
}
  
void Scoring_Mech::blue_color_sort() {
    color_sensor.set_led_pwm(100); 
    pros::delay(250);
    color_sensor.set_integration_time(5);
    int current_rotation = 0;
    while (!driverControl) {
        if ((color_sensor.get_hue() <= 10 or color_sensor.get_hue() >= 350) && (color_sensor.get_saturation() >= 0.6) && color_sensor.get_proximity() >= 250) {
            current_rotation = intake_mtr.get_position();
            while (intake_mtr.get_position() - current_rotation < 525) {
                pros::delay(5);
                continue;
            } 
            current_outtaking = 1;
            intake_mtr.move_velocity(0);
            pros::delay(100);
            intake_mtr.move_velocity(600);
            current_outtaking = 0;
        } 
        pros::delay(5);
    }
}

int Scoring_Mech::blue_color_sort_task() {
    scoring_mech.blue_color_sort();
    return 1;
}

void Scoring_Mech::intake_detector() {
    int timeout = 0;
    while (!driverControl) {
        intake_mtr.get_target_velocity();
        if (current_intaking > 300 && intake_mtr.get_actual_velocity() <= 25) {
            timeout += 10;
        }
        else {
            timeout = 0;
        }

        if (timeout > 250) {
            intake_mtr.move_velocity(-600);
            pros::delay(100);
            intake_mtr.move_velocity(600);
            pros::delay(250);
            timeout = 0;
        }
        pros::delay(10);
    }
}

int Scoring_Mech::intake_detector_task() {
    scoring_mech.intake_detector();
    return 1;
}

void Scoring_Mech::rush_helper() {
    scoring_mech.move1(75);
    pros::delay(100);
    scoring_mech.intake_move(600);
    pros::delay(375);
    scoring_mech.move1(0);
    pros::delay(210);
    //pneumatics.doinker_left_v(1);
    pneumatics.doinker_right_v(1);
    scoring_mech.intake_move(10);
}

int Scoring_Mech::rush_helper_task() {
    scoring_mech.rush_helper();
    return 1;
}
