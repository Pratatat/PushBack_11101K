#include "main.h"
#include "algorithm"
using namespace std;

Scoring_Mech::Scoring_Mech(int8_t intake_mtr_grp, int8_t intake_mtr_grp2,int8_t intake_color_sensor_grp) 
    : bottom_intake(intake_mtr_grp)
    , top_intake(intake_mtr_grp2)
    , color_sensor(intake_color_sensor_grp) {}


void Scoring_Mech::initialize() {
    bottom_intake.set_encoder_units(pros::v5::MotorUnits::counts);
    bottom_intake.set_brake_mode(MOTOR_BRAKE_COAST);
    top_intake.set_encoder_units(pros::v5::MotorUnits::counts);
    top_intake.set_brake_mode(MOTOR_BRAKE_COAST);
    color_sensor.set_integration_time(5);
    color_sensor.set_led_pwm(100);
}
/*
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
        }
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
        }
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
*/


void Scoring_Mech::intake_control() {
    //printf("bottom intake temp %f\n", bottom_intake.get_temperature());
    //printf("Top intake temp %f\n", top_intake.get_temperature());
    
    if (master.get_digital(DIGITAL_L1) && (current_outtaking == 0)){
        // hood up lift up
        pneumatics.intakepiston_v(1);
        pros::delay(75);
        pneumatics.hood_v(1);
        bottom_intake.move_velocity(600);
        top_intake.move_velocity(600);

    } else if ((master.get_digital(DIGITAL_L2)) && (current_outtaking == 0)) {
        // hood up lift down
        pneumatics.intakepiston_v(0);
        pros::delay(75);
        pneumatics.hood_v(1);
        bottom_intake.move_velocity(600);
        top_intake.move_velocity(600);
        

    } else if ((master.get_digital(DIGITAL_R1)) && (current_outtaking == 0)) {
        // hood down lift up
        pneumatics.hood_v(0);
        pneumatics.intakepiston_v(1);
        bottom_intake.move_velocity(600);
        top_intake.move_velocity(600);
        

    } else if ((master.get_digital(DIGITAL_R2)) && (current_outtaking == 0)) {
        //hood down
        pneumatics.hood_v(0);
        bottom_intake.move_velocity(-600);
        top_intake.move_velocity(-600);

    } else if (current_outtaking == 0){
        bottom_intake.move_velocity(0);
        top_intake.move_velocity(0);
    }
}

int Scoring_Mech::intake_task() {
    while (true) {
        scoring_mech.intake_control();
        pros::delay(10);
    }
    return 1;
}

int Scoring_Mech::intake_autontask() {
    pros::delay(350);
    scoring_mech.top_goal_intake(600);
        
    return 1;
}
int Scoring_Mech::intake_autontask4() {
    pros::delay(100);
    scoring_mech.intake_move(600);
        
    return 1;
}
int Scoring_Mech::intake_autontask2() {
    pros::delay(700);
    scoring_mech.top_goal_intake(600);
        
    return 1;
}
int Scoring_Mech::intake_autontask3() {
    pros::delay(250);
    scoring_mech.top_goal_intake(600);
        
    return 1;
}
int Scoring_Mech::anti_jam_auton() {
    pros::delay(500);
    scoring_mech.top_goal_intake(-600);
    pros::delay(100);
    scoring_mech.intake_move(600);
    return 1;
}

void Scoring_Mech::top_goal_intake(double velocity) {
    pneumatics.intakepiston_v(1);
    pneumatics.hood_v(1);
    bottom_intake.move_velocity(velocity);
    top_intake.move_velocity(velocity);
}
void Scoring_Mech::intake_move(double velocity) {
    pneumatics.hood_v(0);
    bottom_intake.move_velocity(velocity);
    top_intake.move_velocity(velocity);
}
void Scoring_Mech::bottom_intake_move(double velocity) {
    pneumatics.hood_v(0);
    bottom_intake.move_velocity(velocity);
}
void Scoring_Mech::mid_intake_move(double velocity) {
    pneumatics.intakepiston_v(0);
    pneumatics.hood_v(1);
    bottom_intake.move_velocity(velocity);
    top_intake.move_velocity(velocity);
}
void Scoring_Mech::mid_intake_move_skills(double velocityBottom, double velocityTop) {
    bottom_intake.move_velocity(velocityBottom);
    top_intake.move_velocity(velocityTop);
}
void Scoring_Mech::top_intake_move(double velocity) {
    bottom_intake.move_velocity(0);
    top_intake.move_velocity(-velocity);
}

// use intake rotations
void Scoring_Mech::red_color_sort() { 
    color_sensor.set_led_pwm(100); 
    pros::delay(250);
    color_sensor.set_integration_time(5);
    int current_rotation = 0;
    while (!driverControl) {
        if ((color_sensor.get_hue() <= 240 and color_sensor.get_hue() >= 200) && (color_sensor.get_saturation() <= 0.9 and color_sensor.get_saturation() >= 0.5) && color_sensor.get_proximity() >= 250) {
            current_rotation = bottom_intake.get_position();
            while (bottom_intake.get_position() - current_rotation < 525) {
                pros::delay(5);
                continue;
            } 
            current_outtaking = 1;
            bottom_intake.move_velocity(0);
            pros::delay(100);
            bottom_intake.move_velocity(600);
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
            current_rotation = bottom_intake.get_position();
            while (bottom_intake.get_position() - current_rotation < 525) {
                pros::delay(5);
                continue;
            } 
            current_outtaking = 1;
            bottom_intake.move_velocity(0);
            pros::delay(100);
            bottom_intake.move_velocity(600);
            current_outtaking = 0;
        } 
        pros::delay(5);
    }
}

int Scoring_Mech::blue_color_sort_task() {
    scoring_mech.blue_color_sort();
    return 1;
}
static constexpr int CMD_REV     = -200;  // reverse speed for unjam
static constexpr int REV_MS      = 200;   // reverse duration
static constexpr int PAUSE_MS    = 100;   // pause after reverse
static constexpr int COOLDOWN_MS = 300;   // ignore new jams briefly
static constexpr int LOOP_MS     = 20;    // loop period

static constexpr double TORQUE_JAM   = 0.6;   // high torque threshold
static constexpr double VEL_LOW      = 75.0;  // "stalled" actual velocity
static constexpr double VEL_RECOVER  = 150.0; // "recovered" velocity
static constexpr int JAM_DEBOUNCE_MS = 150;   // must be jammy this long
static constexpr int RECOVER_MS      = 150;   // must be healthy this long

void Scoring_Mech::intake_detector() {
  enum State { RUN, UNJAM_REV, UNJAM_PAUSE, COOLDOWN } state = RUN;

  static constexpr int CMD_FWD     = 600;   

  int jam_ms = 0;
  int healthy_ms = 0;
  int state_ms = 0;

  int commanded = CMD_FWD;

  while (true) {
    double actual = bottom_intake.get_actual_velocity();
    double torque = bottom_intake.get_torque();

    switch (state) {
      case RUN: {
        // Command steady forward
        if (commanded != CMD_FWD) {
          commanded = CMD_FWD;
          bottom_intake.move_velocity(CMD_FWD);
          top_intake.move_velocity(CMD_FWD);
        }

        // JAM condition: high torque + LOW actual velocity while we are commanding forward
        bool jammy = (commanded > 0) && (torque > TORQUE_JAM) && (actual < VEL_LOW);

        if (jammy) {
          jam_ms += LOOP_MS;
          healthy_ms = 0;
        } else {
          jam_ms = 0;
          // Track healthy window for re-arm hysteresis
          if ((torque < TORQUE_JAM) && (actual > VEL_RECOVER)) {
            healthy_ms += LOOP_MS;
          } else {
            healthy_ms = 0;
          }
        }

        if (jam_ms >= JAM_DEBOUNCE_MS) {
          // Trigger anti-jam
          bottom_intake.move_velocity(CMD_REV);
          top_intake.move_velocity(0);
          state = UNJAM_REV;
          state_ms = 0;
          jam_ms = 0;
        }
        break;
      }

      case UNJAM_REV: {
        state_ms += LOOP_MS;
        if (state_ms >= REV_MS) {
          bottom_intake.move_velocity(0);
          top_intake.move_velocity(0);
          state = UNJAM_PAUSE;
          state_ms = 0;
        }
        break;
      }

      case UNJAM_PAUSE: {
        state_ms += LOOP_MS;
        if (state_ms >= PAUSE_MS) {
          bottom_intake.move_velocity(CMD_FWD);
          top_intake.move_velocity(CMD_FWD);
          state = COOLDOWN;
          state_ms = 0;
        }
        break;
      }

      case COOLDOWN: {
        // ignore jam signals briefly to prevent chatter
        state_ms += LOOP_MS;
        if (state_ms >= COOLDOWN_MS) {
          state = RUN;
          state_ms = 0;
          healthy_ms = 0;
        }
        break;
      }
    }

    // simple logging (optional)
    // printf("[state %d] torque=%.2f actual=%.1f cmd=%d\n", state, torque, actual, commanded);

    pros::delay(LOOP_MS);
  }
}

int Scoring_Mech::intake_detector_task() {
    scoring_mech.intake_detector();
    return 1;
}
