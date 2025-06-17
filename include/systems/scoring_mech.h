#include "main.h"
#include <array>

class Scoring_Mech{
    public:
    //int neutral_stake_position = 0;
    //int neutral_stake_position = 1;
    int neutral_stake_position = 2;


    int about_to_score = 0;
    int current_outtaking = 0;
    pros::MotorGroup neutral_stake_mtr;
    pros::Rotation neutral_stake_rot;
    pros::Motor intake_mtr;
    pros::Optical color_sensor;
    Scoring_Mech(std::initializer_list<std::int8_t> neutral_stake_mtr_grp, int8_t neutral_stake_rot_grp, int8_t intake_mtr_grp, int8_t intake_color_sensor_grp);

    void initialize();

    void neutral_stake_control();
    static int neutral_stake_task();
    int angle_positions[4] = {36000, 33000, 19000, 13000};
    int up_thresholds[3] = {1000, 900, 750};
    int down_thresholds[3] = {500, 500, 1500};
    int timeout = 0;
    int current_intaking = 0;

    void move1(double voltage);
    void set_brake_mode(char brake_type);

    void intake_control();
    static int intake_task();
    void intake_move(double velocity);
    bool driverControl = false;

    static int rush_helper_task();
    void rush_helper();
    static int neutral_stake_score_task();
    void neutral_stake_score();
    static int neutral_stake_setup_task();
    void neutral_stake_setup();
    static int intake_detector_task();
    void intake_detector();
    static int red_color_sort_task();
    void red_color_sort();
    static int blue_color_sort_task();
    void blue_color_sort();
};