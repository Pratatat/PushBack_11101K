#include "main.h"

class Drive;
class Scoring_Mech;
class Pneumatics;

extern Drive chassis;
extern Scoring_Mech scoring_mech;
extern Pneumatics pneumatics;

void default_constants();

void drive_test();
void odom_offset_test();
void turn_test();
void swing_test();
void full_test();
void odom_test();
void tank_odom_test();
void holonomic_odom_test();
void intake_test();


void RightSevenElim();
void LeftSevenQual();
void RightFour();
void LeftFour();
void LeftSeven();
void RightSeven();
void LeftSevenElim();
void AWP();
void RightAWP();
void RightRush();
void LeftRush();

void skills();
void right_auton_setup();
void left_auton_setup();
void auton_setup();
void test();

void set_coordinates_with_ds();
void set_X_coordinate_with_ds();
void set_Y_coordinate_with_ds();