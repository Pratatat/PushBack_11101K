#include "main.h"

class Drive;
class Scoring_Mech;
class Pneumatics;

extern Drive chassis;
extern Scoring_Mech scoring_mech;
extern Pneumatics pneumatics;

void default_constants();

void drive_test();
void turn_test();
void swing_test();
void full_test();
void odom_test();
void tank_odom_test();
void holonomic_odom_test();

void redLeftQual();
void redRightQual();
void redLeftElim();
void redRightElim();
void blueRightQual();
void blueLeftQual();
void blueRightElim();
void blueLeftElim();
void skills();
void auton_setup();
void test();