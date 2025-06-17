#include "main.h"

//Controller
pros::Controller master(pros::E_CONTROLLER_MASTER);

//Variables
int clench_down = 0;
int climb_open = 0;
int doinker_left_open = 0;
int doinker_right_open = 0;
double PI = 3.14159265359;

float reduce_0_to_360(float angle) {
  while(!(angle >= 0 && angle < 360)) {
    if( angle < 0 ) { angle += 360; }
    if(angle >= 360) { angle -= 360; }
  }
  return(angle);
}

float reduce_negative_180_to_180(float angle) {
  while(!(angle >= -180 && angle < 180)) {
    if( angle < -180 ) { angle += 360; }
    if(angle >= 180) { angle -= 360; }
  }
  return(angle);
}

float reduce_negative_90_to_90(float angle) {
  while(!(angle >= -90 && angle < 90)) {
    if( angle < -90 ) { angle += 180; }
    if(angle >= 90) { angle -= 180; }
  }
  return(angle);
}

float to_rad(float angle_deg){
  return(angle_deg/(180.0/PI));
}

float to_deg(float angle_rad){
  return(angle_rad*(180.0/PI));
}

float clamp(float input, float min, float max){
  if( input > max ){ return(max); }
  if(input < min){ return(min); }
  return(input);
}

float doinker(float input, float min, float max){
  if( input > max ){ return(max); }
  if(input < min){ return(min); }
  return(input);
}

bool is_reversed(double input){
  if(input<0) return(true);
  return(false);
}

float to_volt(float percent){
  return(percent*12.0/100.0);
}

float to_mili_volt(float percent){
  return(percent*120.0);
}

int to_port(int port){
  if(port>8){
    return(0);
  }
  return(port-1);
} 

float deadband(float input, float width){
  if (fabs(input)<width){
    return(0);
  }
  return(input);
}