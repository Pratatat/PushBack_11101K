#include "main.h"
#include "pros/motors.h"
#define MOVING_VOLTAGE 25
float PII = 3.14159265359;
float GYRO_SCALE;
using namespace std;


Drive::Drive(enum::drive_setup_enum drive_setup, std::initializer_list<std::int8_t> DriveL, std::initializer_list<std::int8_t> DriveR, int gyro_port, int distance_port_v, int distance_port_h, float wheel_diameter, float wheel_ratio, float gyro_scale, int ForwardTracker_port, float ForwardTracker_diameter, float ForwardTracker_center_distance, int SidewaysTracker_port, float SidewaysTracker_diameter, float SidewaysTracker_center_distance) 
  : DriveL(DriveL),
  DriveR(DriveR),
  Gyro(gyro_port),
  distance_sensor_v(distance_port_v),
  distance_sensor_h(distance_port_h),
  in_to_deg(wheel_ratio/360.0*PII*wheel_diameter),
  ForwardTracker_center_distance(ForwardTracker_center_distance),
  ForwardTracker_diameter(ForwardTracker_diameter),
  ForwardTracker_in_to_deg_ratio(PII*ForwardTracker_diameter/360.0),
  SidewaysTracker_center_distance(SidewaysTracker_center_distance),
  SidewaysTracker_diameter(SidewaysTracker_diameter),
  SidewaysTracker_in_to_deg_ratio(PII*SidewaysTracker_diameter/360.0),
  drive_setup(drive_setup),
  R_ForwardTracker(ForwardTracker_port),
  R_SidewaysTracker(SidewaysTracker_port),
  E_ForwardTracker(NUM_V5_PORTS, ForwardTracker_port),
  E_SidewaysTracker(NUM_V5_PORTS, SidewaysTracker_port) 
  {
  // Set constants for tick_per_inch calculation
  float WHEEL_DIAMETER = wheel_diameter;
  float RATIO = wheel_ratio;
  GYRO_SCALE = gyro_scale;

  if (drive_setup != ZERO_TRACKER_NO_ODOM){
    if (drive_setup == TANK_ONE_ENCODER || drive_setup == TANK_ONE_ROTATION || drive_setup == ZERO_TRACKER_ODOM){
      odom.set_physical_distances(ForwardTracker_center_distance, 0);
      // Setting the sideways distance to 0 essentially tells the robot that there is a sideways tracker
      // in the center of the robot that never moves. Even though the tracker isn't really there, the odom
      // still works fine.
    } else {
      odom.set_physical_distances(ForwardTracker_center_distance, SidewaysTracker_center_distance);
    }
  }
}

bool Drive::imu_calibrate() {
  Gyro.reset();
  return true;

}



void Drive::reset_drive_sensor() {
   DriveL.tare_position();
   DriveR.tare_position();
}

void Drive::print_odom_vals() {
  DriveL.set_encoder_units(pros::E_MOTOR_ENCODER_DEGREES);
  float avg = ((DriveL.get_position() + DriveR.get_position())/2) * (10.21/480);
  printf("Encoder avg: %f\n",avg);
  printf("Odom value: %f\n", odom.Y_position);
  std::string encorder_avg, odom_value;
  encorder_avg = std::to_string(avg);
  odom_value = std::to_string(odom.Y_position);
  
  pros::screen::draw_rect(0,0,480,240);
    pros::screen::set_pen(pros::Color::white);
    //pros::screen::print(TEXT_LARGE, 50, 50, encorder_avg.c_str());
    pros::screen::print(TEXT_LARGE, 50, 100, odom_value.c_str());
}

void Drive::arcade_control() {
   int horz = master.get_analog(ANALOG_LEFT_X);
   int vert = master.get_analog(ANALOG_LEFT_Y);

   if (abs(horz) < deadzone){
       horz = 0;}
   if (abs(vert) < deadzone){
       vert = 0;}

   vert = vert * 100; 
   horz = horz * abs(horz)/127 * 120;

   DriveL.move_voltage(vert + horz);
   DriveR.move_voltage(vert - horz);
}

void Drive::arcade_control_double() {
   //Grabs joystick position
   int vert = master.get_analog(ANALOG_LEFT_Y);
   int horz = master.get_analog(ANALOG_RIGHT_X);
   //Checks to make sure joystick isn't it deadzone
   if (abs(horz) < deadzone){
       horz = 0;}
   if (abs(vert) < deadzone){
       vert = 0;}
   vert = vert * 100;
   horz = horz * 100;
   DriveL.move_voltage(vert + horz);
   DriveR.move_voltage(vert - horz);
}
void Drive::arcade_control_double_reversed() {
   //Grabs joystick position
   int vert = master.get_analog(ANALOG_RIGHT_Y);
   int horz = master.get_analog(ANALOG_LEFT_X);
   //Checks to make sure joystick isn't it deadzone
   if (abs(horz) < deadzone){
       horz = 0;}
   if (abs(vert) < deadzone){
       vert = 0;}
   vert = vert * 100;
   horz = horz * 100;
   DriveL.move_voltage(vert + horz);
   DriveR.move_voltage(vert - horz);
}

void Drive::tank_control() {
int straight = -master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
int turn = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X);

// Apply exponential scaling to the turn value
turn = (turn * turn * turn) / (127 * 127);
// Deadzone adjustment
if(abs(straight) < deadzone) {
    straight = 0;
}
if(abs(turn) < deadzone) {
    turn = 0;
}

int leftMotorVoltage = straight - turn;
int rightMotorVoltage = straight + turn;

// Limit motor voltage
leftMotorVoltage = std::clamp(leftMotorVoltage, -127, 127);
rightMotorVoltage = std::clamp(rightMotorVoltage, -127, 127);

// Adjust for minimum moving voltage
if(leftMotorVoltage > deadzone && leftMotorVoltage < MOVING_VOLTAGE) leftMotorVoltage = MOVING_VOLTAGE;
if(leftMotorVoltage < -deadzone && leftMotorVoltage > -MOVING_VOLTAGE) leftMotorVoltage = -MOVING_VOLTAGE;
if(rightMotorVoltage > deadzone && rightMotorVoltage < MOVING_VOLTAGE) rightMotorVoltage = MOVING_VOLTAGE;
if(rightMotorVoltage < -deadzone && rightMotorVoltage > -MOVING_VOLTAGE) rightMotorVoltage = -MOVING_VOLTAGE;

DriveL.move_voltage(leftMotorVoltage);
DriveR.move_voltage(rightMotorVoltage);

}

void Drive::initialize() {
 R_ForwardTracker.reset_position();
 R_ForwardTracker.set_data_rate(5);
 R_SidewaysTracker.reset_position();
 R_SidewaysTracker.set_data_rate(5);
 imu_calibrate();
 Gyro.set_data_rate(5);
 reset_drive_sensor();
 set_brake_mode('H');
}

bool Drive::check() {
  if (!R_ForwardTracker.is_installed() || !R_SidewaysTracker.is_installed()) {
    return false;
  }
  else {
    return true;
  }
}
void Drive::macro(){}

void Drive::set_brake_mode(char brake_type) {
   if (brake_type == 'H'){
      DriveL.set_brake_mode(MOTOR_BRAKE_HOLD);
      DriveR.set_brake_mode(MOTOR_BRAKE_HOLD);
   }
   else if (brake_type == 'C'){
      DriveL.set_brake_mode(MOTOR_BRAKE_COAST);
      DriveR.set_brake_mode(MOTOR_BRAKE_COAST);
   }
}

float Drive::get_gyro() {
   return Gyro.get_heading();
}

void Drive::reset_gyro() {
   Gyro.set_heading(0);
}

////////////// vertical distance sensor functions 

double Drive::distance_from_nearest_object_v(){
  return (distance_sensor_v.get()/25.4) + 5.15;
}

bool Drive::is_object_detected_v(){
  return distance_sensor_v.get_confidence() > 0;
}

double Drive::velocity_of_detected_object_v(){
  return distance_sensor_v.get_object_velocity();
}

/////////// horizontal distance sensor functions 

double Drive::distance_from_nearest_object_h(){
  return (distance_sensor_h.get()/25.4) + 5.225;
}

bool Drive::is_object_detected_h(){
  return distance_sensor_h.get_confidence() > 0;
}

double Drive::velocity_of_detected_object_h(){
  return distance_sensor_h.get_object_velocity();
}




void Drive::drive_with_voltage(float leftVoltage, float rightVoltage){
  //cout << chassis.get_absolute_heading() << " " << R_SidewaysTracker.get_position() << " " <<  R_ForwardTracker.get_position() << endl;
  if (fabs(leftVoltage) < 0 && fabs(rightVoltage) < 0) return;
  DriveL.move_voltage(leftVoltage*1000);
  DriveR.move_voltage(rightVoltage*1000);
  /*while (chassis.get_absolute_heading() > 270 || chassis.get_absolute_heading() < 10) {
    DriveL.move_voltage(leftVoltage * 1000);
    DriveR.move_voltage(rightVoltage * 1000);
  }*/
  //cout << chassis.get_absolute_heading() << " " << R_SidewaysTracker.get_position() << " " <<  R_ForwardTracker.get_position() << endl;
}

//Set PID constants
void Drive::set_turn_constants(float turn_max_voltage, float turn_kp, float turn_ki, float turn_kd, float turn_starti){
  this->turn_max_voltage = turn_max_voltage;
  this->turn_kp = turn_kp;
  this->turn_ki = turn_ki;
  this->turn_kd = turn_kd;
  this->turn_starti = turn_starti;
} 

void Drive::set_drive_constants(float drive_max_voltage, float drive_kp, float drive_ki, float drive_kd, float drive_starti){
  this->drive_max_voltage = drive_max_voltage;
  this->drive_kp = drive_kp;
  this->drive_ki = drive_ki;
  this->drive_kd = drive_kd;
  this->drive_starti = drive_starti;
} 

void Drive::set_heading_constants(float heading_max_voltage, float heading_kp, float heading_ki, float heading_kd, float heading_starti){
  this->heading_max_voltage = heading_max_voltage;
  this->heading_kp = heading_kp;
  this->heading_ki = heading_ki;
  this->heading_kd = heading_kd;
  this->heading_starti = heading_starti;
}

void Drive::set_swing_constants(float swing_max_voltage, float swing_kp, float swing_ki, float swing_kd, float swing_starti){
  this->swing_max_voltage = swing_max_voltage;
  this->swing_kp = swing_kp;
  this->swing_ki = swing_ki;
  this->swing_kd = swing_kd;
  this->swing_starti = swing_starti;
} 

// Set PID exit conditions
void Drive::set_turn_exit_conditions(float turn_settle_error, float turn_settle_time, float turn_timeout){
  this->turn_settle_error = turn_settle_error;
  this->turn_settle_time = turn_settle_time;
  this->turn_timeout = turn_timeout;
}

void Drive::set_drive_exit_conditions(float drive_settle_error, float drive_settle_time, float drive_timeout){
  this->drive_settle_error = drive_settle_error;
  this->drive_settle_time = drive_settle_time;
  this->drive_timeout = drive_timeout;
}

void Drive::set_swing_exit_conditions(float swing_settle_error, float swing_settle_time, float swing_timeout){
  this->swing_settle_error = swing_settle_error;
  this->swing_settle_time = swing_settle_time;
  this->swing_timeout = swing_timeout;
}

float Drive::get_absolute_heading(){
  return Gyro.get_heading();
  //return(reduce_0_to_360(Gyro.get_rotation() * (360.0/358))); 
}

float Drive::get_left_position_in(){
  return(DriveL.get_position(0)*in_to_deg);
}

float Drive::get_right_position_in(){
  return(DriveR.get_position(0)*in_to_deg);
}

void Drive::turn_to_angle(float angle){
  turn_to_angle(angle, turn_max_voltage, turn_settle_error, turn_settle_time, turn_timeout, turn_kp, turn_ki, turn_kd, turn_starti);
}

void Drive::turn_to_angle(float angle, float turn_max_voltage){
  turn_to_angle(angle, turn_max_voltage, turn_settle_error, turn_settle_time, 
  turn_timeout, turn_kp, turn_ki, turn_kd, turn_starti);
}

void Drive::turn_to_angle(float angle, float turn_max_voltage, float turn_settle_error, float turn_settle_time, float turn_timeout, float turn_kp, float turn_ki, float turn_kd, float turn_starti){
  //cout << chassis.get_absolute_heading() << " " << R_ForwardTracker.get_position() << " " << R_SidewaysTracker.get_position() << endl;
  desired_heading = angle;
  PID turnPID(reduce_negative_180_to_180(angle - get_absolute_heading()), turn_kp, turn_ki, turn_kd, turn_starti, turn_settle_error, turn_settle_time, turn_timeout);
  while(turnPID.is_settled() == false){
    float error = reduce_negative_180_to_180(angle - get_absolute_heading());
    float output = turnPID.compute(error);
    output = clamp(output, -turn_max_voltage, turn_max_voltage);
    drive_with_voltage(output, -output);
    pros::Task::delay(10);
  }
  DriveL.brake();
  DriveR.brake();
  //cout << chassis.get_absolute_heading() << " " << R_ForwardTracker.get_position() << " " << R_SidewaysTracker.get_position() << endl;
}

void Drive::drive_distance(float distance){
  drive_distance(distance, desired_heading, drive_max_voltage, heading_max_voltage, drive_settle_error, drive_settle_time, drive_timeout, drive_kp, drive_ki, drive_kd, drive_starti, heading_kp, heading_ki, heading_kd, heading_starti);
}

void Drive::drive_distance(float distance, float drive_max_voltage){
  drive_distance(distance, desired_heading, drive_max_voltage, heading_max_voltage, drive_settle_error, drive_settle_time, drive_timeout, drive_kp, drive_ki, drive_kd, drive_starti, heading_kp, heading_ki, heading_kd, heading_starti);
}

void Drive::drive_distance(float distance, float heading, float drive_max_voltage, float heading_max_voltage){
  drive_distance(distance, heading, drive_max_voltage, heading_max_voltage, drive_settle_error, drive_settle_time, drive_timeout, drive_kp, drive_ki, drive_kd, drive_starti, heading_kp, heading_ki, heading_kd, heading_starti);
}

void Drive::drive_distance(float distance, float drive_max_voltage, float drive_settle_error, float drive_settle_time, float drive_timeout, float drive_kp, float drive_ki, float drive_kd, float drive_starti){
  PID drivePID(distance, drive_kp, drive_ki, drive_kd, drive_starti, drive_settle_error, drive_settle_time, drive_timeout);
  float start_average_position = (get_left_position_in()+get_right_position_in())/2.0;
  // Rather than resetting the drive position , this function just notes what the drive position started at and determines error relative to that value.
  float average_position = start_average_position;
  while(drivePID.is_settled() == false){
    average_position = (get_left_position_in()+get_right_position_in())/2.0;
    float drive_error = distance+start_average_position-average_position;
    // Just like for turns, reducing from -180 to 180 degrees ensures that the robot takes the 
    // quickest path to the desired heading.
    float drive_output = drivePID.compute(drive_error);
    drive_output = clamp(drive_output, -drive_max_voltage, drive_max_voltage);
    drive_with_voltage(drive_output, drive_output);
    pros::Task::delay(10);
  }
  DriveL.brake();
  DriveR.brake();
}


void Drive::drive_distance(float distance, float heading, float drive_max_voltage, float heading_max_voltage, float drive_settle_error, float drive_settle_time, float drive_timeout, float drive_kp, float drive_ki, float drive_kd, float drive_starti, float heading_kp, float heading_ki, float heading_kd, float heading_starti){
  desired_heading = heading;
  PID drivePID(distance, drive_kp, drive_ki, drive_kd, drive_starti, drive_settle_error, drive_settle_time, drive_timeout);
  PID headingPID(reduce_negative_180_to_180(heading - get_absolute_heading()), heading_kp, heading_ki, heading_kd, heading_starti);
  float start_average_position = (get_left_position_in()+get_right_position_in())/2.0;
  // Rather than resetting the drive position , this function just notes what the drive position started at
  // and determines error relative to that value.
  float average_position = start_average_position;
  while(drivePID.is_settled() == false){
    average_position = (get_left_position_in()+get_right_position_in())/2.0;
    float drive_error = distance+start_average_position-average_position;
    float heading_error = reduce_negative_180_to_180(heading - get_absolute_heading());
    // Just like for turns, reducing from -180 to 180 degrees ensures that the robot takes the 
    // quickest path to the desired heading.
    float drive_output = drivePID.compute(drive_error);
    float heading_output = headingPID.compute(heading_error);

    drive_output = clamp(drive_output, -drive_max_voltage, drive_max_voltage);
    heading_output = clamp(heading_output, -heading_max_voltage, heading_max_voltage);
    drive_with_voltage(drive_output+heading_output, drive_output-heading_output);
    pros::Task::delay(10);
  }
  DriveL.brake();
  DriveR.brake();
}



///////////////////////////////////////////
// DISTANCE SENSOR DRIVE UNTIL FUNCTIONS BELOW
//////////////////////////////////////////

void Drive::drive_until(float distance){
  drive_until(distance, desired_heading, drive_max_voltage, heading_max_voltage, drive_settle_error, drive_settle_time, drive_timeout, drive_kp, drive_ki, drive_kd, drive_starti, heading_kp, heading_ki, heading_kd, heading_starti);
}

void Drive::drive_until(float distance, float drive_max_voltage){
  drive_until(distance, desired_heading, drive_max_voltage, heading_max_voltage, drive_settle_error, drive_settle_time, drive_timeout, drive_kp, drive_ki, drive_kd, drive_starti, heading_kp, heading_ki, heading_kd, heading_starti);
}

void Drive::drive_until(float distance, float heading, float drive_max_voltage, float heading_max_voltage){
  drive_until(distance, heading, drive_max_voltage, heading_max_voltage, drive_settle_error, drive_settle_time, drive_timeout, drive_kp, drive_ki, drive_kd, drive_starti, heading_kp, heading_ki, heading_kd, heading_starti);
}

void Drive::drive_until(float distance, float drive_max_voltage, float drive_settle_error, float drive_settle_time, float drive_timeout, float drive_kp, float drive_ki, float drive_kd, float drive_starti){
  std::cout << "regular drive_until is called "<< std::endl;
  PID drivePID(distance_from_nearest_object_v() - distance, drive_kp, drive_ki, drive_kd, drive_starti, drive_settle_error, drive_settle_time, drive_timeout);
  std::cout << "regular drive_until is called1 "<< std::endl;
  float start_average_position = (get_left_position_in()+get_right_position_in())/2.0;
  // Rather than resetting the drive position , this function just notes what the drive position started at and determines error relative to that value.
  float average_position = start_average_position;
  std::cout << "is_settled() =  "<< drivePID.is_settled() << std::endl;
  while(drivePID.is_settled() == false){
    average_position = (get_left_position_in()+get_right_position_in())/2.0;
    float drive_error = distance+start_average_position-average_position;
    // Just like for turns, reducing from -180 to 180 degrees ensures that the robot takes the 
    // quickest path to the desired heading.
    float drive_output = drivePID.compute(distance_from_nearest_object_v() - distance);
    drive_output = clamp(drive_output, -drive_max_voltage, drive_max_voltage);
    drive_with_voltage(drive_output, drive_output);
    pros::Task::delay(10);
  }
  DriveL.brake();
  DriveR.brake();
}


void Drive::drive_until(float distance, float heading, float drive_max_voltage, float heading_max_voltage, float drive_settle_error, float drive_settle_time, float drive_timeout, float drive_kp, float drive_ki, float drive_kd, float drive_starti, float heading_kp, float heading_ki, float heading_kd, float heading_starti){
  std::cout << "heading drive_until is called "<< std::endl;
  desired_heading = heading;
  PID drivePID(distance_from_nearest_object_v() - distance, drive_kp, drive_ki, drive_kd, drive_starti, drive_settle_error, drive_settle_time, drive_timeout);
   std::cout << "heading drive_until is called "<< std::endl;
  PID headingPID(reduce_negative_180_to_180(heading - get_absolute_heading()), heading_kp, heading_ki, heading_kd, heading_starti);
  float start_average_position = (get_left_position_in()+get_right_position_in())/2.0;
  // Rather than resetting the drive position , this function just notes what the drive position started at
  // and determines error relative to that value.
  float average_position = start_average_position;
  std::cout << "is_settled() =  "<< drivePID.is_settled() << std::endl;
  while(drivePID.is_settled() == false){
    average_position = (get_left_position_in()+get_right_position_in())/2.0;
    float drive_error = distance+start_average_position-average_position;
    float heading_error = reduce_negative_180_to_180(heading - get_absolute_heading());
    // Just like for turns, reducing from -180 to 180 degrees ensures that the robot takes the 
    // quickest path to the desired heading.
    std::cout << "error: " << distance_from_nearest_object_v() - distance << std::endl;
    float drive_output = drivePID.compute(distance_from_nearest_object_v() - distance);
    std::cout << "motor output: " << drive_output << std::endl;
    float heading_output = headingPID.compute(heading_error);

    drive_output = clamp(drive_output, -drive_max_voltage, drive_max_voltage);
    heading_output = clamp(heading_output, -heading_max_voltage, heading_max_voltage);
    drive_with_voltage(drive_output+heading_output, drive_output-heading_output);
    pros::Task::delay(10);
  }
  DriveL.brake();
  DriveR.brake();
}

////////////////////////////////////////////
/////////////////////////////////////////////

void Drive::left_swing_to_angle(float angle){
  left_swing_to_angle(angle, swing_max_voltage, swing_settle_error, swing_settle_time, swing_timeout, swing_kp, swing_ki, swing_kd, swing_starti);
}

void Drive::left_swing_to_angle(float angle, float swing_max_voltage){
  left_swing_to_angle(angle, swing_max_voltage, swing_settle_error, swing_settle_time, swing_timeout, swing_kp, swing_ki, swing_kd, swing_starti);
}

void Drive::left_swing_to_angle(float angle, float swing_max_voltage, float swing_settle_error, float swing_settle_time, float swing_timeout, float swing_kp, float swing_ki, float swing_kd, float swing_starti){
  desired_heading = angle;
  PID swingPID(reduce_negative_180_to_180(angle - get_absolute_heading()), swing_kp, swing_ki, swing_kd, swing_starti, swing_settle_error, swing_settle_time, swing_timeout);
  while(swingPID.is_settled() == false){
    float error = reduce_negative_180_to_180(angle - get_absolute_heading());
    float output = swingPID.compute(error);
    output = clamp(output, -swing_max_voltage, swing_max_voltage);
    DriveL.move_voltage(output * 1000 + 200);
    //Only the left side of the drive turns, making this a "left swing".
    DriveR.set_brake_mode(MOTOR_BRAKE_HOLD);
    DriveR.brake();
    pros::Task::delay(10);
  }
  DriveL.brake();
  DriveR.brake();
}

void Drive::right_swing_to_angle(float angle){
  right_swing_to_angle(angle, swing_max_voltage, swing_settle_error, swing_settle_time, swing_timeout, swing_kp, swing_ki, swing_kd, swing_starti);
}

void Drive::right_swing_to_angle(float angle, float swing_max_voltage){
  right_swing_to_angle(angle, swing_max_voltage, swing_settle_error, swing_settle_time, swing_timeout, swing_kp, swing_ki, swing_kd, swing_starti);
}

void Drive::right_swing_to_angle(float angle, float swing_max_voltage, float swing_settle_error, float swing_settle_time, float swing_timeout, float swing_kp, float swing_ki, float swing_kd, float swing_starti){
  desired_heading = angle;
  PID swingPID(reduce_negative_180_to_180(angle - get_absolute_heading()), swing_kp, swing_ki, swing_kd, swing_starti, swing_settle_error, swing_settle_time, swing_timeout);
  while(swingPID.is_settled() == false){
    float error = reduce_negative_180_to_180(angle - get_absolute_heading());
    float output = swingPID.compute(error);
    output = clamp(output, -swing_max_voltage, swing_max_voltage);
    DriveR.move_voltage(-output * 5000);
    //Only the right side of the drive turns, so this is a "right swing".
    DriveL.set_brake_mode(MOTOR_BRAKE_HOLD);
    DriveL.brake();
    pros::Task::delay(10);
  }
  DriveR.brake();
  DriveL.brake();
}

float Drive::get_ForwardTracker_position(){
  if (drive_setup==ZERO_TRACKER_ODOM){
    // For zero tracker odom, the right side of the drive becomes the tracker.
    return(get_right_position_in());
  }
  if (drive_setup==TANK_ONE_ENCODER || drive_setup == TANK_TWO_ENCODER){
    return(E_ForwardTracker.get_value()*ForwardTracker_in_to_deg_ratio);
  }else{
    // This if-else just discriminates based on whether the sensor is an encoder or rotation sensor.
    return(R_ForwardTracker.get_position()*ForwardTracker_in_to_deg_ratio/100);
  }
}

float Drive::get_SidewaysTracker_position(){
  if (drive_setup==TANK_ONE_ENCODER || drive_setup == TANK_ONE_ROTATION || drive_setup == ZERO_TRACKER_ODOM){
    return(0);
    // These setups all pretend that there is a sideways tracker  in the center that just never moves.
  }else if (drive_setup == TANK_TWO_ENCODER){
    return(E_SidewaysTracker.get_value()*SidewaysTracker_in_to_deg_ratio);
  }else{
    return(R_SidewaysTracker.get_position()*SidewaysTracker_in_to_deg_ratio/100);
  }
}

void Drive::position_track(){
  while(1){
    odom.update_position(get_ForwardTracker_position(), get_SidewaysTracker_position(), get_absolute_heading());
    pros::Task::delay(5);
  }
}

void Drive::set_heading(float orientation_deg){
  //std::cout << orientation_deg << std::endl;
  
  
  
  Gyro.set_heading(orientation_deg*GYRO_SCALE/360.0);
  //Gyro.set_heading(orientation_deg);
  //std::cout << Gyro.get_heading() << std::endl;

}

void Drive::set_coordinates(float X_position, float Y_position, float orientation_deg){
  odom.set_position(X_position, Y_position, orientation_deg, get_ForwardTracker_position(), get_SidewaysTracker_position());
  set_heading(orientation_deg);
  odom_task = pros::Task(position_track_task);
}

float Drive::get_X_position(){
  return(odom.X_position);
}

float Drive::get_Y_position(){
  return(odom.Y_position);
}

void Drive::drive_to_point(float X_position, float Y_position){
  drive_to_point(X_position, Y_position, drive_max_voltage, heading_max_voltage, drive_settle_error, drive_settle_time, drive_timeout, drive_kp, drive_ki, drive_kd, drive_starti, heading_kp, heading_ki, heading_kd, heading_starti);
}

void Drive::drive_to_point(float X_position, float Y_position, float drive_max_voltage, float heading_max_voltage){
  drive_to_point(X_position, Y_position, drive_max_voltage, heading_max_voltage, drive_settle_error, drive_settle_time, drive_timeout, drive_kp, drive_ki, drive_kd, drive_starti, heading_kp, heading_ki, heading_kd, heading_starti);
}

void Drive::drive_to_point(float X_position, float Y_position, float drive_max_voltage, float heading_max_voltage, float drive_settle_error, float drive_settle_time, float drive_timeout){
  drive_to_point(X_position, Y_position, drive_max_voltage, heading_max_voltage, drive_settle_error, drive_settle_time, drive_timeout, drive_kp, drive_ki, drive_kd, drive_starti, heading_kp, heading_ki, heading_kd, heading_starti);
}

void Drive::drive_until(double voltage, double distance, int timeout){
 int start_time = pros::millis();
 //double d = distance * 25.4;
    // Debug: see what the sensor is actually reading
    pros::screen::print(TEXT_LARGE, 50, 200, 
        ("Dist: " + std::to_string(chassis.distance_from_nearest_object_v())).c_str());
  // Pause to read the value
  while (chassis.distance_from_nearest_object_v() > distance){
    //DriveL.move_voltage(voltage * 1000);
    //DriveR.move_voltage(voltage * 1000);
    drive_with_voltage(voltage, voltage);
    std::cout << "Current Distance:" << chassis.distance_from_nearest_object_v() << std::endl;
    std::cout << "Stop Distance:" << distance << std::endl;
    //pros::delay(5);
  }
   drive_with_voltage(0, 0);
  
}

void Drive::drive_to_point(float X_position, float Y_position, float drive_max_voltage, float drive_settle_error, float drive_settle_time, float drive_timeout, float drive_kp, float drive_ki, float drive_kd, float drive_starti){
  PID drivePID(hypot(X_position-get_X_position(),Y_position-get_Y_position()), drive_kp, drive_ki, drive_kd, drive_starti, drive_settle_error, drive_settle_time, drive_timeout);
  while(drivePID.is_settled() == false){
    float drive_error = hypot(X_position-get_X_position(),Y_position-get_Y_position());
    // The drive error is just equal to the distance between the current and desired points.
    float drive_output = drivePID.compute(drive_error);
    drive_output = clamp(drive_output, drive_max_voltage, drive_max_voltage);
    drive_with_voltage(drive_output, drive_output);
    pros::Task::delay(10);
  }
  desired_heading = get_absolute_heading();
  set_brake_mode('H');
  DriveL.brake();
  DriveR.brake();
}

void Drive::drive_to_point(float X_position, float Y_position, float drive_max_voltage, float heading_max_voltage, float drive_settle_error, float drive_settle_time, float drive_timeout, float drive_kp, float drive_ki, float drive_kd, float drive_starti, float heading_kp, float heading_ki, float heading_kd, float heading_starti){
  
  PID drivePID(hypot(X_position-get_X_position(),Y_position-get_Y_position()), drive_kp, drive_ki, drive_kd, drive_starti, drive_settle_error, drive_settle_time, drive_timeout);
  PID headingPID(reduce_negative_180_to_180(to_deg(atan2(X_position-get_X_position(),Y_position-get_Y_position()))-get_absolute_heading()), heading_kp, heading_ki, heading_kd, heading_starti);
  /*set_brake_mode('C');
  while (true) {
    float heading_error = reduce_negative_180_to_180(to_deg(atan2(X_position-get_X_position(),Y_position-get_Y_position()))-get_absolute_heading());
    cout << heading_error << " " << cos(to_rad(heading_error)) << " "  << headingPID.compute(heading_error) << endl;
    pros::delay(500);
  }*/
  
  while(drivePID.is_settled() == false){
    float drive_error = hypot(X_position-get_X_position(),Y_position-get_Y_position());

    float heading_error = reduce_negative_180_to_180(to_deg(atan2(X_position-get_X_position(),Y_position-get_Y_position()))-get_absolute_heading());
    //cout << heading_error << endl;  
    float drive_output = drivePID.compute(drive_error);

    float heading_scale_factor = cos(to_rad(heading_error));
    drive_output*=heading_scale_factor;

    heading_error = reduce_negative_90_to_90(heading_error);
    
    float heading_output = headingPID.compute(heading_error);
    //cout << heading_error << " " << heading_output << endl;
    if (drive_error < drive_settle_error*2) { 
      heading_output = 0; 
    }

    drive_output = clamp(drive_output, -fabs(heading_scale_factor)-drive_max_voltage, fabs(heading_scale_factor)+drive_max_voltage);
    heading_output = clamp(heading_output, -heading_max_voltage, heading_max_voltage);
    //cout << heading_error << " " << drive_output + heading_output << " " << drive_output - heading_output <<  endl;
    drive_with_voltage(drive_output + heading_output, drive_output - heading_output);
    pros::Task::delay(10);
  }
  desired_heading = get_absolute_heading();
  DriveR.set_brake_mode(MOTOR_BRAKE_HOLD);
  DriveR.brake();
  DriveL.set_brake_mode(MOTOR_BRAKE_HOLD);
  DriveL.brake();
  //pros::delay(500);
  //std::cout << chassis.get_X_position() << " " << chassis.get_Y_position() << " " << chassis.get_absolute_heading() << std::endl;
}

void Drive::turn_to_point(float X_position, float Y_position){
  turn_to_point(X_position, Y_position, 0, turn_max_voltage, turn_settle_error, turn_settle_time, turn_timeout, turn_kp, turn_ki, turn_kd, turn_starti);
}

void Drive::turn_to_point(float X_position, float Y_position, float extra_angle_deg, float turn_max_voltage){
  turn_to_point(X_position, Y_position, extra_angle_deg, turn_max_voltage, turn_settle_error, turn_settle_time, turn_timeout, turn_kp, turn_ki, turn_kd, turn_starti);
}

void Drive::turn_to_point(float X_position, float Y_position, float extra_angle_deg, float turn_max_voltage, float turn_settle_error, float turn_settle_time, float turn_timeout){
  turn_to_point(X_position, Y_position, extra_angle_deg, turn_max_voltage, turn_settle_error, turn_settle_time, turn_timeout, turn_kp, turn_ki, turn_kd, turn_starti);
}

void Drive::turn_to_point(float X_position, float Y_position, float extra_angle_deg, float turn_max_voltage, float turn_settle_error, float turn_settle_time, float turn_timeout, float turn_kp, float turn_ki, float turn_kd, float turn_starti){
  PID turnPID(reduce_negative_180_to_180(to_deg(atan2(X_position-get_X_position(),Y_position-get_Y_position())) - get_absolute_heading()), turn_kp, turn_ki, turn_kd, turn_starti, turn_settle_error, turn_settle_time, turn_timeout);
  while(turnPID.is_settled() == false){
    float error = reduce_negative_180_to_180(to_deg(atan2(X_position-get_X_position(),Y_position-get_Y_position())) - get_absolute_heading() + extra_angle_deg);
    float output = turnPID.compute(error);
    output = clamp(output, -turn_max_voltage, turn_max_voltage);
    drive_with_voltage(output, -output);
    pros::Task::delay(10);
  }
  desired_heading = get_absolute_heading();
  DriveL.set_brake_mode(MOTOR_BRAKE_HOLD);
  DriveL.brake();
  DriveR.set_brake_mode(MOTOR_BRAKE_HOLD);
  DriveR.brake();
  //pros::delay(500);
  //std::cout << chassis.get_X_position() << " " << chassis.get_Y_position() << " " << chassis.get_absolute_heading() << std::endl;
}

void Drive::left_swing_to_point(float X_position, float Y_position){
 left_swing_to_point(X_position, Y_position, 0, swing_max_voltage, swing_settle_error, swing_settle_time, swing_timeout, swing_kp, swing_ki, swing_kd, swing_starti);
}

void Drive::left_swing_to_point(float X_position, float Y_position, float extra_angle_deg, float swing_max_voltage){
 left_swing_to_point(X_position, Y_position, extra_angle_deg, swing_max_voltage, swing_settle_error, swing_settle_time, swing_timeout, swing_kp, swing_ki, swing_kd, swing_starti);
}


void Drive::left_swing_to_point(float X_position, float Y_position, float extra_angle_deg, float swing_max_voltage, float swing_settle_error, float swing_settle_time, float swing_timeout, float swing_kp, float swing_ki, float swing_kd, float swing_starti){
  PID turnPID(reduce_negative_180_to_180(to_deg(atan2(X_position-get_X_position(),Y_position-get_Y_position())) - get_absolute_heading()), swing_kp, swing_ki, swing_kd, swing_starti, swing_settle_error, swing_settle_time, swing_timeout);
  while(turnPID.is_settled() == false){
    float error = reduce_negative_180_to_180(to_deg(atan2(X_position-get_X_position(),Y_position-get_Y_position())) - get_absolute_heading() + extra_angle_deg);
    // Again, using atan2(x,y) puts 0 degrees on the positive Y axis.
    float output = turnPID.compute(error);
    output = clamp(output, -swing_max_voltage, swing_max_voltage);
    DriveL.move_voltage(output * 1000);
    DriveR.set_brake_mode(MOTOR_BRAKE_HOLD);
    DriveR.brake();
    pros::Task::delay(10);
  }
  DriveL.brake();
  DriveR.brake();
}

void Drive::right_swing_to_point(float X_position, float Y_position){
right_swing_to_point(X_position, Y_position, 0, swing_max_voltage, swing_settle_error, swing_settle_time, swing_timeout, swing_kp, swing_ki, swing_kd, swing_starti);
}


void Drive::right_swing_to_point(float X_position, float Y_position, float extra_angle_deg, float swing_max_voltage){
right_swing_to_point(X_position, Y_position, extra_angle_deg, swing_max_voltage, swing_settle_error, swing_settle_time, swing_timeout, swing_kp, swing_ki, swing_kd, swing_starti);
}

void Drive::right_swing_to_point(float X_position, float Y_position, float extra_angle_deg, float swing_max_voltage, float swing_settle_error, float swing_settle_time, float swing_timeout, float swing_kp, float swing_ki, float swing_kd, float swing_starti){
 PID turnPID(reduce_negative_180_to_180(to_deg(atan2(X_position-get_X_position(),Y_position-get_Y_position())) - get_absolute_heading()), swing_kp, swing_ki, swing_kd, swing_starti, swing_settle_error, swing_settle_time, swing_timeout);
 while(turnPID.is_settled() == false){
   float error = reduce_negative_180_to_180(to_deg(atan2(X_position-get_X_position(),Y_position-get_Y_position())) - get_absolute_heading() + extra_angle_deg);
   // Again, using atan2(x,y) puts 0 degrees on the positive Y axis.
   float output = turnPID.compute(error);
   output = clamp(output, -swing_max_voltage, swing_max_voltage);
   DriveR.move_voltage(output * 1000);
   DriveL.set_brake_mode(MOTOR_BRAKE_HOLD);
   DriveL.brake();
   pros::Task::delay(10);
 }
 DriveR.brake();
 DriveL.brake();
}

int Drive::position_track_task(){
  chassis.position_track();
  return(0);
}

double Drive::absolute_y_pos()
{
  return odom.Y_position;
}

void Drive::y_pos_task()
{
  print_coords();
}

void Drive::print_coords()
{
  std::cout << odom.X_position << "  " << odom.X_position << "  " << chassis.get_absolute_heading() << std::endl;
}

void Drive::calculate() {
  while (true) {
      /*string x_str = std::to_string(chassis.get_X_position());
      string y_str = std::to_string(chassis.get_Y_position());
      string heading_str = std::to_string(chassis.get_absolute_heading());
      pros::screen::draw_rect(0, 0, 480, 240);
      pros::screen::set_pen(pros::Color::white);
      pros::screen::print(TEXT_LARGE, 50, 50, x_str.c_str());
      pros::screen::print(TEXT_LARGE, 50, 100, y_str.c_str());
      pros::screen::print(TEXT_LARGE, 50, 150, heading_str.c_str()); 
      pros::delay(3000);*/
  }
}