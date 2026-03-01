// PID.cpp
#include "main.h"
#include <cmath>
#include <iostream>

PID::PID(float error,
         float kp, float ki, float kd, float starti)
  : error(error),
    kp(kp), ki(ki), kd(kd),
    starti(starti)
{}

PID::PID(float error,
         float kp, float ki, float kd, float starti,
         float settle_error, float settle_time, float timeout)
  : error(error),
    kp(kp), ki(ki), kd(kd),
    starti(starti),
    settle_error(settle_error),
    settle_time(settle_time),
    timeout(timeout)
{}

float PID::compute(float error) {
  // Proportional, Integral, Derivative
  float P = kp * error;
  float I = ki * accumulated_error;
  float D = kd * (error - previous_error);

  output = P + I + D;

  previous_error = error;
  accumulated_error += error + starti;

  // settled logic
  if (std::fabs(error) < settle_error) {
    time_spent_settled += 10;
  } else {
    time_spent_settled = 0;
  }
  time_spent_running += 10;

  return output;
}

bool PID::is_settled(){
  if (time_spent_running > timeout && timeout != 0) {
    std::cout << "bad :" << time_spent_running << std::endl;
    return true;
  }
  if (time_spent_settled > settle_time) {
    std::cout << "good :" << time_spent_running << std::endl;
    return true;
  }
  return false;
}
