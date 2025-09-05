// PID.hpp
#pragma once
#ifndef _USER_PID_HPP
#define _USER_PID_HPP

#include "main.h"
using namespace std;

class PID
{
public:
  float error              = 0;
  float kp                 = 0;
  float ki                 = 0;
  float kd                 = 0;
  float starti             = 0;
  float settle_error       = 0;
  float settle_time        = 0;
  float timeout            = 0;
  float accumulated_error  = 0;
  float previous_error     = 0;
  float output             = 0;
  float time_spent_settled = 0;
  float time_spent_running = 0;

  PID(float error,
      float kp, float ki, float kd, float starti);

  PID(float error,
      float kp, float ki, float kd, float starti,
      float settle_error, float settle_time, float timeout);

  float compute(float error);
  bool is_settled();
};

#endif  // _USER_PID_HPP
