#include "pid.h"

void PID_Init(PID_Controller *pid, float kp, float ki, float kd, float min, float max) {
  pid->kp = kp; pid->ki = ki; pid->kd = kd;
  pid->output_min = min; pid->output_max = max;
}

float PID_Update(PID_Controller *pid, float error, float dt) {
  pid->integral += error * dt;
  pid->integral = (pid->integral > pid->output_max) ? pid->output_max :
                 (pid->integral < pid->output_min) ? pid->output_min : pid->integral;

  float derivative = (error - pid->prev_error) / dt;
  pid->prev_error = error;

  float output = pid->kp * error + pid->ki * pid->integral + pid->kd * derivative;
  return (output > pid->output_max) ? pid->output_max :
         (output < pid->output_min) ? pid->output_min : output;
}
