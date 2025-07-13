#pragma once

typedef struct {
  float kp, ki, kd;
  float integral, prev_error;
  float output_min, output_max;
} PID_Controller;

void PID_Init(PID_Controller *pid, float kp, float ki, float kd, float min, float max);
float PID_Update(PID_Controller *pid, float error, float dt);
