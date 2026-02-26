#ifndef CONTROL_H_
#define CONTROL_H_

typedef struct {
  float u1;
  float u2;
  float u3;
  float u4;
} Control_Inputs;

typedef struct {
  float motor_value_1;
  float motor_value_2;
  float motor_value_3;
  float motor_value_4;
} Motor_Values;

Motor_Values control_inputs_to_motor_values(Control_Inputs inputs);
Motor_Values clip_motor_values(Motor_Values values);
float pid_control_pitch(float pitch, float desired_pitch, float kp, float ki,
                        float kd);
float pid_control_roll(float roll, float desired_roll, float kp, float ki,
                       float kd);
void shutdown();

#endif
