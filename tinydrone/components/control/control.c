#include "control.h"

#include "motor_driver.h"
#include <math.h>

#include "freertos/FreeRTOS.h"
#include "esp_sleep.h"

// Control inputs
// static float s_u1; // u1: thrust up
// static float s_u2; // u2: pitch control axis rotor 2 and 4
// static float s_u3; // u3: roll control axis rotor 1 and 3
// static float s_u4; // u4: yaw control axis

// //rotor forces
// static float s_f1; // f1: thrust rotor 1
// static float s_f2; // f2: thrust rotor 2
// static float s_f3; // f3: thrust rotor 3
// static float s_f4; // f4: thrust rotor 4

static portMUX_TYPE spinlock = portMUX_INITIALIZER_UNLOCKED;

Motor_Values control_inputs_to_motor_values(Control_Inputs inputs)
{ // Using the linear equations given in the header to compute rotor forces
    // inverse:
    // lgs: u1 = f1 + f2 + f3 + f4
    // lgs: u2 = -f2 + f4
    // lgs: u3 = - f1 + f3
    // lgs: u4 = f1 + -f2 + f3 - f4
    Motor_Values motor_values = {
        .motor_value_1 = 0.25f * inputs.u1 - 0.5f * inputs.u3 + 0.25f * inputs.u4,
        .motor_value_2 = 0.25f * inputs.u1 - 0.5f * inputs.u2 - 0.25f * inputs.u4,
        .motor_value_3 = 0.25f * inputs.u1 + 0.5f * inputs.u3 + 0.25f * inputs.u4,
        .motor_value_4 = 0.25f * inputs.u1 + 0.5f * inputs.u2 - 0.25f * inputs.u4
    };
    return motor_values;
}

Motor_Values clip_motor_values(Motor_Values values) {
    values.motor_value_1 = fmaxf(fminf(values.motor_value_1, 1.0f), 0.0f);
    values.motor_value_2 = fmaxf(fminf(values.motor_value_2, 1.0f), 0.0f);
    values.motor_value_3 = fmaxf(fminf(values.motor_value_3, 1.0f), 0.0f);
    values.motor_value_4 = fmaxf(fminf(values.motor_value_4, 1.0f), 0.0f);
    return values;
}

float pid_control_pitch(float pitch, float desired_pitch, float kp, float ki, float kd) {
    static float integral = 0.0f;
    static float previous_error = 0.0f;
    static float old_kp, old_ki, old_kd = 0;

    // reset for new pid parameters
    if (kp != old_kp || ki != old_ki || kd != old_kd) {
        integral = 0.0f;
        previous_error = 0.0f;
        old_kp = kp;
        old_ki = ki;
        old_kd = kd;
    }

    float error = desired_pitch - pitch;
    integral += error;
    float derivative = error - previous_error;
    float output = kp * error + ki * integral + kd * derivative;
    previous_error = error;
    return output;
}

float pid_control_roll(float roll, float desired_roll, float kp, float ki, float kd) {
    static float integral = 0.0f;
    static float previous_error = 0.0f;
    static float old_kp, old_ki, old_kd = 0;

    // reset for new pid parameters
    if (kp != old_kp || ki != old_ki || kd != old_kd) {
        integral = 0.0f;
        previous_error = 0.0f;
        old_kp = kp;
        old_ki = ki;
        old_kd = kd;
    }

    float error = desired_roll - roll;
    integral += error;
    float derivative = error - previous_error;
    float output = kp * error + ki * integral + kd * derivative;
    previous_error = error;
    return output;
}

void shutdown() {
    //shut all motors off
    taskENTER_CRITICAL(&spinlock);

    set_motor(1, 0);
    set_motor(2, 0);
    set_motor(3, 0);
    set_motor(4, 0);

    //disables everything on esp32 except causes for wakeup (no causes exist)
    esp_deep_sleep_start();

    return;
}
