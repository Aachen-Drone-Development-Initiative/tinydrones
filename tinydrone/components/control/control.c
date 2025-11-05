#include "control.h"

// Control inputs
static float s_u1; // u1: thrust up
static float s_u2; // u2: roll/pitch control axis rotor 2 and 4
static float s_u3; // u3: roll/pitch control axis rotor 1 and 3
static float s_u4; // u4: yaw control axis

//rotor forces
static float s_f1; // f1: thrust rotor 1
static float s_f2; // f2: thrust rotor 2
static float s_f3; // f3: thrust rotor 3
static float s_f4; // f4: thrust rotor 4

void control_inputs_to_rotor_forces(void) { // Using the linear equations given in the header to compute rotor forces 
    s_f1 = 0.25f * s_u1 + 0.5f * s_u3 + 0.25f * s_u4;
    s_f2 = -0.25f * s_u1 - 1.5f * s_u2 + 0.25f * s_u4;
    s_f3 = 0.25f * s_u1 - 0.25f * s_u3 + 0.25f * s_u4;
    s_f4 = -0.25f * s_u1 - 0.5f * s_u2 - 0.25f * s_u4;
}

void reset_control_inputs(void) { // Resets control inputs to zero
    s_u1 = 0.0f;
    s_u2 = 0.0f;
    s_u3 = 0.0f;
    s_u4 = 0.0f;
}

void reset_rotor_forces(void) { // Resets rotor forces to zero
    s_f1 = 0.0f;
    s_f2 = 0.0f;
    s_f3 = 0.0f;
    s_f4 = 0.0f;
}

void calculate_control_inputs(void) {
    #warning "calculate_control_inputs function is not implemented yet."
}

void control_rotors_with_f(void) {
    #warning "control_rotors_with_f function is not implemented yet."
}