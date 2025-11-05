#ifndef CONTROL_H_
#define CONTROL_H_

//Data inputs to receive, unknow as of now

// Control inputs
extern float s_u1; // u1: thrust up
extern float s_u2; // u2: roll/pitch control axis rotor 2 and 4
extern float s_u3; // u3: roll/pitch control axis rotor 1 and 3
extern float s_u4; // u4: yaw control axis

//rotor forces
extern float s_f1; // f1: thrust rotor 1
extern float s_f2; // f2: thrust rotor 2
extern float s_f3; // f3: thrust rotor 3
extern float s_f4; // f4: thrust rotor 4

void control_inputs_to_rotor_forces(); //converts u1, u2 ,u3 ,u4 to f1, f2, f3, f4 via lgs:
//lgs: u1 = f1 + f2 + f3 + f4
//lgs: u2 = -f2 + f4
//lgs: u3 = f1 - f3
//lgs: u4 = f1 + -f2 + f3 - f4

void reset_control_inputs(); //sets u1, u2, u3, u4 to zero

void reset_rotor_forces(); //sets f1, f2, f3, f4 to zero

void calculate_control_inputs(); //calculates u1, u2, u3, u4  | right now no clue how to do this

void control_rotors_with_f(void); //uses f1, f2, f3, f4 to control the rotors | no clue how to do this yet

#endif
