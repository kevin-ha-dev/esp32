#pragma once

// Signed speed for PID: -1.0 = full reverse, 0 = stop, +1.0 = full forward.
// Direction changes immediately; there is no delay inside this call.
void motor_init(void);
void motor_set(float speed);
void motor_stop(void);
