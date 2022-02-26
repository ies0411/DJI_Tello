
/**
 * @brief PID library header
 *
 */
#ifndef __TELLO_PID_H__
#define __TELLO_PID_H__

#include <cmath>
#include <iostream>

class PIDImpl;
class PID {
   public:
    // Kp -  proportional gain
    // Ki -  Integral gain
    // Kd -  derivative gain
    // dt -  loop interval time
    // max - maximum value of manipulated variable
    // min - minimum value of manipulated variable

    void PID_set(double dt, double max, double min, double Kp, double Kd, double Ki);
    // Returns the manipulated variable given a setpoint and current process value
    double calculate(double setpoint, double pv);
    ~PID();

   private:
    PIDImpl *pimpl;
};

#endif
