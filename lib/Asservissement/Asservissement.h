#include <Arduino.h>

class Asservissement
{
public:
  Asservissement(){
    
  }
  Asservissement(float Kp, float Ki, float Kd);
  ~Asservissement();

  float AsserMot(float pcons, float pos, float dt);//dt en secondes
private:
  // Constants for PID control
  float Kp_; // Proportional gain
  float Ki_; // Integral gain
  float Kd_; // Derivative gain

  float calculatePID(float error, float dt);
};

