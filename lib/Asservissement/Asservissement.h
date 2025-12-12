#include <Arduino.h>

class Asservissement
{
public:
  Asservissement(){
    
  }
  Asservissement(float Kp, float Ki, float Kd);
  ~Asservissement();

  float AsserMot(float pcons, float pos, float dt);//dt en secondes
  float calculatePID(float error, float dt);
  void resetPID(){
      integral = 0.0;
      prev_error = 0.0;
  }

private:
  // Constants for PID control
  float Kp_; // Proportional gain
  float Ki_; // Integral gain
  float Kd_; // Derivative gain

  float integral = 0.0, prev_error = 0.0;
  const float MAX_INTEGRAL = (2000.0F);

  float derivative;
  float output;

};

