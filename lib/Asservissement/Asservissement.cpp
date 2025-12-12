#include <Asservissement.h>

Asservissement::Asservissement(float Kp, float Ki, float Kd)
{
  Kp_ = Kp;
  Ki_ = Ki;
  Kd_ = Kd;
}

Asservissement::~Asservissement()
{
}

float Asservissement::calculatePID(float error, float dt)
{
    // Variables for PID control
    integral += error * dt;
    if (integral > MAX_INTEGRAL) {
      integral = MAX_INTEGRAL;
    } else if (integral < -MAX_INTEGRAL) {
      integral = -MAX_INTEGRAL;
    }
    derivative = (error - prev_error) / dt;
    output = Kp_ * error + Ki_ * integral + Kd_ * derivative;
    prev_error = error;
    return output;
}

/**********************************************************************************/
/* NOM : Asser_Pos                                                                */
/* ARGUMENT : float pcons -> position voulue exprimée en tick d'encodeur          */
/*            float pos -> position actuelle exprimée en tick d'encodeur          */
/*            float dt -> temps depuis le debut du mouvement en seconde           */
/*                                1024 par ex par tour de roue                    */
/* RETOUR : rien                                                                  */
/* DESCRIPTIF : Fonction appelee pour calculer la commande d'un moteur            */
/**********************************************************************************/
float Asservissement::AsserMot(float pcons, float pos, float dt){
    float error = pcons - pos;
    return calculatePID(error, dt);
}

// float AsserMotD(float pcons, float dt){
//     float pos = lireCodeurD();
//     float error = pcons - pos;
//     return calculatePID(error, dt);
// }