#include <Motors.h>
#include <Zumo32U4.h>

Zumo32U4Encoders encoders;

void write_PWMD(float vitD){
    if (vitD >= 0) // Mode Avancer
    {
        Moteur_D_PH_Write(0);
        if (vitD > VIT_MAX)
            {vitD = VIT_MAX;} // Palier de vitesse fixé à 250
        PWM_D_Write(vitD);
    }
    else if (vitD < 0) // Mode Reculer
    {
        Moteur_D_PH_Write(1);
        if (vitD < -VIT_MAX)
            {vitD = -VIT_MAX;} // Palier de vitesse fixé à 250
        PWM_D_Write(-vitD);
    }
}

void write_PWMG(float vitG){
    if (vitG >= 0) // Mode Avancer
    {
        Moteur_G_PH_Write(0);
        if (vitG > VIT_MAX)
            {vitG = VIT_MAX;} // Palier de vitesse fixé à 250
        PWM_G_Write(vitG);
    }
    else if (vitG < 0) // Mode Reculer
    {
        Moteur_G_PH_Write(1);
        if (vitG < -VIT_MAX)
            {vitG = -VIT_MAX;} // Palier de vitesse fixé à 250
        PWM_G_Write(-vitG);
    }
}

void Moteur_D_PH_Write(bool on){
    static int pinMotDPH = 15, init = 1;
    if(init){init = 0; pinMode(pinMotDPH, OUTPUT);}
    digitalWrite(pinMotDPH, on);
}
void PWM_D_Write(float vitD){
    static int pinMotDEN = 9, init = 1;
    if(init){init = 0; pinMode(pinMotDEN, OUTPUT);}
    analogWrite(pinMotDEN, vitD);
}

void Moteur_G_PH_Write(bool on){
    static int pinMotGPH = 16, init = 1;
    if(init){init = 0; pinMode(pinMotGPH, OUTPUT);}
    digitalWrite(pinMotGPH, on);
}
void PWM_G_Write(float vitD){
    static int pinMotGEN = 10, init = 1;
    if(init){init = 0; pinMode(pinMotGEN, OUTPUT);}
    analogWrite(pinMotGEN, vitD);
}

int16_t lireCodeurD(){
    return encoders.getCountsRight();
}

int16_t lireCodeurG(){
    return encoders.getCountsLeft();
}

int16_t resetCodeurD(){
    return encoders.getCountsAndResetRight();
}

int16_t resetCodeurG(){
    return encoders.getCountsAndResetLeft();
}