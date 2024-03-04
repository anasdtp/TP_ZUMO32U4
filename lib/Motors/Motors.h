#include <Arduino.h>

#define VIT_MAX 155

void write_PWMD(float vit);
void write_PWMG(float vit);

void Moteur_D_PH_Write(bool on);
void PWM_D_Write(float vitD);
void Moteur_G_PH_Write(bool on);
void PWM_G_Write(float vitD);

int16_t lireCodeurD();
int16_t lireCodeurG();
int16_t resetCodeurD();
int16_t resetCodeurG();