#include <Arduino.h>
#include <Zumo32U4.h>
#include <Wire.h>

typedef struct {
    int proximiteGauche;
    int proximiteAvant;
    int proximiteDroite;

    int LigneGauche;
    int LigneAvant;
    int LigneDroit;
}ValeurCapteurs;

class Capteurs
{

public:
    void begin(){
        lineSensors = new (Zumo32U4LineSensors);
        proxSensors = new (Zumo32U4ProximitySensors);
        lineSensors->initThreeSensors();
        proxSensors->initThreeSensors();

        calibrateLineSensors();
    }

    void printReadingsToSerial();

    ValeurCapteurs readSensors();

    void printValeurCapteurs(ValeurCapteurs valeur);
    
private:
    Zumo32U4LineSensors *lineSensors;
    Zumo32U4ProximitySensors *proxSensors;

    unsigned int lineSensorValues[5] = { 0, 0, 0, 0, 0 };

    bool proxLeftActive;
    bool proxFrontActive;
    bool proxRightActive;

    void calibrateLineSensors();
};
