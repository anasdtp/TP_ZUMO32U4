#ifndef AUTOMATE_H
#define AUTOMATE_H

#include <Arduino.h>
#include <Asservissement.h>
#include <Zumo32U4.h>


class Automate : public Asservissement
{
public:
    Automate(Zumo32U4ButtonB *buttonB, Zumo32U4LineSensors *lineSensors,
            Zumo32U4Motors *motors, Zumo32U4Encoders *encoders,
            Zumo32U4IMU *imu, Zumo32U4Buzzer *buzzer)
        : buttonB(buttonB), lineSensors(lineSensors), motors(motors),
          encoders(encoders), imu(imu), buzzer(buzzer),
          Asservissement(0.18, 0.0005, 0.25) {};

    void automate();
    
private:
    const unsigned long TIMEOUT_BT = 10000; // 10 secondes
    const unsigned long LINE_LOST_TIMEOUT = 500; // 500 ms

    unsigned long timeoutBluetooth = 0;
    unsigned long lastLineDetectionTime = 0;
    int etat_autom = 0;
    bool lineLost = false;
    int lastError = 0;
    int lastLinePosition = 0;

    Zumo32U4Buzzer *buzzer;
    Zumo32U4LineSensors *lineSensors;
    Zumo32U4Motors *motors;
    Zumo32U4ButtonB *buttonB;
    Zumo32U4Encoders *encoders;
    Zumo32U4IMU *imu;
};



#endif // AUTOMATE_H