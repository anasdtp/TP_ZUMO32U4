#include "Capteurs.h"

void Capteurs::calibrateLineSensors()
{
  // To indicate we are in calibration mode, turn on the yellow LED
  // and print "Line cal" on the display.
  ledYellow(1);

  Serial.println(F("Line cal"));

  for (uint16_t i = 0; i < 400; i++)
  {
    Serial.print(i);
    lineSensors->calibrate();
  }
  Serial.println("");
  ledYellow(0);
}

void Capteurs::printReadingsToSerial()
{
  static char buffer[80];
  sprintf(buffer, "%d %d %d %d %d %d  %d %d %d  %4d %4d %4d %4d %4d\n",
    proxSensors->countsLeftWithLeftLeds(),
    proxSensors->countsLeftWithRightLeds(),
    proxSensors->countsFrontWithLeftLeds(),
    proxSensors->countsFrontWithRightLeds(),
    proxSensors->countsRightWithLeftLeds(),
    proxSensors->countsRightWithRightLeds(),
    proxLeftActive,
    proxFrontActive,
    proxRightActive,
    lineSensorValues[0],
    lineSensorValues[1],
    lineSensorValues[2],
    lineSensorValues[3],
    lineSensorValues[4]
  );
  Serial.print(buffer);
}

ValeurCapteurs Capteurs::readSensors(){
    ValeurCapteurs valeur;
    // Send IR pulses and read the proximity sensors.
    proxSensors->read();

    // Just read the proximity sensors without sending pulses.
    proxLeftActive = proxSensors->readBasicLeft();
    proxFrontActive = proxSensors->readBasicFront();
    proxRightActive = proxSensors->readBasicRight();

    valeur.proximiteDroite = proxSensors->countsRightWithLeftLeds();
    valeur.proximiteGauche = proxSensors->countsLeftWithRightLeds();
    valeur.proximiteAvant = proxSensors->countsFrontWithLeftLeds();

    // Read the line sensors.
    lineSensors->readCalibrated(lineSensorValues);

    valeur.LigneGauche = lineSensorValues[0];
    valeur.LigneAvant = lineSensorValues[1];
    valeur.LigneDroit = lineSensorValues[2];

    // Send the results to the display and to the serial monitor.
    // printReadingsToSerial();
    return valeur;
}

void Capteurs::printValeurCapteurs(ValeurCapteurs valeur){
    Serial.print("valeur.proximiteGauche : ");
    Serial.print(valeur.proximiteDroite);
    Serial.print(" valeur.proximiteDroite : ");
    Serial.print(valeur.proximiteGauche);
    Serial.print(" valeur.proximiteAvant : ");
    Serial.print(valeur.proximiteAvant);
    Serial.print(" valeur.LigneAvant : ");
    Serial.print(valeur.LigneGauche);
    Serial.print(" valeur.LigneGauche : ");
    Serial.print(valeur.LigneAvant);
    Serial.print(" valeur.LigneDroit : ");
    Serial.print(valeur.LigneDroit);
    Serial.println("");
}