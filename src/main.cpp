#include <Arduino.h>
#include <Wire.h>
#include <Zumo32U4.h>
#include <Mouvement.h>

#define TYPE_MOUVEMENT_CARRE 55
#define TYPE_MOUVEMENT_TRIANGLE 66

float Kp = 0.25, Ki = 0.0009, Kd = 0.;
Asservissement *motorD;
Asservissement *motorG;

Mouvement *mvt;

typedef struct FIGURE{
  char type;
  int centimetre;
}FIGURE;

FIGURE fig;

Zumo32U4LCD display;

Zumo32U4IMU imu;

char report[120];


void mvt_ligneDroite(int centimetre);
void mvt_rotation(int degree);

void mvt_figure_loop();
bool Figure(char type_mvt, int centimetre, int degree);
void figure_carre(int centimetre);
void figure_triangle(int centimetre);

bool frontBP(int pin);
void clignotementLeds(bool yellow, bool red, bool green);
void AffichageLed(Position position);
void CentraleInertielle();
void setup() {
  Serial.begin(115200);
  Wire.begin();
  if (!imu.init())
  {
    // Failed to detect the compass.
    ledRed(1);
    while(1)
    {
      Serial.println(F("Failed to initialize IMU sensors."));
      delay(100);
    }
  }

  imu.enableDefault();

  delay(100);
  
  motorD = new Asservissement(Kp, Ki, Kd);
  motorG = new Asservissement(Kp, Ki, Kd);
  mvt = new Mouvement(motorD, motorG);
  
  // ligneDroite(5);
  // mvt_rotation(90);
  // figure_carre(10);
  // figure_triangle(10);
}

void loop() {
  
  mvt->loop();

  mvt_figure_loop();

  clignotementLeds(true, true, true);

  AffichageLed(mvt->getPosition());

  CentraleInertielle();
}

void mvt_ligneDroite(int centimetre){
  float distance = centimetre * 10;//En mm
  mvt->liste.type = TYPE_DEPLACEMENT_LIGNE_DROITE;
  mvt->liste.line.distance = distance;
}

void mvt_rotation(int degree){//en degrés
  mvt->liste.type = TYPE_DEPLACEMENT_ROTATION;
  mvt->liste.Rotation.degree = degree;
}

void mvt_figure_loop(){
  switch (fig.type)
  {
    case TYPE_MOUVEMENT_CARRE:
    {
      if(Figure(TYPE_MOUVEMENT_CARRE, fig.centimetre, 90)){
        fig.type = 0;
      }
    }
    break;
    case TYPE_MOUVEMENT_TRIANGLE:
    {
      if(Figure(TYPE_MOUVEMENT_TRIANGLE, fig.centimetre, int(180. - (180./3.)))){
        fig.type = 0;
      }
    }
    break;
    default:
      break;
  }
}

bool Figure(char type_mvt, int centimetre, int degree){
  static int etat_figure = 0, etatFigure[8] = {1,2,1,2,1,2,1,2}, pointeurEtat = 0, nbEtapes = 8;
  
  switch (etat_figure)
  {
    case 0:
    {
      if(type_mvt == TYPE_MOUVEMENT_TRIANGLE){
        nbEtapes = 6;
      }else{
        nbEtapes = 8;
      }
      etat_figure = etatFigure[0];
    }
    break;
    case 1://Ligne droite
    {
      mvt_ligneDroite(centimetre);
      etat_figure = 3;
    }
    break;
    case 2://Rotation
    {
      mvt_rotation(degree);
      etat_figure = 3;
    }
    break;
    case 3://Attente fin d'un mouvement
    {
      if(mvt->next_action){//Flag indiquant la fin d'un mouvement elementaire
        mvt->next_action = false;

        pointeurEtat ++;
        if(pointeurEtat >=nbEtapes){
          pointeurEtat = 0;
          etat_figure = 0;
          return true;
        }else{
          etat_figure = etatFigure[pointeurEtat];
        }
      }
    }
    break;
    default:
      break;
  }
  return false;
}

void figure_carre(int centimetre){
  fig.type = TYPE_MOUVEMENT_CARRE;
  fig.centimetre = centimetre;
}

void figure_triangle(int centimetre){
  fig.type = TYPE_MOUVEMENT_TRIANGLE;
  fig.centimetre = centimetre;
}

bool frontBP(int pin){
  static int prevVal = 0, init = true, pinBPA = 14;
  if(init){init = false; pinMode(pinBPA, INPUT_PULLUP);}
  
  int BP = digitalRead(pin);
  if(BP != prevVal){
    prevVal = BP;
    if(!BP){
      return true;
    }
  }
  return false;
}

void clignotementLeds(bool yellow, bool red, bool green){
  static int init = true, pinGreen = 30, pinRed = 17, pinYellow = 13, etatLed = true;
  static unsigned long startMs = 0, tempsMs = 200;
  if(init){init = false; pinMode(pinGreen, OUTPUT); pinMode(pinRed, OUTPUT); pinMode(pinYellow, OUTPUT);}

  if((millis() - startMs)>=(tempsMs)){
    startMs = millis();
    etatLed = !etatLed;
    digitalWrite(pinGreen, yellow&(!etatLed)); digitalWrite(pinRed, red&(!etatLed)); digitalWrite(pinYellow, green&etatLed);
  }
}

void AffichageLed(Position position){
  static unsigned long startMs = 0, tempsMs = 500;
  static Position pastPosition;
  if((millis() - startMs)>=(tempsMs)){
    // startMs = millis();
    if(pastPosition.x!=position.x || pastPosition.y!=position.y || pastPosition.theta!=position.theta){
      pastPosition.x = position.x;
      pastPosition.y = position.y;
      pastPosition.theta = position.theta;
        // Clear the screen
      display.clear();
      // Print a string
      display.print("x : ");
      // Print a number
      display.print(int(position.x*100.));
      // Go to the next line
      display.gotoXY(0, 1);
      display.print("y : ");
      // Print a number
      display.print(int(position.y*100.));
      // Go to the next line
      // display.gotoXY(0, 2);
      // display.print("t : ");
      // // Print a number
      // display.print(position.theta);
    }
    // Serial.println(position.x);
    // Serial.println(position.y);
    // Serial.println(position.theta*180./M_PI);//EN degrés
    // Serial.println("");
    startMs = millis();  
  }
}

void CentraleInertielle(){
  static unsigned long startMs = 0, tempsMs = 100;
  if((millis() - startMs)>=(tempsMs)){
    imu.read();//On récupére les données des capteurs
    //On enregistre les données dans des variables
    float accx = imu.a.x* 0.061e-3 * 9.8, 
          accy = imu.a.y* 0.061e-3 * 9.8, 
          accz = imu.a.z* 0.061e-3 * 9.8;
    // Serial.print(accx);
    // Serial.print("m/s² ");
    // Serial.print(accy);
    // Serial.print("m/s² ");
    // Serial.print(accz);
    // Serial.print("m/s² ");//On affiche les données afin de tester le bon fonctionnement
    
    float angle= atan(accx/accz)*180./PI;
    // Serial.print("angle ");
    // Serial.print(angle);
    // Serial.println("° ");

    float GyroX = imu.m.x* 90./10285. , 
          GyroY = imu.m.y* 90./10285., 
          GyroZ = imu.m.z* 90./10285.;

    // Calcul du cap du robot à partir du gyroscope
    float cap = GyroZ/1000. * (millis() / 1000.0); // mise à jour du cap

    Serial.print("GyroX: ");
    Serial.print(GyroX);
    Serial.print(" mDeg/s, GyroY: ");
    Serial.print(GyroY);
    Serial.print(" mDeg/s, GyroZ: ");
    Serial.print(GyroZ);
    Serial.println(" mDeg/s");

    Serial.print("Cap du robot: ");
    Serial.print(cap);
    Serial.println(" degrés");


    startMs = millis();
  }
}