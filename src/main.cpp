/*This demo requires a Zumo 32U4 
*** Configuration :  
	5 capteur de lignes donc vérifier que les 2 cavaliers sur la carte capteurs sont bien surla position interieure
*/

#include <Wire.h>
#include <Zumo32U4.h>
#include "gyro_use.h"
#include <math.h>
#include <Asservissement.h>

// This is the maximum speed the motors will be allowed to turn.
// A maxSpeed of 400 lets the motors go at top speed.  Decrease
// this value to impose a speed limit.
const uint16_t maxSpeed = 400;

Zumo32U4Buzzer buzzer;
Zumo32U4LineSensors lineSensors;
Zumo32U4Motors motors;
Zumo32U4ButtonB buttonB;
Zumo32U4Encoders encoders;
Zumo32U4IMU imu;

Asservissement pid(0.18, 0.0005, 0.25);

// tableau pour récupérer les valeurs des 5 capteurs sol
#define NUM_SENSORS 5
unsigned int lineSensorValues[NUM_SENSORS];

//******** Fonction de rotation suivant le gyro
void turncalibrate(short nbangle45){
	// nbangle45 représente le nombre de fois 45°
	// positif = sens trigo
	int16_t speedgauche = 160*(nbangle45>0 ? -1 :1);
	// lancer la rentation suivant le signe de nbangle45
	// on tourne sur place
	motors.setSpeeds(speedgauche,-speedgauche);
	turnSensorReset(); // initialiser l'angle du gyro "turnAgle"
	int32_t angleC = turnAngle45 *nbangle45;
	while((nbangle45>=0 ? turnAngle < angleC :turnAngle >= angleC )){
			// calibrer les capteurs sol
			  lineSensors.calibrate();
			// mettre à jour le calcul de l'angle gyro
			  turnSensorUpdate();
	}
	// arret moteurs
	motors.setSpeeds(0, 0);
	// dernier mise à jour integration angle Gyro
	turnSensorUpdate();
}

bool isLineOK(unsigned int *p){
	unsigned char i;
	unsigned char sens_stat=0;
	for(i=0;i<5;i++){
		sens_stat=sens_stat<<1;
		// on calcul un seil de décision pour dire que c'est blanc 0 ou noir 1
		// si le capteur est supérier à un seil alors c'est nor donc 1 sinon c'est blan 0
		sens_stat=sens_stat+(p[i]>(lineSensors.calibratedMinimumOn[i]*2+
						lineSensors.calibratedMaximumOn[i])/3 ? 1:0);
		// variable ret va contenir 5 bits LSB utiles
		//  les bits ret  ... b4 b3 b2 b1 b0
		// chaque bit bi représente un capteur (b4 de gauche et b0 celui de droite)
		// bi= 1 alors le capteur a déecté une ligne noire, sinon 0
	} 
  return (((sens_stat&0x0E)!=0) && ((sens_stat&0x11)==0));
  }

// Fonction pour effectuer la rotation 90° à gaucher et à droite 
// de la ligne pour effectuer le calibrage des 5 capteurs sol
void calibrateSensors2() { 
	const uint16_t calibrationSpeed = 160;
	// Wait 0.1 second and then begin automatic sensor calibration
	// by rotating in place to sweep the sensors over the line
	delay(100);
	lineSensors.calibrate();
	// Turn to the left 90 degrees.
	// on remet à chaque fois la variable angle à 0
	turncalibrate(2);  //+90°
	turncalibrate(-4);  // -180
	turncalibrate(2); // +90 pour revenir su rla ligne
}

//*****************************************************
// STEUP initialisations
//****************************************************
void setup() {
  	Serial1.begin(38400);
	Wire.begin(); //I2C 
	buttonB.waitForButton(); // on attend d'appuyer sur le bouton B
	delay(800); //attendre pour ne pas risquer de blaisser le doigt
	lineSensors.initFiveSensors(); //config  5capteurs sol
	if(!imu.init()){  //initialisation de la centrale inertielle
		ledRed(1);  // si erreur allumer led Rouge
		while(1); //on ne va pas plus loin vu qu'il y a erreur
	}
	imu.enableDefault(); //initialiser centrale Imu par défaut
	imu.configureForTurnSensing(); // sensibilité maximale

	turnSensorSetup(); // calcul offset du gyro

	// Play music and wait for it to finish before we start driving.
	buzzer.play("L16 cdegreg4"); //petite musique pour la route
	while(buzzer.isPlaying());
	// encodeurs de roue initialiser en lisant les valeurs
	encoders.getCountsAndResetRight(); 
	encoders.getCountsAndResetLeft();
}

int16_t positionl;
int16_t lastError = 0;
// Fonction PID pour maintenir le robot sur cap constant
// cap initial
void PID_gyro(){
	// captick angle relatif du robot par rapport à 
  // sa position initiale
  turnSensorUpdate(); 
  int32_t error= turnAngle;
	int16_t speedDifference = error/90000 ;
	//+ 6 * (error - lastError);
	lastError = error;

	// Get individual motor speeds.  The sign of speedDifference
	// determines if the robot turns left or right.
	int16_t leftSpeed = (int16_t)250 + speedDifference;
	int16_t rightSpeed = (int16_t)250-(250*4)/100 - speedDifference;

	// Constrain our motor speeds to be between 0 and maxSpeed.
	// One motor will always be turning at maxSpeed, and the other
	// will be at maxSpeed-|speedDifference| if that is positive,
	// else it will be stationary.  For some applications, you
	// might want to allow the motor speed to go negative so that
	// it can spin in reverse.
	leftSpeed = constrain(leftSpeed, 0, (int16_t)maxSpeed);
	rightSpeed = constrain(rightSpeed, 0, (int16_t)maxSpeed);
	// mettre à jour les valeurs de commandes moteurs
	motors.setSpeeds(leftSpeed, rightSpeed);
//  Serial1.print(turnAngle);
//  Serial1.print("   ");
//  Serial1.println(speedDifference);
}

long integral;
void PID(){
  int16_t speedDifference = pid.calculatePID(positionl, 1);

  // Get individual motor speeds.  The sign of speedDifference
  // determines if the robot turns left or right.
  int16_t leftSpeed = (int16_t)maxSpeed + speedDifference;
  int16_t rightSpeed = (int16_t)maxSpeed - speedDifference;

  // Constrain our motor speeds to be between 0 and maxSpeed.
  // One motor will always be turning at maxSpeed, and the other
  // will be at maxSpeed-|speedDifference| if that is positive,
  // else it will be stationary.  For some applications, you
  // might want to allow the motor speed to go negative so that
  // it can spin in reverse.
  leftSpeed = constrain(leftSpeed, -(int16_t)maxSpeed, (int16_t)maxSpeed);
  rightSpeed = constrain(rightSpeed, - (int16_t)maxSpeed, (int16_t)maxSpeed);

  motors.setSpeeds(leftSpeed, rightSpeed);
}

long ll,lastll;
long distdroit,distgauche;
unsigned char autom;
uint16_t incomingByte;
unsigned long timeoutBluetooth = 0;
const unsigned long TIMEOUT_BT = 10000; // 10 secondes timeout Bluetooth

// Variables pour détection perte de ligne
int16_t lastLinePosition = 2000; // Dernière position valide de la ligne
unsigned long lastLineDetectionTime = 0;
const unsigned long LINE_LOST_TIMEOUT = 500; // 500ms sans détection = ligne perdue
boolean lineLost = false;
const int16_t LINE_LOST_THRESHOLD = 2500; // Seuil de perte de ligne (capteurs au maximum)

// Diagramme d'état :
// État 0: Attente état path (en attente du Bluetooth)
// État 1: Suivi de ligne (automatique avec PID ligne)
// État 1b: Ligne perdue - basculer en PID gyro pour retrouver ligne
// État 2: Relay utilisé (mode relais)

void automate(){
       switch(autom){
        case 0 :
                  // ===== ÉTAT 0: Attente état path =====
                  // Mode Bluetooth - contrôle manuel
                  // Condition: bouton B appuyé
                if( buttonB.getSingleDebouncedRelease()){
                      delay(800);
                      autom=1;  // Transition vers État 1
                      buzzer.play("L16 cde");
                      integral=0;
                      lastError=0;
                      lineLost=false;
                      turnSensorReset();
                      Serial1.println("->État 1 (auto)");
                }
                // Condition: 'x' reçu par Bluetooth
                // (géré dans loop)
                
                // Sécurité : si pas de commande Bluetooth depuis 10s, arrêt moteurs
                if(millis() - timeoutBluetooth > TIMEOUT_BT){
                      motors.setSpeeds(0,0);
                }
                
                // Vérifier s'il y a une ligne visible en État 0
                if(!isLineOK(lineSensorValues)){
                      if(millis() - lastLineDetectionTime > LINE_LOST_TIMEOUT){
                          lineLost = true;
                      }
                } else {
                      lastLineDetectionTime = millis();
                      lineLost = false;
                }
                break;
                
        case 1 :
                  // ===== ÉTAT 1: Suivi de ligne (normal) =====
                  // Vérifier si la ligne est visible avec isLineOK
                  if(!isLineOK(lineSensorValues)){
                      // Ligne pas détectée
                      if(millis() - lastLineDetectionTime > LINE_LOST_TIMEOUT){
                          // Timeout dépassé : ligne perdue
                          lineLost = true;
                          buzzer.play("L16 c");
                          Serial1.println("LIGNE PERDUE -> PID gyro");
                      }
                  } else {
                      // Ligne détectée : réinitialiser timer
                      lastLineDetectionTime = millis();
                      lastLinePosition = positionl;
                      lineLost = false;
                  }
                  
                  // Choisir le mode de suivi
                  if(!lineLost){
                      // Ligne visible : utiliser PID classique sur position ligne
                      PID();
                  } else {
                      // Ligne perdue : utiliser PID gyro pour maintenir cap et chercher
                      PID_gyro();
                  }
                  
                  // Condition: bouton B pour revenir en État 0
                  if( buttonB.getSingleDebouncedRelease()){
                      delay(800);
                      autom=0;
                      motors.setSpeeds(0,0);
                      buzzer.play("L16 edcg");
                      Serial1.println("->État 0 (manuel)");
                      break;
                  }
                  
                  // Condition: 's' reçu par Bluetooth (géré dans loop)
                break;

        case 2 :
                  // ===== ÉTAT 2: Relay utilisé =====
                  // Mode relais/passthrough
                  // Vérifier s'il y a une ligne visible en État 2
                  if(!isLineOK(lineSensorValues)){
                      if(millis() - lastLineDetectionTime > LINE_LOST_TIMEOUT){
                          lineLost = true;
                      }
                  } else {
                      lastLineDetectionTime = millis();
                      lineLost = false;
                  }
                break;

        default :
                break;
        
    }
}

// Fonction LOOP exécutée à l'infini
//
void loop(){
// automate gestion bluetooth
//////////////////////////////
    if(Serial1.available()){   // permet de vérifier si un caractère a été reçu par le bluetooth
            incomingByte = Serial1.read();   // Lire ce caractère si c'est bon
            timeoutBluetooth = millis(); // Actualiser le timeout
            //Serial1.println((char)incomingByte);
            switch((unsigned)incomingByte){    // décider quoi faire avec le caractère reçu
                  case 'c': // calibrage des 5 capteurs ligne sol
                            calibrateSensors2();
                            Serial1.println("Calibrage OK");
                            incomingByte=0;
                            break;
                  case 'f': // deplacement vers l'avant (seulement en état 0)
                            if(autom==0){
                              motors.setSpeeds(180,180);
                            }
                            incomingByte=0;
                            break;
                  case 'x' : // Transition État 0 -> État 1 (suivi auto)
                            if(autom==0){
                              autom=1;
                              integral=0;
                              lastError=0;
                              lineLost=false;
                              turnSensorReset(); // Réinitialiser gyro
                              lastLineDetectionTime=millis();
                              buzzer.play("L16 cde");
                              Serial1.println("->État 1 (auto)");
                            }
                            incomingByte=0;
                            break;
                  case 'l': // tourne à gauche (seulement en état 0)
                            if(autom==0){
                              motors.setSpeeds(-160, 160);
                            }
                            incomingByte=0;
                            break;                         
                  case 'r': // tourne à droite (seulement en état 0)
                            if(autom==0){
                              motors.setSpeeds(160, -160);
                            }
                            incomingByte=0;
                            break;
                            
                  case 's': // arrêt - Transition État 1 -> État 0
                            if(autom==1){
                              autom=0;
                              motors.setSpeeds(0,0);
                              buzzer.play("L16 edcg");
                              Serial1.println("->État 0 (manuel)");
                            } else {
                              motors.setSpeeds(0,0);
                            }
                            incomingByte=0;
                            break; 
                  case 'v' : // envoi de la tension batterie et position ligne
                            {
                              unsigned short v=readBatteryMillivolts ();
                              Serial1.print("V:");
                              Serial1.print(v);
                              Serial1.print(" P:");
                              Serial1.print(positionl);
                              Serial1.print(" DR:");
                              Serial1.print(distdroit);
                              Serial1.print(" DG:");
                              Serial1.print(distgauche);
                              if(lineLost){
                                Serial1.println(" [LIGNE PERDUE]");
                              } else {
                                Serial1.println(" [OK]");
                              }
                            }
                            incomingByte=0;
                            break;
                  case 'e' : // arrêt d'urgence
                            motors.setSpeeds(0,0);
                            autom=0;
                            buzzer.play("t200L8 g");
                            Serial1.println("Urgence!");
                            incomingByte=0;
                            break;
                  case 'i' : // info - affiche l'état actuel
                            Serial1.print("État:");
                            Serial1.print(autom);
                            Serial1.print(" Pos:");
                            Serial1.println(positionl);
                            incomingByte=0;
                            break;
                  default : 
                            break;
            }
    }
    
// coeur du système superviseur
// lance et fait les acquisitions capteurs
// appelle l'automate
//////////////////////////////////////////////
  positionl = lineSensors.readLine(lineSensorValues)-2000;  
	ll=micros();
	if((ll-lastll)>=10000){  // lorsque le délai est de 10 ms alors faire                
			lastll=ll;
			// calculer le deplacement des 2 roues
			distdroit = distdroit + encoders.getCountsAndResetRight();
			distgauche = distgauche + encoders.getCountsAndResetLeft();
      automate();
	}

}
