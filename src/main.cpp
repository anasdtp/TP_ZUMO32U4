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
const uint16_t maxSpeed = 270;

Zumo32U4Buzzer buzzer;
Zumo32U4LineSensors lineSensors;
Zumo32U4ProximitySensors proxSensors;
Zumo32U4Motors motors;
Zumo32U4ButtonB buttonB;
Zumo32U4Encoders encoders;
Zumo32U4IMU imu;

Asservissement pid(0.26, 0.0005, 0.25);
Asservissement pid_gyro(30, 0, 0); // PID pour maintien de cap gyro

// tableau pour récupérer les valeurs des 5 capteurs sol
#define NUM_SENSORS 5
unsigned int lineSensorValues[NUM_SENSORS];

int8_t checkLine(unsigned int *vals);
int8_t updateStateLine();
int8_t stateLine();
void calibrateSensors2();
void turncalibrate(short nbangle45);
//*****************************************************
// STEUP initialisations
//****************************************************
void setup() {
  	Serial1.begin(38400);
	Wire.begin(); //I2C 
	buttonB.waitForButton(); // on attend d'appuyer sur le bouton B
	delay(800); //attendre pour ne pas risquer de blaisser le doigt
	lineSensors.initFiveSensors(); //config  5capteurs sol
	proxSensors.initThreeSensors(); //config 3 capteurs proximité
	if(!imu.init()){  //initialisation de la centrale inertielle
		ledRed(1);  // si erreur allumer led Rouge
		while(1); //on ne va pas plus loin vu qu'il y a erreur
	}
	imu.enableDefault(); //initialiser centrale Imu par défaut
	imu.configureForTurnSensing(); // sensibilité maximale

  delay(500);
	turnSensorSetup(); // calcul offset du gyro

	// Play music and wait for it to finish before we start driving.
	//buzze.play("L16 cdegreg4"); //petite musique pour la route
	// while(buzzer.isPlaying());
	// encodeurs de roue initialiser en lisant les valeurs
	encoders.getCountsAndResetRight(); 
	encoders.getCountsAndResetLeft();
	Serial1.println("->Setup done");
}

int16_t positionl;
int32_t gyroRefAngle = 0; // Angle du gyro mémorisé quand la ligne est perdue
unsigned long gyroModeStartTime = 0; // Timer pour mode gyro

// Variables pour détection d'obstacles
const uint8_t PROXIMITY_THRESHOLD = 1; // Seuil de détection d'obstacle
const uint16_t REVERSE_SPEED = 150; // Vitesse de recul
const uint16_t REVERSE_TIME = 500; // Temps de recul en ms (500ms)
bool obstacleDetected = false;
unsigned long reverseStartTime = 0;

// Fonction PID pour maintenir le robot sur cap constant avec gyroscope
void PID_gyro(){
	static unsigned long lastPIDGyroTime = millis();
	
	// Calculer l'erreur d'angle (en unités gyro)
	int32_t error = turnAngle - gyroRefAngle;
	
	// Convertir l'erreur en degrés pour le PID : error / turnAngle1 = degrés
	float errorDegrees = (float)error / (float)turnAngle1;
	
	// Calculer la correction avec le PID gyro
	int16_t speedDifference = (int16_t)pid_gyro.calculatePID(errorDegrees, (millis() - lastPIDGyroTime) / 1000.0);
	lastPIDGyroTime = millis();
	
	// Vitesse de base réduite en mode récupération pour meilleure stabilité
	const int16_t baseSpeed = 200;
	int16_t leftSpeed = baseSpeed + speedDifference;
	int16_t rightSpeed = baseSpeed - speedDifference;
	
	// Contraindre les vitesses
	leftSpeed = constrain(leftSpeed, -(int16_t)maxSpeed, (int16_t)maxSpeed);
	rightSpeed = constrain(rightSpeed, -(int16_t)maxSpeed, (int16_t)maxSpeed);
	
	motors.setSpeeds(leftSpeed, rightSpeed);
}

long integral;
void PID(){
	static int16_t speedDifference = 0;

	speedDifference = pid.calculatePID(positionl, 1);

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
  leftSpeed = constrain(leftSpeed, 0, (int16_t)maxSpeed);
  rightSpeed = constrain(rightSpeed, 0, (int16_t)maxSpeed);

  motors.setSpeeds(leftSpeed, rightSpeed);
}

long ll,lastll; 
long distdroit,distgauche;
unsigned char autom = 0, etat_ligne = 1;
int8_t searchDirection = 1; // Direction de recherche : 1=droite, -1=gauche
unsigned long searchStartTime = 0;
uint16_t incomingByte;
unsigned long timeoutBluetooth = 0;
const unsigned long TIMEOUT_BT = 10000; // 10 secondes timeout Bluetooth

// Variables pour détection perte de ligne
int16_t lastLinePosition = 2000; // Dernière position valide de la ligne
unsigned long lastLineDetectionTime = 0;
const unsigned long LINE_LOST_TIMEOUT = 3000; // 3000ms sans détection = ligne perdue
const unsigned long LINE_SEARCH_TIMEOUT = 2000; // 2s max pour recherche active
const int16_t LINE_LOST_THRESHOLD = 2500; // Seuil de perte de ligne (capteurs au maximum)

void suivieLigne(){
	// Filtre sur trois mesures pour éviter les faux positifs
	// static uint8_t proxFront = 0, proxFrontPrev = 0;
	
	// // Vérifier les capteurs de proximité pour détecter un obstacle
	// proxSensors.read();
	// // uint8_t proxLeft = proxSensors.countsLeftWithLeftLeds();
	// proxFrontPrev = proxFront;
	// proxFront = proxSensors.countsFrontWithLeftLeds() + proxSensors.countsFrontWithRightLeds();
	// // uint8_t proxRight = proxSensors.countsRightWithRightLeds();
	
	// // Si obstacle détecté devant
	// if(proxFront >= PROXIMITY_THRESHOLD && proxFrontPrev >= PROXIMITY_THRESHOLD)
	// {
	// 	if(!obstacleDetected){
	// 		// Obstacle vient d'être détecté
	// 		obstacleDetected = true;
	// 		reverseStartTime = millis();
	// 		Serial1.println("->Obstacle détecté! Arrêt et recul");
	// 	}
		
	// 	// Reculer pendant REVERSE_TIME ms
	// 	if(millis() - reverseStartTime < REVERSE_TIME){
	// 		motors.setSpeeds(-REVERSE_SPEED, -REVERSE_SPEED);
	// 	} else {
	// 		// Après le recul, arrêter et retourner à l'état manuel (autom 0)
	// 		motors.setSpeeds(0, 0);
	// 		autom = 0;
	// 		etat_ligne = 1;
	// 		pid.resetPID();
	// 		pid_gyro.resetPID();
	// 		obstacleDetected = false;
	// 		Serial1.println("->Retour état 0 (manuel)");
	// 	}
	// 	return; // Ne pas continuer le suivi de ligne
	// }
	
	obstacleDetected = false; // Réinitialiser si pas d'obstacle
	
	switch(etat_ligne){
		case 1: {// SUIVI DE LIGNE NORMAL
			PID();
			
			int8_t ligne = stateLine();
			if(ligne == 0){
				// Ligne perdue - passer en mode maintien de cap gyro
				etat_ligne = 0;
				lastLineDetectionTime = millis();
				gyroModeStartTime = millis();
				gyroRefAngle = turnAngle; // Mémoriser l'angle actuel
				pid_gyro.resetPID();
				lastLinePosition = positionl;
				Serial1.println("->Ligne perdue, mode gyro");
			}
		}
			break;

		case 0: // LIGNE PERDUE - MAINTIEN DE CAP GYRO
			{
				PID_gyro();
				int8_t ligne = checkLine(lineSensorValues);
				if(ligne == 1){
					// Ligne retrouvée - retour au suivi normal
					etat_ligne = 1;
					pid.resetPID();
					//buzze.play("L16 c"); // Signal sonore de récupération
					Serial1.println("->Ligne retrouvée");
				}
				// else if(millis() - gyroModeStartTime > LINE_LOST_TIMEOUT){
				// 	// Timeout maintien cap - passer en recherche active
				// 	etat_ligne = 2;
				// 	searchStartTime = millis();
				// 	searchDirection = (lastLinePosition > 0) ? 1 : -1; // Chercher du côté où était la ligne
				// 	gyroRefAngle = turnAngle; // Nouvelle référence pour balayage
				// }
			}
			break;

		case 2: {// RECHERCHE ACTIVE DE LA LIGNE
			int8_t ligne = checkLine(lineSensorValues);
			if(ligne == 1){
				// Ligne retrouvée - retour au suivi
				etat_ligne = 1;
				pid.resetPID();
				//buzze.play("L16 cde"); // Signal de succès
			}
			else if(millis() - searchStartTime > LINE_SEARCH_TIMEOUT){
				// Timeout recherche - arrêt et retour mode manuel
				autom = 2;
				etat_ligne = 1;
				pid.resetPID();
				pid_gyro.resetPID();
				//buzze.play("L16 g"); // Signal d'échec
			}
			else{
				// Balayage oscillant pour retrouver la ligne
				unsigned long searchTime = millis() - searchStartTime;
				float oscillation = sin(searchTime / 300.0) * 30.0; // Oscillation ±30°
				int32_t targetAngle = gyroRefAngle + (int32_t)(oscillation * turnAngle1) * searchDirection;
				
				int32_t error = turnAngle - targetAngle;
				float errorDegrees = (float)error / (float)turnAngle1;
				
				int16_t speedDiff = (int16_t)(errorDegrees * 8.0); // Gain simple pour oscillation
				int16_t leftSpeed = 150 + speedDiff;
				int16_t rightSpeed = 150 - speedDiff;
				
				leftSpeed = constrain(leftSpeed, -200, 200);
				rightSpeed = constrain(rightSpeed, -200, 200);
				
				motors.setSpeeds(leftSpeed, rightSpeed);
			}
		}
			break;

		default:
			motors.setSpeeds(0,0);
			break;
	}
}

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
					  etat_ligne = 1;
					  pid.resetPID();
					  pid_gyro.resetPID();
                      //buzze.play("L16 cde");
                      turnSensorReset();
                      lastLineDetectionTime = millis(); // Initialiser timer au démarrage
                      Serial1.println("->État 1 (auto)");
                }
                // Condition: 'x' reçu par Bluetooth
                // (géré dans loop)
                
                // Sécurité : si pas de commande Bluetooth depuis 10s, arrêt moteurs
                if(millis() - timeoutBluetooth > TIMEOUT_BT){
                      motors.setSpeeds(0,0);
                }
                break;
                
        case 1 :{
                  // ===== ÉTAT 1: Suivi de ligne (auto) =====
                    if(checkLine(lineSensorValues) == 3){// tous les capteurs détectent du noir
						autom = 2; // on s'arrete
					}
                    else{
						suivieLigne();
					}
                  
                  // Condition: bouton B pour revenir en État 0
                  if( buttonB.getSingleDebouncedRelease()){
                      delay(800);
                      autom=0;
					  pid.resetPID();
					  pid_gyro.resetPID();
                      motors.setSpeeds(0,0);
                      //buzze.play("L16 edcg");
                      Serial1.println("->État 0 (manuel)");
                      break;
                  }
				}
                  // Condition: 's' reçu par Bluetooth (géré dans loop)
                break;

        case 2 :
                  //On est arrivés à la fin. On arrete les moteurs et on est pret a repartir
                  motors.setSpeeds(0,0);
				  pid.resetPID();
				  pid_gyro.resetPID();
				  etat_ligne = 1;
				  //buzze.play("L16 edcg");
				  autom=0;
				  Serial1.println("->État 0 (manuel)");
                break;

        default :
                break;
    }
}

// Fonction LOOP exécutée à l'infini
//
void loop(){
  // IMPORTANT : Mise à jour gyro à CHAQUE cycle pour précision
  turnSensorUpdate();
  
  positionl = lineSensors.readLine(lineSensorValues)-2000;
  updateStateLine();
// automate gestion bluetooth
//////////////////////////////
    if(Serial1.available()){   // permet de vérifier si un caractère a été reçu par le bluetooth
            incomingByte = Serial1.read();   // Lire ce caractère si c'est bon
            timeoutBluetooth = millis(); // Actualiser le timeout
            //Serial1.println((char)incomingByte);
            switch((unsigned)incomingByte){    // décider quoi faire avec le caractère reçu
                  case 'c': // calibrage des 5 capteurs ligne sol
                            calibrateSensors2();
                            incomingByte=0;
                            break;
                  case 'f': // deplacement vers l'avant (seulement en état 0)
                            if(autom==0){
                              motors.setSpeeds(180,180);
                            }
                            incomingByte=0;
                            break;
                  case 'x' : // Transition État 0 -> État 1 (suivi auto)
                            if(autom==0 && checkLine(lineSensorValues)){
                              autom=1;
                              etat_ligne=1;
                              pid.resetPID();
                              pid_gyro.resetPID();
                              turnSensorReset(); // Réinitialiser gyro
                              lastLineDetectionTime=millis();
                              //buzze.play("L16 cde");
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
                              etat_ligne=1;
                              pid.resetPID();
                              pid_gyro.resetPID();
                              motors.setSpeeds(0,0);
                              //buzze.play("L16 edcg");
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
                            }
                            incomingByte=0;
                            break;
                  case 'e' : // arrêt d'urgence
                            motors.setSpeeds(0,0);
                            autom=0;
                            //buzze.play("t200L8 g");
                            Serial1.println("Urgence!");
                            incomingByte=0;
                            break;
                  case 'i' : // info - affiche l'état actuel
                            Serial1.print("État:");
                            Serial1.print(autom);
                            Serial1.print(" Ligne:");
                            Serial1.print(etat_ligne);
                            Serial1.print(" Pos:");
                            Serial1.print(positionl);
                            Serial1.print(" Gyro:");
                            Serial1.println((float)turnAngle / (float)turnAngle1);
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
    
	ll=micros();
	if((ll-lastll)>=10000){  // lorsque le délai est de 10 ms alors faire                
			lastll=ll;
			// calculer le deplacement des 2 roues
			distdroit = distdroit + encoders.getCountsAndResetRight();
			distgauche = distgauche + encoders.getCountsAndResetLeft();
      automate();
	}
}


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

// Retourne l'état de la ligne détectée
// 0 = pas de ligne (tous les capteurs captent du blanc)
// 1 = ligne OK (capteurs centraux détectent du noir)
// 3 = tous les capteurs détectent du noir
// Historique réduit à 2 échantillons pour meilleure réactivité (20ms au lieu de 30ms)
int8_t valeurLine[2] = {1,1}; 
int8_t indexLine = 0;

int8_t updateStateLine(){
	unsigned char i;
	unsigned char sens_stat = 0;

	// Calculer sens_stat à partir des capteurs
	for(i = 0; i < 5; i++){
		sens_stat = sens_stat << 1;
		sens_stat = sens_stat + (lineSensorValues[i] > (lineSensors.calibratedMinimumOn[i]*2 + lineSensors.calibratedMaximumOn[i])/3 ? 1 : 0);
	}
	
	// Déterminer l'état de la ligne
	int8_t line;
	if(sens_stat == 0x1F) {
		line = 3; // tous les capteurs détectent du noir
	} else {
		line = (((sens_stat & 0x0E) != 0) && ((sens_stat & 0x11) == 0)) ? 1 : 0;
	}
	
	// Appliquer le filtrage avec historique réduit (2 échantillons)
	valeurLine[indexLine] = line ? 1 : 0;
	indexLine = (indexLine + 1) % 2;

	if (line == 3){
		return 3;
	}
	// Ligne détectée si les 2 derniers échantillons sont positifs
	return (valeurLine[0] + valeurLine[1]) >= 2 ? 1 : 0;
}

int8_t stateLine(){
	return (valeurLine[0] + valeurLine[1]) >= 2 ? 1 : 0;
}

int8_t checkLine(unsigned int *vals){
	// Wrapper pour compatibilité avec le reste du code
	unsigned char i;
	unsigned char sens_stat = 0;
	for(i = 0; i < 5; i++){
		sens_stat = sens_stat << 1;
		sens_stat = sens_stat + (vals[i] > (lineSensors.calibratedMinimumOn[i]*2 + lineSensors.calibratedMaximumOn[i])/3 ? 1 : 0);
	}
	if((sens_stat & 0x1F) == 0x1F) {return 3;}
	return (((sens_stat & 0x0E) != 0) && ((sens_stat & 0x11) == 0));
}



// Fonction pour effectuer la rotation 90° à gaucher et à droite 
// de la ligne pour effectuer le calibrage des 5 capteurs sol
void calibrateSensors2() { 
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
