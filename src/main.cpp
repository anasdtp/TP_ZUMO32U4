/*This demo requires a Zumo 32U4 
*** Configuration :  
	5 capteur de lignes donc vérifier que les 2 cavaliers sur la carte capteurs sont bien surla position interieure
*/
#include <Arduino.h>
#include <Wire.h>
#include <Zumo32U4.h>
#include "gyro_use.h"
#include <math.h>

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


// tableau pour récupérer les valeurs des 5 capteurs sol
#define NUM_SENSORS 5
unsigned int lineSensorValues[NUM_SENSORS];


//*****************************************************
// STEUP initialisations
//****************************************************

unsigned long l1,l2;
float x,y,z;
int xi, yi, zi;
double xd, yd, zd;

void turncalibrate(short nbangle45);
void calibrateSensors2();
void PID();
unsigned char  Analyse(int *p);

void setup() {
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
	calibrateSensors2(); // fonction pour calibrer les capteurs sol

	// Play music and wait for it to finish before we start driving.
	buzzer.play("L16 cdegreg4"); //petite musique pour la route
	while(buzzer.isPlaying());
	// encodeurs de roue initialiser en lisant les valeurs
	encoders.getCountsAndResetRight(); 
	encoders.getCountsAndResetLeft();
}

int16_t positionl;
int16_t lastError = 0;
float integralError = 0;  // Somme cumulée des erreurs (terme intégral)
const float MAX_INTEGRAL = 5000.0;  // Limite anti-windup pour l'intégrale

// Paramètres PID
float Kp = 0.25;  // Gain proportionnel (ajustable)
float Ki = 0.0;   // Gain intégral (ajustable)
float Kd = 6.0;   // Gain dérivé (ajustable)

long ll,lastll;
long distdroit,distgauche;
// Fonction LOOP exécutée à l'infini
//
void loop(){
	// Positionl nous done la osition du robot par rapport à la ligne noire au sol
	positionl = lineSensors.readLine(lineSensorValues)-2000;
	// extraire l'état des 5 capteurs sur 5 bits (1 il y a la piste noire sinon 0
	char sens_stat=Analyse(lineSensorValues);
	ll=micros();
	if((ll-lastll)>=10000){  // lorsque le délai est de 10 ms alors faire                  
			lastll=ll;
			// calculer le deplacement des 2 roues
			distdroit = distdroit + encoders.getCountsAndResetRight();
			distgauche = distgauche + encoders.getCountsAndResetLeft();

			if(((sens_stat&0x0E)!=0) && ((sens_stat&0x11)==0)){
					PID(); //suivi de ligne noire avec PID complet
			}else{
				// Ligne perdue - arrêter les moteurs et réinitialiser l'intégrale
				motors.setSpeeds(0, 0);
				integralError = 0;  // Reset de l'intégrale pour éviter l'accumulation
			}
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

// Fonction PID complète pour maintenir le robot sur la ligne
// Implémentation d'un vrai contrôleur PID avec P + I + D
void PID(){
	// L'erreur est la distance par rapport au centre de la ligne
	// position 2000 = centré, donc erreur = positionl (déjà décalé de 2000 dans loop)
	int16_t error = positionl;
	
	// ===== TERME PROPORTIONNEL (P) =====
	// Réagit à l'erreur actuelle
	float proportional = Kp * error;
	
	// ===== TERME INTÉGRAL (I) =====
	// Accumule les erreurs passées pour corriger les biais systématiques
	integralError += error;
	
	// Anti-windup: limiter l'accumulation de l'intégrale
	if (integralError > MAX_INTEGRAL) {
		integralError = MAX_INTEGRAL;
	} else if (integralError < -MAX_INTEGRAL) {
		integralError = -MAX_INTEGRAL;
	}
	
	float integral = Ki * integralError;
	
	// ===== TERME DÉRIVÉ (D) =====
	// Réagit au taux de changement de l'erreur (anticipation)
	int16_t derivative_error = error - lastError;
	float derivative = Kd * derivative_error;
	
	// ===== CALCUL DE LA COMMANDE PID =====
	// Correction totale = P + I + D
	float pidOutput = proportional + integral + derivative;
	
	// La différence de vitesse détermine la correction de trajectoire
	int16_t speedDifference = (int16_t)pidOutput;
	
	// Sauvegarder l'erreur pour le prochain cycle
	lastError = error;
	
	// ===== APPLICATION AUX MOTEURS =====
	// Vitesse de base pour les deux moteurs
	// Le signe de speedDifference détermine si on tourne à gauche ou à droite
	int16_t leftSpeed = (int16_t)maxSpeed + speedDifference;
	int16_t rightSpeed = (int16_t)maxSpeed - speedDifference;
	
	// Contraindre les vitesses entre 0 et maxSpeed
	// Pour un suivi plus agressif, on pourrait permettre des vitesses négatives
	// leftSpeed = constrain(leftSpeed, -maxSpeed, (int16_t)maxSpeed);
	// rightSpeed = constrain(rightSpeed, -maxSpeed, (int16_t)maxSpeed);
	leftSpeed = constrain(leftSpeed, 0, (int16_t)maxSpeed);
	rightSpeed = constrain(rightSpeed, 0, (int16_t)maxSpeed);
	
	// Appliquer les commandes aux moteurs
	motors.setSpeeds(leftSpeed, rightSpeed);
}

// analyser les données capteur sol
// retourne un char contenant  5 bitset
// chaque bit représente un capteur
// 0 pas de ligne noire, 1 ligne noire détectée
unsigned char  Analyse(unsigned int *p){
	unsigned char i;
	unsigned char ret=0;
	for(i=0;i<5;i++){
		ret=ret<<1;
		// on calcul un seil de décision pour dire que c'est blanc 0 ou noir 1
		// si le capteur est supérier à un seil alors c'est nor donc 1 sinon c'est blan 0
		ret=ret+(p[i]>(lineSensors.calibratedMinimumOn[i]*2+
						lineSensors.calibratedMaximumOn[i])/3 ? 1:0);
		// variable ret va contenir 5 bits LSB utiles
		//  les bits ret  ... b4 b3 b2 b1 b0
		// chaque bit bi représente un capteur (b4 de gauche et b0 celui de droite)
		// bi= 1 alors le capteur a déecté une ligne noire, sinon 0
	} 
	return ret;      
}