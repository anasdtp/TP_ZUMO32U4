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
  //Serial.begin(9600);
  for(int i = 0; i<4; i++){
    y = y+ (1.44e3/4);
    z = z+ (6.3452e1/4);
    yi = yi+i*5;
    zi = zi+i*12;
  }
 // y=1.34e2;
 // z=3.3452e3;
  buttonB.waitForButton();
  delay(800);
  l1=micros();
  xi=yi+zi;
  l2=micros();
  Serial.println(l2-l1);
  l1=micros();
  xi=yi*zi;
  l2=micros();
  Serial.println(l2-l1);
  l1=micros();
  xi=yi/zi;
  l2=micros();
  Serial.println(l2-l1);
  while(1);
}


void oldsetup() {
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
					PID(); //suivi de ligne noire
			}else{
				// arreter des moteurs donc ne plus avancer
					motors.setSpeeds(0, 0);
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
	const uint16_t calibrationSpeed = 160; // valeur piphométrique de la vitesse de rotation
	// Wait 0.1 second and then begin automatic sensor calibration
	// by rotating in place to sweep the sensors over the line
	delay(100);
	lineSensors.calibrate();
	// Turn to the left 90 degrees.
	// on remet à chaque fois la variable angle à 0
	turncalibrate(2);  //+90° (val multiple de 45°) gauche
	turncalibrate(-4);  // -180 droite
	turncalibrate(2); // +90 pour revenir su rla ligne gauche
}

// Fonction PID pour maintenir le robot sur la ligne
// avec suivi de la ligne
void PID(){
	// Our "error" is how far we are away from the center of the
	// line, which corresponds to position 2000 done il loop function.
	int16_t error = positionl;
	int16_t speedDifference = error / 4 + 6 * (error - lastError);
	lastError = error;

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
	// mettre à jour les valeurs de commandes moteurs
	motors.setSpeeds(leftSpeed, rightSpeed);
}

// analyser les données capteur sol
// retourne un char contenant  5 bitset
// chaque bit représente un capteur
// 0 pas de ligne noire, 1 ligne noire détectée
unsigned char  Analyse(int *p){
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
/*Procedure bluetooth
1- robot off
2- choisir un nom de 5 à 7 caractères max
3- introduire une config au robot 
4- appuyer sur le bouton Blutooth
pendant l'appui la 2 eme personne alume le robot (le branche )
on reste appuyer jusqu'à la led bleu clignote lentement
5- relacher le bouton Bluetooth
