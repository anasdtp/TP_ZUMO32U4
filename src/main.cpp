/*This demo requires a Zumo 32U4 
*** Configuration :  
	5 capteur de lignes donc vérifier que les 2 cavaliers sur la carte capteurs sont bien surla position interieure
*/

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
//	calibrateSensors2(); // fonction pour calibrer les capteurs sol

	// Play music and wait for it to finish before we start driving.
	buzzer.play("L16 cdegreg4"); //petite musique pour la route
	while(buzzer.isPlaying());
	// encodeurs de roue initialiser en lisant les valeurs
	encoders.getCountsAndResetRight(); 
	encoders.getCountsAndResetLeft();
}

int16_t positionl;
int16_t lastError = 0;
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


uint32_t ll,lastll;
long distdroit,distgauche;
// Fonction LOOP exécutée à l'infini
//
void oldloop(){
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

char motoron;
int32_t dmax;
uint8_t incomingByte;
uint32_t tab[200];
uint16_t indx=0;

void loop(){
	
   if(Serial1.available()){
            incomingByte = Serial1.read();
            Serial1.print(incomingByte,HEX);Serial1.println("**********************");
            switch(incomingByte&0xFF){                  
                  case '1':
                  case '2':
                  case '3':
                        Serial1.println("&&&&&&&&&&&&&&&&&&");
                          dmax= (incomingByte&0x0F)*377;
                            // encodeurs de roue initialiser en lisant les valeurs
                          encoders.getCountsAndResetRight(); 
                          encoders.getCountsAndResetLeft();
                          distdroit=0;
                          distgauche=0;
						  int16_t speed= 150;
                          motors.setSpeeds(speed, speed - (speed*5)/100);
                          motoron=1;
                          break;        
                  case 'p':
                          int v=readBatteryMillivolts();
                          Serial1.println(v); 
                          break;
                  default :
                          break;
               }
    }
  ll=micros();
  if((ll-lastll)>=(uint32_t)100000){  // lorsque le délai est de 10 ms alors faire                  
      lastll=ll;
      // calculer le deplacement des 2 roues
      distdroit = distdroit + encoders.getCountsAndResetRight();
      distgauche = distgauche + encoders.getCountsAndResetLeft();
     // Serial1.print(distgauche);Serial1.print("   ");Serial1.print(distdroit);
      //Serial1.print("   ");Serial1.println(dmax);
      if(motoron==1){
		//Serial1.println(micros());
			if (indx < 200) {
				tab[indx] = micros();
				indx++;
			}
          if((distdroit+distgauche)/2>=dmax){
                 motors.setSpeeds(0, 0);
                 motoron=0;
				 for (indx=0; indx<200; indx++) {
					Serial1.println(micros());
					Serial1.println(tab[indx]);
          		}
      }
	}
  }
}
