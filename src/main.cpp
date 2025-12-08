#include <Arduino.h>
#include <Wire.h>
#include <Zumo32U4.h>
#include <GyroUse.h>
#include <math.h>

// This is the maximum speed the motors will be allowed to turn.
// A maxSpeed of 400 lets the motors go at top speed.  Decrease
// this value to impose a speed limit.
const uint16_t maxSpeed = 400;

#define NUM_SENSORS 5

// Structure pour encapsuler les variables de contrôle
struct RobotController {
    // Create GyroUse instance with IMU reference
    Zumo32U4IMU imu;
    GyroUse gyroSensor;

    // Other Zumo objects
    Zumo32U4Buzzer buzzer;
    Zumo32U4LineSensors lineSensors;
    Zumo32U4Motors motors;
    Zumo32U4ButtonB buttonB;
    Zumo32U4Encoders encoders;

    // Variables de contrôle PID et recherche
    int16_t positionl;
    int16_t lastError;
    int16_t lastValidError;  // Dernière erreur valide pour la recherche
    unsigned long searchStartTime;
    bool isSearching;

    // Variables de distance
    long distdroit, distgauche;
    long ll, lastll;

    // tableau pour récupérer les valeurs des 5 capteurs sol
    unsigned int lineSensorValues[NUM_SENSORS];

    // Constructeur
    RobotController() : gyroSensor(imu), positionl(0), lastError(0), lastValidError(0), 
                       searchStartTime(0), isSearching(false), distdroit(0), distgauche(0), 
                       ll(0), lastll(0) {}
};

// Instance globale unique du contrôleur
RobotController robot;

//******** Fonction de rotation suivant le gyro
void turncalibrate(short nbangle45){
	// nbangle45 représente le nombre de fois 45°
	// positif = sens trigo
	int16_t speedgauche = 160*(nbangle45>0 ? -1 :1);
	// lancer la rentation suivant le signe de nbangle45
	// on tourne sur place
	robot.motors.setSpeeds(speedgauche,-speedgauche);
	robot.gyroSensor.reset(); // initialiser l'angle du gyro "turnAgle"
	int32_t angleC = GyroUse::turnAngle45 * nbangle45;
	unsigned long timeout = millis() + 3000; // Timeout de 3 secondes
	while((nbangle45>=0 ? robot.gyroSensor.getTurnAngle() < angleC : robot.gyroSensor.getTurnAngle() > angleC) && millis() < timeout){
			// calibrer les capteurs sol
			  robot.lineSensors.calibrate();
			// mettre à jour le calcul de l'angle gyro
			  robot.gyroSensor.update();
			// delay(1); // Petit délai pour éviter une boucle trop rapide
	}
	// arret moteurs
	robot.motors.setSpeeds(0, 0);
	// dernier mise à jour integration angle Gyro
	robot.gyroSensor.update();
}

// Fonction pour effectuer la rotation 90° à gaucher et à droite 
// de la ligne pour effectuer le calibrage des 5 capteurs sol
void calibrateSensors2() { 
	const uint16_t calibrationSpeed = 160;
	// Wait 0.1 second and then begin automatic sensor calibration
	// by rotating in place to sweep the sensors over the line
	delay(100);
	robot.lineSensors.calibrate();
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
  Serial.begin(115200);
	Wire.begin(); //I2C 
	robot.buttonB.waitForButton(); // on attend d'appuyer sur le bouton B
  Serial.println("ZUMO32U4 LINE FOLLOWER WITH GYRO");

	delay(800); //attendre pour ne pas risquer de blaisser le doigt
	robot.lineSensors.initFiveSensors(); //config  5capteurs sol
	if(!robot.imu.init()){  //initialisation de la centrale inertielle
		ledRed(1);  // si erreur allumer led Rouge
    Serial.println("IMU init failed");
		while(1); //on ne va pas plus loin vu qu'il y a erreur
	}
	robot.imu.enableDefault(); //initialiser centrale Imu par défaut
	robot.imu.configureForTurnSensing(); // sensibilité maximale

	robot.gyroSensor.setup(); // calcul offset du gyro
	calibrateSensors2(); // fonction pour calibrer les capteurs sol

	// Play music and wait for it to finish before we start driving.
	// buzzer.play("L16 cdegreg4"); //petite musique pour la route
	// while(buzzer.isPlaying());

	// encodeurs de roue initialiser en lisant les valeurs
	robot.encoders.getCountsAndResetRight(); 
	robot.encoders.getCountsAndResetLeft();
  Serial.println("Setup complete, starting line following...");
}


// Fonction PID pour maintenir le robot sur la ligne
// avec suivi de la ligne
void PID(){
	// Our "error" is how far we are away from the center of the
	// line, which corresponds to position 2000 done il loop function.
	int16_t error = robot.positionl;
	int16_t speedDifference = error / 4 + 6 * (error - robot.lastError);
	robot.lastError = error;

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
	robot.motors.setSpeeds(leftSpeed, rightSpeed);
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
		ret=ret+(p[i]>(robot.lineSensors.calibratedMinimumOn[i]*2+
						robot.lineSensors.calibratedMaximumOn[i])/3 ? 1:0);
		// variable ret va contenir 5 bits LSB utiles
		//  les bits ret  ... b4 b3 b2 b1 b0
		// chaque bit bi représente un capteur (b4 de gauche et b0 celui de droite)
		// bi= 1 alors le capteur a déecté une ligne noire, sinon 0
	} 
	return ret;      
}



// Fonction LOOP exécutée à l'infini
//
void loop(){
	// Mettre à jour les données du gyroscope et accéléromètre
	robot.gyroSensor.update();
	
	// Positionl nous done la osition du robot par rapport à la ligne noire au sol
	robot.positionl = robot.lineSensors.readLine(robot.lineSensorValues)-2000;
	// extraire l'état des 5 capteurs sur 5 bits (1 il y a la piste noire sinon 0
	char sens_stat=Analyse(robot.lineSensorValues);
	robot.ll=micros();
	if((robot.ll-robot.lastll)>=10000){  // lorsque le délai est de 10 ms alors faire                  
			robot.lastll=robot.ll;
			// calculer le deplacement des 2 roues
			robot.distdroit = robot.distdroit + robot.encoders.getCountsAndResetRight();
			robot.distgauche = robot.distgauche + robot.encoders.getCountsAndResetLeft();

			if(((sens_stat&0x0E)!=0) && ((sens_stat&0x11)==0)){
					PID(); //suivi de ligne noire
					robot.lastValidError = robot.positionl; // Sauvegarder la dernière position valide
					robot.isSearching = false;
			}else{
				// Recherche intelligente de la ligne basée sur la position connue
				if(!robot.isSearching){
					Serial.println("Searching for line...");
					Serial.print("Last known position: X=");
					Serial.print(robot.gyroSensor.getPositionX());
					Serial.print("mm, Y=");
					Serial.print(robot.gyroSensor.getPositionY());
					Serial.println("mm");
					robot.searchStartTime = millis();
					robot.isSearching = true;
				}
				
				unsigned long searchTime = millis() - robot.searchStartTime;
				
				// Mettre à jour la position en temps réel
				robot.gyroSensor.update();
				
				if(searchTime < 800){
					// Phase 1: Navigation intelligente vers la dernière position connue
					int16_t searchSpeed = maxSpeed/3;
					
					// Calculer la direction vers la dernière position valide de ligne
					float currentX = robot.gyroSensor.getPositionX();
					float currentY = robot.gyroSensor.getPositionY();
					
					// Si on a une erreur significative, utiliser la direction de l'erreur
					if(abs(robot.lastValidError) > 50){
						if(robot.lastValidError > 100){
							// La ligne était à droite, tourner à droite
							robot.motors.setSpeeds(searchSpeed, searchSpeed/4);
							Serial.println("Searching right based on last error");
						}else if(robot.lastValidError < -100){
							// La ligne était à gauche, tourner à gauche  
							robot.motors.setSpeeds(searchSpeed/4, searchSpeed);
							Serial.println("Searching left based on last error");
						}else{
							// Avancer tout droit
							robot.motors.setSpeeds(searchSpeed, searchSpeed);
							Serial.println("Searching straight");
						}
					}else{
						// Si pas d'erreur significative, utiliser la vitesse pour déterminer la direction
						float velX = robot.gyroSensor.getVelocityX();
						float velY = robot.gyroSensor.getVelocityY();
						
						if(abs(velX) > abs(velY)){
							// Plus de mouvement en X, probablement en virage
							if(velX > 0){
								robot.motors.setSpeeds(searchSpeed/4, searchSpeed); // Tourner à gauche
								Serial.println("Searching left based on velocity");
							}else{
								robot.motors.setSpeeds(searchSpeed, searchSpeed/4); // Tourner à droite
								Serial.println("Searching right based on velocity");
							}
						}else{
							robot.motors.setSpeeds(searchSpeed, searchSpeed); // Avancer
							Serial.println("Searching forward based on velocity");
						}
					}
				}else if(searchTime < 2000){
					// Phase 2: Balayage en spirale basé sur la position
					int16_t turnSpeed = maxSpeed/4;
					int16_t searchSpeed = maxSpeed/3;
					
					// Créer un mouvement en spirale pour explorer la zone
					if(((searchTime/150) % 4) == 0){
						// Tourner à gauche
						robot.motors.setSpeeds(-turnSpeed, turnSpeed);
						Serial.println("Spiral search: turning left");
					}else if(((searchTime/150) % 4) == 1){
						// Avancer
						robot.motors.setSpeeds(searchSpeed/2, searchSpeed/2);
						Serial.println("Spiral search: moving forward");
					}else if(((searchTime/150) % 4) == 2){
						// Tourner à droite
						robot.motors.setSpeeds(turnSpeed, -turnSpeed);
						Serial.println("Spiral search: turning right");
					}else{
						// Avancer plus loin
						robot.motors.setSpeeds(searchSpeed/2, searchSpeed/2);
						Serial.println("Spiral search: extending forward");
					}
				}else if(searchTime < 3500){
					// Phase 3: Retour vers le point de départ
					float currentX = robot.gyroSensor.getPositionX();
					float currentY = robot.gyroSensor.getPositionY();
					float distance = sqrt(currentX*currentX + currentY*currentY);
					
					if(distance > 50.0f){ // Si on est à plus de 5cm du départ
						// Calculer l'angle pour revenir au départ
						float targetAngle = atan2(-currentY, -currentX);
						float currentAngle = robot.gyroSensor.getOrientationRadians();
						float angleDiff = targetAngle - currentAngle;
						
						// Normaliser l'angle entre -PI et PI
						while(angleDiff > M_PI) angleDiff -= 2*M_PI;
						while(angleDiff < -M_PI) angleDiff += 2*M_PI;
						
						int16_t turnSpeed = maxSpeed/4;
						if(abs(angleDiff) > 0.2f){ // Plus de 11 degrés
							if(angleDiff > 0){
								robot.motors.setSpeeds(-turnSpeed, turnSpeed); // Tourner à gauche
							}else{
								robot.motors.setSpeeds(turnSpeed, -turnSpeed); // Tourner à droite
							}
							Serial.println("Returning to start: adjusting orientation");
						}else{
							int16_t searchSpeed = maxSpeed/3;
							robot.motors.setSpeeds(searchSpeed/2, searchSpeed/2); // Avancer vers le départ
							Serial.println("Returning to start: moving forward");
						}
					}else{
						robot.motors.setSpeeds(0, 0); // Arrêt au point de départ
						Serial.println("Back at starting point");
					}
				}else{
					// Phase 4: Reset et recommencer
					robot.motors.setSpeeds(-maxSpeed/4, -maxSpeed/4);
					if(searchTime > 4000){
						Serial.println("Restarting search cycle - resetting position");
						robot.gyroSensor.resetPosition(); // Réinitialiser la position de référence
						robot.searchStartTime = millis(); // Recommencer le cycle
					}
				}
			}
	}
}
