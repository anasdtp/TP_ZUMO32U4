/*This demo requires a Zumo 32U4 waitForButton
*** Configuration :  
	pour programmer le bluetooth
	avant d'allumer le zumo, maintenir le bouton du bluetooth appuyé
	allumer le zumo
	attendez quelques secondes
	appuyez sur le bouton B
	Première musique
	deuxieme musique, c'est bon le blue tooth est programmé
	vous pouvez maintenant travailler
	pas besoin de refaire l'opération
*/

#include <Wire.h>
#include <Zumo32U4.h>

Zumo32U4Buzzer buzzer;
Zumo32U4ButtonB buttonB;


//*****************************************************
// STEUP initialisations
//****************************************************
void setup() {
  Serial1.begin(38400);
  buzzer.play("L16 cdegreg4"); //petite musique pour la route
  while(buzzer.isPlaying());
	buttonB.waitForButton(); // on attend d'appuyer sur le bouton B
	delay(800); //attendre pour ne pas risquer de blaisser le doigt

 Serial1.println("AT+NAME=IA-nes");// mettez votre nom ici
  delay(800);
  Serial1.println("AT+UART=38400,0,0");
 buzzer.play("L16 reg4cdeg"); //petite musique pour la route
  while(buzzer.isPlaying());
}

void loop(){

}
