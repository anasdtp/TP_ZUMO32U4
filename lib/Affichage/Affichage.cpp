#include "Affichage.h"


void Affichage::afficher(Position position){
    static unsigned long startMs = 0, tempsMs = 500;
  static Position pastPosition; static bool init = true;
  if(init){
    init = false;
    pastPosition.x = -1.;
    pastPosition.y = -1.;
    pastPosition.theta = -1.;
  }
  if((millis() - startMs)>=(tempsMs)){
    // startMs = millis();
    if(pastPosition.x!=position.x 
        || pastPosition.y!=position.y 
        || pastPosition.theta!=position.theta){
          //Pour eviter d'afficher la meme position plusieurs fois
      pastPosition.x = position.x;
      pastPosition.y = position.y;
      pastPosition.theta = position.theta;
      display->clear();
      display->print(int(position.x*100.));//En cm
      display->print(" ");
      display->print(int(position.y*100.));//En cm
      display->gotoXY(0, 1);
      display->print("t: ");
      display->print(int(position.theta*180./PI));//En degres
    }
    startMs = millis();  
  }
}