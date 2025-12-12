#include "Automate.h"

void Automate::automate(){
       switch(etat_autom){
        case 0 :
                  // ===== ÉTAT 0: Attente état path =====
                  // Mode Bluetooth - contrôle manuel
                  // Condition: bouton B appuyé
                if( buttonB->getSingleDebouncedRelease()){
                      delay(800);
                      etat_autom=1;  // Transition vers État 1
                      buzzer->play("L16 cde");
                      resetPID();
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
                          buzzer->play("L16 c");
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
                  if( buttonB->getSingleDebouncedRelease()){
                      delay(800);
                      etat_autom=0;
                      motors.setSpeeds(0,0);
                      buzzer->play("L16 edcg");
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