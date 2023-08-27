/**
 * 
 * test simple pour une carte WB55
 * le bus UART Feetech est connecté à l'UART 1 du stm32 en mode half duplex
 * 
*/

#include <HardwareSerial.h>
#include "STSServoDriver.h"
#include "FTservoMove.h"

#define NB_PAS_DEMI_TOUR 4540
#define NB_PAS_1M 10000

#define INDEX_MOTEUR_DROITE   1
#define INDEX_MOTEUR_GAUCHE   2

//Définit 
HardwareSerial COMserie (PA_9);
STSServoDriver busFTUART;


FTServoMove moteurDroite(&busFTUART,INDEX_MOTEUR_DROITE);
FTServoMove moteurGauche(&busFTUART,INDEX_MOTEUR_GAUCHE);

void setup() {
  COMserie.begin(500000);
  //Serial.begin(115200);
  //while(!Serial);
  //Serial.println("début setup...");
  busFTUART.init(NC, &COMserie);
  //Serial.println("init ftservomove ...");
  moteurDroite.init(4000 , 40, 2070);
  moteurGauche.init(4000 , 40, 2070);
  //Serial.println("fin setup");
}

void attenteFinDeplacement() {
  while (
    moteurDroite.estIlEnRoute() == true &&
    moteurGauche.estIlEnRoute() == true)
  {
    delay(100);
  }
}

void loop() {

  //moteurDroite.Affinage_Deceleration();
  //moteurGauche.Affinage_Deceleration();  

  moteurDroite.parcoursCetteDistance(-NB_PAS_1M);
  moteurGauche.parcoursCetteDistance(NB_PAS_1M);

  attenteFinDeplacement();

  moteurDroite.parcoursCetteDistance(NB_PAS_DEMI_TOUR);
  moteurGauche.parcoursCetteDistance(NB_PAS_DEMI_TOUR);


  attenteFinDeplacement();


  //while (true);


}
