/**
 * 
 * test simple pour une carte WB55
 * le bus UART Feetech est connecté à l'UART 1 du stm32 en mode half duplex
 * 
*/

#include <HardwareSerial.h>
#include "STSServoDriver.h"
#include "FTservoMove.h"

#define NB_PAS_DEMI_TOUR 4537
#define NB_PAS_QUART_TOUR 2268
#define NB_PAS_1M 8000

#define INDEX_MOTEUR_DROITE   1
#define INDEX_MOTEUR_GAUCHE   2

//Définit 
HardwareSerial COMserie (PA_9);
STSServoDriver busFTUART;

#define DBG_INO
#if defined(DBG_INO)
#   define INO_DBHEADER Serial.print(String("FTServoMove[")+String(IdServoMoteur_)+String("]:"));
#   define INO_COURBE Serial.print("\v Toto:")
#   define INO_DBPRINT(a,b) {INO_DBHEADER;Serial.print(a);Serial.print(":");Serial.print(b);}
#   define INO_DBPRINT1LN(a) Serial.println(a);
#   define INO_DBPRINTLN(a,b) INO_DBHEADER;Serial.print(a);Serial.print(":");Serial.println(b,DEC);
#   define INO_COURBEPRINTLN(a) INO_COURBE;Serial.println(a,DEC);
#else
#   define DBHEADER {}
#   define DBPRINT(a,b) {}
#   define DBPRINT1LN(a,b) {}
#   define DBPRINTLN(a,b,FORMAT) {}
#endif


FTServoMove moteurDroite(&busFTUART,INDEX_MOTEUR_DROITE);
FTServoMove moteurGauche(&busFTUART,INDEX_MOTEUR_GAUCHE);
uint32_t CombienDeTour;

void setup() {
  COMserie.begin(500000);
  Serial.begin(115200);
  while(!Serial);
  Serial.println("Début setup...");
  busFTUART.init(NC, &COMserie);
  Serial.println("Unit ftservomove ...");
  moteurDroite.init(4000 , 40, 2070);
  moteurGauche.init(4000 , 40, 2070);
  Serial.println("fin setup");
  CombienDeTour = 0;
}

void attenteFinDeplacement() {
  while (
    moteurDroite.estIlEnRoute() == true &&
    moteurGauche.estIlEnRoute() == true)
  {
    delay(100);
    //INO_COURBEPRINTLN(moteurDroite.PositionCourante(), moteurGauche.PositionCourante());
  }
  delay (300);
}

void avancerDeXPas(uint32_t x_Nb_Pas_A_Avancer) {
  moteurGauche.parcoursCetteDistance(x_Nb_Pas_A_Avancer);
  moteurDroite.parcoursCetteDistance(-x_Nb_Pas_A_Avancer);
  attenteFinDeplacement();
}

void faireUnDemiTourDroite(){
  moteurDroite.parcoursCetteDistance(NB_PAS_DEMI_TOUR);
  moteurGauche.parcoursCetteDistance(NB_PAS_DEMI_TOUR);
  attenteFinDeplacement();
}

void faireUnDemiTourGauche(){
  moteurDroite.parcoursCetteDistance(-NB_PAS_DEMI_TOUR);
  moteurGauche.parcoursCetteDistance(-NB_PAS_DEMI_TOUR);
  attenteFinDeplacement();
}

void faireUnQuartDeTourDroite(){
  moteurDroite.parcoursCetteDistance(NB_PAS_QUART_TOUR);
  moteurGauche.parcoursCetteDistance(NB_PAS_QUART_TOUR);
  attenteFinDeplacement();
}

void faireUnQuartDeTourGauche(){
  moteurDroite.parcoursCetteDistance(-NB_PAS_QUART_TOUR);
  moteurGauche.parcoursCetteDistance(-NB_PAS_QUART_TOUR);
  attenteFinDeplacement();
}


void loop() {
  //moteurDroite.Affinage_Deceleration();
  //moteurGauche.Affinage_Deceleration();  

  avancerDeXPas(NB_PAS_1M);
  faireUnDemiTourDroite();
  avancerDeXPas(NB_PAS_1M);
  faireUnDemiTourDroite();
  CombienDeTour++;
  Serial.print("Un tour de plus ... = ");
  Serial.println(CombienDeTour);

 // while (true);
}
