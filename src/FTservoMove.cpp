#include "FTservoMove.h"

// #define DBG
#if defined(DBG)
#   define DBHEADER Serial.print(String("FTServoMove[")+String(IdServoMoteur_)+String("]:"));
#   define COURBE Serial.print("\v Toto:")
#   define DBPRINT(a,b) {DBHEADER;Serial.print(a);Serial.print(":");Serial.print(b);}
#   define DBPRINT1LN(a) DBHEADER;Serial.println(a);
#   define DBPRINTLN(a,b) DBHEADER;Serial.print(a);Serial.print(":");Serial.println(b,DEC);
#   define COURBEPRINTLN(a) COURBE;Serial.println(a,DEC);
#else
#   define DBHEADER {}
#   define DBPRINT(a,b) {}
#   define DBPRINT1LN(a) {}
#   define DBPRINTLN(a,b) {}
#endif

#define PIN_DEBUG  LED_BLUE // BLUE LED
bool toggle = false;

namespace STS::parameter {
    const uint16_t COMPTEUR_MAX = 4096;
    const uint16_t VALEUR_MAX_PETITE_DISTANCE = 27000;
    const uint8_t  NB_IT_DEMARRAGE = 10;
}

FTServoMove *obj[10];
LPTIM_HandleTypeDef hlptim1;

uint8_t obj_ptr=0;

bool timerisRunning = false;

/**
 * static C functions
 * 
 **/ 

extern "C" void LPTIM1_IRQHandler(void)
{
  HAL_LPTIM_IRQHandler(&hlptim1);
}

extern "C" void HAL_LPTIM_AutoReloadMatchCallback(LPTIM_HandleTypeDef *hlptim) {
  // Timer callback
  if(toggle) digitalToggle( PIN_DEBUG );
  toggle = !toggle;

  for( uint8_t i=0; i<10; i++) {
      if(obj[i]!=NULL) {
        obj[i]->Avance_Recule_callback();
      }
    }
}

void HAL_LPTIM_MspInit(LPTIM_HandleTypeDef* hlptim)
{
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};
  if(hlptim->Instance==LPTIM_INSTANCE)
  {
    PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_LPTIM1;
    PeriphClkInitStruct.Lptim1ClockSelection = LL_RCC_LPTIM1_CLKSOURCE_PCLK1;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
    {
      Error_Handler();
    }

  /* Peripheral clock enable */
   __HAL_RCC_LPTIM1_CLK_ENABLE();

  /* ## - 2 - Force & Release the LPTIM Periheral Clock Reset ############### */  
  /* Force the LPTIM Peripheral Clock Reset */
  __HAL_RCC_LPTIM1_FORCE_RESET();
  
  /* Release the LPTIM Peripheral Clock Reset */
  __HAL_RCC_LPTIM1_RELEASE_RESET();

    /* LPTIM1 interrupt Init */
    HAL_NVIC_SetPriority(LPTIM1_IRQn, 14, 0);
    HAL_NVIC_EnableIRQ(LPTIM1_IRQn);
  }
}

/**
 * this static function aims to manage callback which fires period interrupt
 * Calling this static memeber will subscribe the object to the poller.
 * if the poller is not running, start it.
*/
void FTServoMove::callbackManager(FTServoMove *servo) {
  if(timerisRunning == false ) {
    // Power on blue LED for debug
    pinMode( PIN_DEBUG, OUTPUT );
    digitalWrite( PIN_DEBUG, HIGH );

    timerisRunning = true;
    hlptim1.Instance = LPTIM_INSTANCE;
    hlptim1.Init.Clock.Source = LPTIM_CLOCKSOURCE_APBCLOCK_LPOSC;
    hlptim1.Init.Clock.Prescaler = LPTIM_PRESCALER_DIV128;
    hlptim1.Init.Trigger.Source = LPTIM_TRIGSOURCE_SOFTWARE;
    hlptim1.Init.OutputPolarity = LPTIM_OUTPUTPOLARITY_HIGH;
    hlptim1.Init.UpdateMode = LPTIM_UPDATE_IMMEDIATE;
    hlptim1.Init.CounterSource = LPTIM_COUNTERSOURCE_INTERNAL;
    hlptim1.Init.Input1Source = LPTIM_INPUT1SOURCE_GPIO;
    hlptim1.Init.Input2Source = LPTIM_INPUT2SOURCE_GPIO;
    if (HAL_LPTIM_Init(&hlptim1) != HAL_OK)
    {
      Error_Handler();
    }

    // Clean the FTservo obj array
    for(uint8_t i=0;i<10;i++) obj[i] = NULL;

    uint32_t ARR_RegisterValue = 32000000 * POLLING_DELAY / (128 * 1000) -1;
    HAL_LPTIM_Counter_Start_IT( &hlptim1,ARR_RegisterValue);
  }

  obj[obj_ptr++]= servo;
}

void FTServoMove::init(int32_t Vitesse, uint32_t Acceleration, uint32_t NbPasDeceleration)
{
    VitesseNominale_ = Vitesse;
    Acceleration_ = Acceleration;
    NbPasDeceleration_ = NbPasDeceleration;
    servo_->setOperationMode(IdServoMoteur_ , STS::mode::CONTINUOUS); delay(1);
    servo_->setTargetAcceleration(IdServoMoteur_, Acceleration_, false); delay(1);
    servo_->setTargetVelocity(IdServoMoteur_, 0, false);
    callbackManager(this);
    DBPRINT1LN("init::initialisation terminée.");
}


void FTServoMove::reset_PositionAbsolue(void)
{
    NbPasAbsolu_ = 0;
    NbTourAbsolu_ = 0;
}

int32_t FTServoMove::get_PositionAbsolue(void)
{
    return (NbTourAbsolu_ * STS::parameter::COMPTEUR_MAX + NbPasAbsolu_); 
}

void FTServoMove::changeSpeed (int32_t vitesseNouvelle) {
    DBPRINT1LN("changeSpeed::Début...");
    VitesseNominale_ = vitesseNouvelle;
    servo_->setTargetVelocity(IdServoMoteur_, vitesseNouvelle, false); delay(1);
}

void FTServoMove::calageStart (int32_t vitesseCalage) {
    DBPRINT1LN("calageStart::Début calage...");
    servo_->setOperationMode(IdServoMoteur_, STS::mode::CONTINUOUS); delay(1);
    servo_->setTargetAcceleration(IdServoMoteur_, Acceleration_,false); delay(1);
    servo_->setTargetVelocity(IdServoMoteur_, vitesseCalage, false); delay(1);
    EtatMoteur_ = ETAT_MOTEUR_CALAGE;
}

void FTServoMove::stop(void) {
    DBPRINT1LN("stop::stop.");
    servo_->setOperationMode(IdServoMoteur_, STS::mode::CONTINUOUS); delay(1);
    servo_->setTargetVelocity(IdServoMoteur_, 0, false); delay(1);
    EtatMoteur_ = ETAT_MOTEUR_STOP;
}

void FTServoMove::Affinage_Deceleration(void) {
  int toto1;
  int toto2;

  servo_->setTargetVelocity(IdServoMoteur_, 4000, false);
  delay(3000);
  servo_->setTargetVelocity(IdServoMoteur_, 0, false);
  toto1 = servo_->getCurrentPosition(IdServoMoteur_);
  DBPRINTLN("Position Stop : ",toto1);
  while (servo_->isMoving(IdServoMoteur_) == true);
  delay(1000);
  toto2 = servo_->getCurrentPosition(IdServoMoteur_);
  DBPRINTLN("Position arret : ", toto2);
  DBPRINTLN("Delta : ", (toto2-toto1));
}

uint32_t FTServoMove::PositionCourante(void)
{
  return (servo_->getCurrentPosition(IdServoMoteur_));
}

bool FTServoMove::estIlEnRoute(void) {
    // DBPRINTLN("leMoteurEstIlEnRoute : ",EtatMoteur_ != ETAT_MOTEUR_STOP);
    if (EtatMoteur_ == ETAT_MOTEUR_STOP)
    {
      return (false);
    }
    else
    {
      return (true);
    }
}

void FTServoMove::parcoursCetteDistance (int32_t NbPas) {
  int Vitesse_A_Programmer;
  int Distance_A_Programmer_Mode_Continue;
  int X_DeDepart;

  if (abs(NbPas) <= STS::parameter::VALEUR_MAX_PETITE_DISTANCE) {
    Avance_Recule_PetitDistance( NbPas);
  } else {
    DBPRINTLN("::::::::::::: Parcours Grand distance",IdServoMoteur_);
    X_DeDepart = servo_->getCurrentPosition(IdServoMoteur_); delay(1);
    if (X_DeDepart == 0){
      X_DeDepart = servo_->getCurrentPosition(IdServoMoteur_);
    }
    DBPRINTLN("X_DeDepart : ",X_DeDepart);
    DBPRINTLN("Ajout : ",(NbPas % STS::parameter::COMPTEUR_MAX));
    if (NbPas >= 0) {
      Avance_Recule_ = ROBOT_AVANCE;
      Vitesse_A_Programmer = VitesseNominale_;
      Distance_A_Programmer_Mode_Continue = NbPas - NbPasDeceleration_;
      PositionMoteur_Target_Finale_ = (X_DeDepart + (NbPas % STS::parameter::COMPTEUR_MAX))%STS::parameter::COMPTEUR_MAX;
      NbTours_A_Parcourir_ = (abs(Distance_A_Programmer_Mode_Continue / STS::parameter::COMPTEUR_MAX));
      PositionMoteur_Target_Intermediaire_ = (X_DeDepart + (Distance_A_Programmer_Mode_Continue % STS::parameter::COMPTEUR_MAX));
      if (PositionMoteur_Target_Intermediaire_ >= STS::parameter::COMPTEUR_MAX) {
        NbTours_A_Parcourir_ ++;
        PositionMoteur_Target_Intermediaire_ -= STS::parameter::COMPTEUR_MAX;
      }
    } else {
      Avance_Recule_ = ROBOT_RECULE;
      Vitesse_A_Programmer = -VitesseNominale_;
      Distance_A_Programmer_Mode_Continue = NbPas + NbPasDeceleration_;
      if ((X_DeDepart + (NbPas % STS::parameter::COMPTEUR_MAX)) < 0){
        PositionMoteur_Target_Finale_ = STS::parameter::COMPTEUR_MAX + (X_DeDepart + (NbPas % STS::parameter::COMPTEUR_MAX));
      }else {
        PositionMoteur_Target_Finale_ = X_DeDepart + (NbPas % STS::parameter::COMPTEUR_MAX);
      }
      NbTours_A_Parcourir_ = (abs(Distance_A_Programmer_Mode_Continue / STS::parameter::COMPTEUR_MAX));
      if ((X_DeDepart + (Distance_A_Programmer_Mode_Continue % STS::parameter::COMPTEUR_MAX)) < 0){
        NbTours_A_Parcourir_ ++;
        DBPRINTLN("Un tour de plus - NbTours_A_Parcourir_ : ",NbTours_A_Parcourir_);
        PositionMoteur_Target_Intermediaire_ = STS::parameter::COMPTEUR_MAX + (X_DeDepart + (Distance_A_Programmer_Mode_Continue % STS::parameter::COMPTEUR_MAX));
      }
      else{
        PositionMoteur_Target_Intermediaire_ = X_DeDepart + (Distance_A_Programmer_Mode_Continue % STS::parameter::COMPTEUR_MAX);
      }
    }
    DBPRINTLN("NbTours_A_Parcourir_ : ",NbTours_A_Parcourir_);
    DBPRINTLN("PositionMoteur_Target_Intermediaire_ : ",PositionMoteur_Target_Intermediaire_);
    DBPRINTLN("Position Finale : ",(int) PositionMoteur_Target_Finale_);   
    NbTours_Parcourus_ = 0;
    PositionMoteur_Courante_ = X_DeDepart;
    NbItDemarageMode_ = 0;
    EtatMoteur_ = ETAT_MOTEUR_DEMARAGE_CONTINUE;
    servo_->setOperationMode(IdServoMoteur_, STS::mode::CONTINUOUS); delay(1);
    servo_->setOperationMode(IdServoMoteur_, STS::mode::CONTINUOUS); delay(1);
    servo_->setTargetAcceleration(IdServoMoteur_, Acceleration_, false); delay(1);
    servo_->setTargetAcceleration(IdServoMoteur_, Acceleration_, false); delay(1);
    servo_->setTargetVelocity(IdServoMoteur_, Vitesse_A_Programmer, false); delay(1);
    servo_->setTargetVelocity(IdServoMoteur_, Vitesse_A_Programmer, false); delay(1);
  }
}

void FTServoMove::Avance_Recule_PetitDistance(int32_t NbPas) {
  int X_DeDepart;
  int Position_Finale; 

  servo_->setOperationMode(IdServoMoteur_, STS::mode::POSITION); delay(1);
  servo_->setTargetAcceleration(IdServoMoteur_, Acceleration_, false); delay(1);
  X_DeDepart = servo_->getCurrentPosition(IdServoMoteur_);delay(1);
  if (X_DeDepart == 0){
    X_DeDepart = servo_->getCurrentPosition(IdServoMoteur_);
  }
  //DBPRINTLN("Avance_Recule_PetitDistance : NbPas",(int) NbPas);   
  //DBPRINTLN("Avance_Recule_PetitDistance : X_DeDepart",(int) X_DeDepart);   
  Position_Finale = X_DeDepart + NbPas;
  NbPasTarget_ = NbPas;

  if (NbPas >= 0) {
      Avance_Recule_ = ROBOT_AVANCE;
      PositionMoteur_Target_Finale_ = Position_Finale % STS::parameter::COMPTEUR_MAX;
      NbTours_A_Parcourir_ = (abs(NbPas / STS::parameter::COMPTEUR_MAX));
    } else {
      Avance_Recule_ = ROBOT_RECULE;
      PositionMoteur_Target_Finale_ = STS::parameter::COMPTEUR_MAX + Position_Finale % STS::parameter::COMPTEUR_MAX;
      NbTours_A_Parcourir_ = (abs(NbPas / STS::parameter::COMPTEUR_MAX));
    } 
     
  NbItDemarageMode_ = 0;
  EtatMoteur_ = ETAT_MOTEUR_DEMARAGE_POSITION;
  servo_->setTargetPosition(IdServoMoteur_, Position_Finale, false);
}

void FTServoMove::Avance_Recule_callback() {
  // DBPRINT1LN(F("entering in IT callback"));
  int v_it_PositionMoteurCourante;
  
  if (EtatMoteur_ != ETAT_MOTEUR_STOP) {
    if (EtatMoteur_ == ETAT_MOTEUR_DEMARAGE_POSITION) {
      NbItDemarageMode_ ++;
      if (NbItDemarageMode_ >= STS::parameter::NB_IT_DEMARRAGE){
        // Le moteur doit maintenant être lancé - Message d'erreur sinon
        EtatMoteur_ = ETAT_MOTEUR_POSITION;
        if (servo_->isMoving(IdServoMoteur_) == 0) {
            DBPRINTLN("ETAT_MOTEUR_DEMARAGE_POSITION : Le moteur n'a pas démarré !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!",servo_->getCurrentPosition(IdServoMoteur_));
        }
      }
    } else {
      if (EtatMoteur_== ETAT_MOTEUR_POSITION) {
        if (servo_->isMoving(IdServoMoteur_) == 0) {
          delay(1);
          if (servo_->isMoving(IdServoMoteur_) == 0) {
            //DBPRINTLN("Avance_Recule_callback - Fin Mode position - Position reel    : ",servo_->getCurrentPosition(IdServoMoteur_));
            //DBPRINTLN("Avance_Recule_callback - Fin Mode position - Position attendue: ",(int) PositionMoteur_Target_Finale_);
            NbPasAbsolu_ += NbPasTarget_;
            //DBPRINTLN("Avance_Recule_callback - Fin Mode position - Position absolue: ", (int) NbPasAbsolu_);
            if (abs ((int) (servo_->getCurrentPosition(IdServoMoteur_) - PositionMoteur_Target_Finale_)) > 1){
            DBPRINTLN("Avance_Recule_callback - Delta Position Commande = ",(int) (servo_->getCurrentPosition(IdServoMoteur_) - PositionMoteur_Target_Finale_));
            }
            stop();
            }
          }
        } else {
        if (EtatMoteur_ == ETAT_MOTEUR_DEMARAGE_CONTINUE) {
          NbItDemarageMode_ ++;
          //DBPRINTLN("ETAT_MOTEUR_DEMARAGE_CONTINUE : ",servo_->getCurrentPosition(IdServoMoteur_));
          if (NbItDemarageMode_ >= STS::parameter::NB_IT_DEMARRAGE){
            EtatMoteur_ = ETAT_MOTEUR_CONTINUE;
            DBPRINTLN("ETAT_MOTEUR_DEMARAGE_CONTINUE - FIN : ",servo_->getCurrentPosition(IdServoMoteur_));
          }
        } else {
          v_it_PositionMoteurCourante = servo_->getCurrentPosition(IdServoMoteur_);
          delay(1);
          if (v_it_PositionMoteurCourante == 0){
            v_it_PositionMoteurCourante = servo_->getCurrentPosition(IdServoMoteur_);
            }
          if (EtatMoteur_ == ETAT_MOTEUR_CONTINUE) {
            if (Avance_Recule_ == ROBOT_AVANCE) {
              if (PositionMoteur_Courante_ > v_it_PositionMoteurCourante) {
                NbTours_Parcourus_++;
                DBPRINTLN("AVANCE - Un tour de plus : ",NbTours_Parcourus_);
                //if (NbTours_Parcourus_ == 1){
                //    DBPRINTLN("NbTourAtteint - PositionMoteur_Courante_ : ",PositionMoteur_Courante_);
                //    DBPRINTLN("NbTourAtteint - v_it_PositionMoteurCourante : ",v_it_PositionMoteurCourante);
                //}
              }
            } else {
              if (PositionMoteur_Courante_ < v_it_PositionMoteurCourante) {
                NbTours_Parcourus_++;
                DBPRINTLN("RECULE - Un tour de plus : ",NbTours_Parcourus_);
                //if (NbTours_Parcourus_ == 1){
                //    DBPRINTLN("NbTourAtteint - PositionMoteur_Courante_ : ",PositionMoteur_Courante_);
                //    DBPRINTLN("NbTourAtteint - v_it_PositionMoteurCourante : ",v_it_PositionMoteurCourante);
                //}
              }
            }
            if (NbTours_A_Parcourir_ <= NbTours_Parcourus_) {
              EtatMoteur_ = ETAT_MOTEUR_AFFINAGE;
              DBPRINTLN("NbTourAtteint - Courante : ",servo_->getCurrentPosition(IdServoMoteur_));
              DBPRINTLN("NbTourAtteint - NbTours_Parcourus_ : ",NbTours_Parcourus_);
            }
          } 
          if (EtatMoteur_ == ETAT_MOTEUR_AFFINAGE) {
            if (Avance_Recule_ == ROBOT_AVANCE) {
              if (PositionMoteur_Target_Intermediaire_ <= v_it_PositionMoteurCourante) {
                EtatMoteur_ = ETAT_MOTEUR_FIN_AFFINAGE;
                servo_->setTargetVelocity(IdServoMoteur_, 0, false);
                delay(1);
                servo_->setTargetVelocity(IdServoMoteur_, 0, false);
                delay(1);
                DBPRINTLN("Avance - Affinage - Debut AVANCE - PosCourante : ",v_it_PositionMoteurCourante);
                DBPRINTLN("Avance - Affinage - Arret moteur - Target Int : ",PositionMoteur_Target_Intermediaire_);
              }
            } else {
              //DBPRINTLN("Affinage Recule... En cours... ",NbTours_Parcourus_);
              if (PositionMoteur_Target_Intermediaire_ >= v_it_PositionMoteurCourante) {
                DBPRINTLN("Recule - Affinage - Arret moteur - Courante : ",servo_->getCurrentPosition(IdServoMoteur_));
                DBPRINTLN("Recule - Affinage - Arret moteur - Target Int : ",(int) PositionMoteur_Target_Intermediaire_);
                EtatMoteur_ = ETAT_MOTEUR_FIN_AFFINAGE;
                servo_->setTargetVelocity(IdServoMoteur_, 0, false);
                delay(1);
                servo_->setTargetVelocity(IdServoMoteur_, 0, false);
                delay(1);
              }
            }
          }
          if (EtatMoteur_ == ETAT_MOTEUR_FIN_AFFINAGE) {
            if (servo_->isMoving(IdServoMoteur_) == 0) {
              DBPRINTLN("Affinage Fin - Mode position : ",servo_->getCurrentPosition(IdServoMoteur_));
              DBPRINTLN("Affinage Fin - PositionMoteur_Target_Finale_ : ",PositionMoteur_Target_Finale_);
              servo_->setOperationMode(IdServoMoteur_, STS::mode::POSITION);
              delay(1);
              servo_->setOperationMode(IdServoMoteur_, STS::mode::POSITION);
              delay(1);
              servo_->setTargetAcceleration(IdServoMoteur_, Acceleration_, false);
              delay(1);
              servo_->setTargetAcceleration(IdServoMoteur_, Acceleration_, false);
              delay(1);
              NbItDemarageMode_ = 0;
              EtatMoteur_ = ETAT_MOTEUR_DEMARAGE_POSITION;
              servo_->setTargetPosition(IdServoMoteur_, PositionMoteur_Target_Finale_, false);
              delay(1);
            }
          } 
          PositionMoteur_Courante_ = v_it_PositionMoteurCourante;
        }
      }
    }
  }
}