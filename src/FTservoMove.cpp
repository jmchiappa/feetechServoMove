#include "FTservoMove.h"

#define DBG
#if defined(DBG)
#   define DBHEADER {Serial.print("FTServoMove[");Serial.print(IdServoMoteur_);Serial.print("]:");}
#   define COURBE Serial.print("\v Toto:")  
#   define DBPRINT(a,b) {DBHEADER;Serial.print(a);Serial.print(":");Serial.print(b);}
#   define DBPRINT1LN(a) {DBHEADER;Serial.println(a);}
#   define DBPRINTLN(a,b) {DBHEADER;Serial.print(a);Serial.print(":");Serial.println(b,DEC);}
#   define COURBEPRINTLN(a) {COURBE;Serial.println(a,DEC);}
#else
#   define DBHEADER {}
#   define COURBE  {}
#   define DBPRINT(a,b)  {}
#   define DBPRINT1LN(a)  {}
#   define DBPRINTLN(a,b)  {}
#   define COURBEPRINTLN(a)  {}
#endif

namespace STS::parameter {
    const uint16_t COMPTEUR_MAX = 4096;
    const uint16_t VALEUR_MAX_PETITE_DISTANCE = 27000;
    const uint8_t  NB_IT_DEMARRAGE = 10;
}

/**
 * static C functions
 * 
 **/ 

void FTServoMove::init(int32_t Vitesse, uint32_t Acceleration, uint32_t NbPasDeceleration)
{
    VitesseNominale_ = Vitesse;
    Acceleration_ = Acceleration;
    NbPasDeceleration_ = NbPasDeceleration;
    EtatMoteur_ = ETAT_MOTEUR_STOP;
    servo_->setOperationMode(IdServoMoteur_ , STS::mode::CONTINUOUS); delay(1);
    servo_->setTargetAcceleration(IdServoMoteur_, Acceleration_, false); delay(1);
    servo_->setTargetVelocity(IdServoMoteur_, 0, false);
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
    //DBPRINT1LN("stop::stop.");
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

void FTServoMove::parcoursCetteDistance (int32_t NbPas, int32_t Vitesse, int32_t Acceleration) {
  int Vitesse_A_Programmer;
  int Distance_A_Programmer_Mode_Continue;
  int X_DeDepart;

  if (abs(NbPas) <= STS::parameter::VALEUR_MAX_PETITE_DISTANCE) {
    //DBPRINTLN("::::::::::::: Parcours Petite distance",IdServoMoteur_);
    if (abs(NbPas) > 5)
    {
      Avance_Recule_PetitDistance( NbPas, Vitesse, Acceleration);
    }
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

void FTServoMove::Avance_Recule_PetitDistance (int32_t NbPas, int32_t Vitesse, int32_t Acceleration) {
  int X_DeDepart;
  int NbMaxLecture;

  X_DeDepart = servo_->getCurrentPosition(IdServoMoteur_);delay(1);
  if (X_DeDepart == 0) X_DeDepart = servo_->getCurrentPosition(IdServoMoteur_); delay (1);
  //DBPRINTLN("Avance_Recule_PetitDistance : X_DeDepart = ",(int) X_DeDepart);
  //DBPRINTLN("Avance_Recule_PetitDistance : Distance = ",(int) NbPas);
  if ((X_DeDepart < 100) || (X_DeDepart > 4000)) {
    servo_->setPositionCorrection(IdServoMoteur_, 200, false); delay(1);
  }
  else {
    servo_->setPositionCorrection(IdServoMoteur_, 0, false); delay(1);
  }
  servo_->setOperationMode(IdServoMoteur_, STS::mode::POSITION); delay(1);
  servo_->setTargetAcceleration(IdServoMoteur_, Acceleration, false); delay(1);
  servo_->setTargetVelocity(IdServoMoteur_, Vitesse, false); delay(1);
  X_DeDepart = 0;
  NbMaxLecture = 40;
  while ((X_DeDepart == 0) && (NbMaxLecture != 0))
  {
    X_DeDepart = servo_->getCurrentPosition(IdServoMoteur_); delay(1);
    NbMaxLecture--;
  }
  DBPRINTLN("::::::::::::: Parcours Petit distance",NbPas);
  //DBPRINTLN("Avance_Recule_PetitDistance : X_DeDepart = ",(int) X_DeDepart);  
  //DBPRINTLN("Avance_Recule_PetitDistance : NbPas",(int) NbPas);   
  Position_Finale_ = X_DeDepart + NbPas;
  NbPasTarget_ = NbPas;

  if (NbPas >= 0) {
    Avance_Recule_ = ROBOT_AVANCE;
    PositionMoteur_Target_Finale_ = Position_Finale_ % STS::parameter::COMPTEUR_MAX;
    //NbTours_A_Parcourir_ = (abs(NbPas / STS::parameter::COMPTEUR_MAX));
  } else {
    Avance_Recule_ = ROBOT_RECULE;
    PositionMoteur_Target_Finale_ = STS::parameter::COMPTEUR_MAX + Position_Finale_ % STS::parameter::COMPTEUR_MAX;
    //NbTours_A_Parcourir_ = (abs(NbPas / STS::parameter::COMPTEUR_MAX));
  } 
     
  NbItDemarageMode_ = 0;
  PositionMoteur_Courante_ = X_DeDepart;
  NbPasAbsolu_ = 0;
  // NbTours_Parcourus_ = 0;
  EtatMoteur_ = ETAT_MOTEUR_DEMARAGE_POSITION;
  //servo_->setTargetPosition(IdServoMoteur_, Position_Finale_, false);
  //getDistanceParcourue();
}

void FTServoMove::parcoursCetteDistance_Position_Go(){
  servo_->setTargetPosition(IdServoMoteur_, Position_Finale_, false);
}

int32_t FTServoMove::getDistanceParcourue(){
  int v_it_PositionMoteurCourante = 0;
  int NbMaxLecture;

  NbMaxLecture = 40;
  while ((v_it_PositionMoteurCourante == 0) && (NbMaxLecture != 0))
  {
    v_it_PositionMoteurCourante = servo_->getCurrentPosition(IdServoMoteur_); delay(1);
    NbMaxLecture--;
  }
  if (Avance_Recule_ == ROBOT_AVANCE) {
    //DBPRINTLN("getDistanceParcourue_AVANCE : PositionMoteur_Courante_ = ",(int) PositionMoteur_Courante_);  
    if (v_it_PositionMoteurCourante < PositionMoteur_Courante_){
      if ((PositionMoteur_Courante_ - v_it_PositionMoteurCourante) > 10)
        NbPasAbsolu_ += (4096 - PositionMoteur_Courante_) + v_it_PositionMoteurCourante;
    } else {
      NbPasAbsolu_ += v_it_PositionMoteurCourante - PositionMoteur_Courante_;
    }
    //DBPRINTLN("getDistanceParcourue_AVANCE : v_it_PositionMoteurCourante = ",(int) v_it_PositionMoteurCourante); 
  } else { // On recule...
    //DBPRINTLN("getDistanceParcourue_RECULE : PositionMoteur_Courante_ = ",(int) PositionMoteur_Courante_);  
    if (v_it_PositionMoteurCourante > PositionMoteur_Courante_){
      if ((v_it_PositionMoteurCourante - PositionMoteur_Courante_) > 10)
        NbPasAbsolu_ += - PositionMoteur_Courante_ - (4096 - v_it_PositionMoteurCourante);
    } else {
      NbPasAbsolu_ += v_it_PositionMoteurCourante - PositionMoteur_Courante_;
      }
    //DBPRINTLN("getDistanceParcourue_RECULE : v_it_PositionMoteurCourante = ",(int) v_it_PositionMoteurCourante); 
  }
  PositionMoteur_Courante_ = v_it_PositionMoteurCourante;
  //DBPRINTLN("getDistanceParcourue: ", (int) NbPasAbsolu_);
  return (NbPasAbsolu_);
}

void FTServoMove::Avance_Recule_callback() {
  //DBPRINT1LN(F("entering in IT callback"));
  int v_it_PositionMoteurCourante;
  int NbMaxLecture;
 
  if (EtatMoteur_ != ETAT_MOTEUR_STOP) {
    if (EtatMoteur_ == ETAT_MOTEUR_DEMARAGE_POSITION) {
      NbItDemarageMode_ ++;
      //getDistanceParcourue();
      if (NbItDemarageMode_ >= STS::parameter::NB_IT_DEMARRAGE){
      //   // Le moteur doit maintenant être lancé - Message d'erreur sinon
         EtatMoteur_ = ETAT_MOTEUR_POSITION;
      //   if (servo_->isMoving(IdServoMoteur_) == 0) {
      //        DBPRINTLN("ETAT_MOTEUR_DEMARAGE_POSITION : Le moteur n'a pas démarré !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!",servo_->getCurrentPosition(IdServoMoteur_));
      //   }
      }
    } else {
      if (EtatMoteur_== ETAT_MOTEUR_POSITION) {
        NbMaxLecture = 10;
        while ((servo_->isMoving(IdServoMoteur_) == 0) && (NbMaxLecture != 0))
        {
          NbMaxLecture--; delay(10);
        }
        if (NbMaxLecture == 0) {
          //DBPRINTLN("Avance_Recul_callback - Fin Mode position - Position reel    : ",servo_->getCurrentPosition(IdServoMoteur_));
          //DBPRINTLN("Avance_Recule_callback - Fin Mode position - Position attendue: ",(int) PositionMoteur_Target_Finale_);
          if (abs ((int) (servo_->getCurrentPosition(IdServoMoteur_) - PositionMoteur_Target_Finale_)) > 10){
            DBPRINTLN("!!!!!!!!!!! Avance_Recule_callback - Delta Position Commande = ",(int) (servo_->getCurrentPosition(IdServoMoteur_) - PositionMoteur_Target_Finale_));
          }
          //getDistanceParcourue();
          DBPRINTLN("-------------getDistanceParcourue = ",(int) getDistanceParcourue());

          stop();

          }
        else{
          //Still running - Count NbTour in case of emergency stop
          getDistanceParcourue();
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
                servo_->setTargetVelocity(IdServoMoteur_, 0, false); delay(1);
                servo_->setTargetVelocity(IdServoMoteur_, 0, false); delay(1);
              }
            }
          }
          if (EtatMoteur_ == ETAT_MOTEUR_FIN_AFFINAGE) {
            if (servo_->isMoving(IdServoMoteur_) == 0) {
              DBPRINTLN("Affinage Fin - Mode position : ",servo_->getCurrentPosition(IdServoMoteur_));
              DBPRINTLN("Affinage Fin - PositionMoteur_Target_Finale_ : ",PositionMoteur_Target_Finale_);
              servo_->setOperationMode(IdServoMoteur_, STS::mode::POSITION); delay(1);
              servo_->setOperationMode(IdServoMoteur_, STS::mode::POSITION); delay(1);
              servo_->setTargetAcceleration(IdServoMoteur_, Acceleration_, false); delay(1);
              servo_->setTargetAcceleration(IdServoMoteur_, Acceleration_, false); delay(1);
              NbItDemarageMode_ = 0;
              EtatMoteur_ = ETAT_MOTEUR_DEMARAGE_POSITION;
              servo_->setTargetPosition(IdServoMoteur_, PositionMoteur_Target_Finale_, false); delay(1);
            }
          } 
          PositionMoteur_Courante_ = v_it_PositionMoteurCourante;
        }
      }
    }
  }
}
