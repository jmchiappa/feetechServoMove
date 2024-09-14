/// \file STSServoServoDriver.h
/// \brief A simple driver for the STS-serie TTL servos made by Feetech
///
/// \details This code is meant to work as a minimal example for communicating with
///          the STS servos, in particular the low-cost STS-3215 servo.
///          These servos use a communication protocol identical to
///          the Dynamixel serie, but with a different register mapping
///          (due in part to different functionalities like step mode, multiturn...)
#ifndef __FTSERVOMOVE_H__
#define __FTSERVOMOVE_H__

#include "Arduino.h"
#include <STSServoDriver.h>

class FTServoMove
{
    typedef enum {
        ETAT_MOTEUR_STOP = 0,
        ETAT_MOTEUR_DEMARAGE_POSITION,
        ETAT_MOTEUR_POSITION,
        ETAT_MOTEUR_DEMARAGE_CONTINUE,
        ETAT_MOTEUR_CONTINUE,
        ETAT_MOTEUR_AFFINAGE,
        ETAT_MOTEUR_FIN_AFFINAGE,
        ETAT_MOTEUR_CALAGE
    } etatMoteur_t;

    typedef enum {
        ROBOT_AVANCE,
        ROBOT_RECULE,
    } direction_t;

    public:
        /// \brief Constructor
        FTServoMove(STSServoDriver *servo, uint8_t IdServoMoteur){
            servo_ = servo;
            IdServoMoteur_ = IdServoMoteur;
            VitesseNominale_ = 2000;
            Acceleration_ = 20;
            EtatMoteur_ = ETAT_MOTEUR_STOP;
            PositionMoteur_Target_Finale_ = 0;
            NbPasTarget_ = 0;
            PositionMoteur_Target_Intermediaire_ = 0;
            PositionMoteur_Courante_ = 0;
            NbTours_A_Parcourir_ = 0;
            NbTours_Parcourus_ = 0;
            Avance_Recule_ = ROBOT_AVANCE;
            NbPasDeceleration_= 1050;
            NbPasAbsolu_ = 0;
            NbTourAbsolu_ = 0;
            Position_Finale_ = 0;
        }

        /// \brief 
        /// Met le moteur en mode “continue”
        /// Définit l’accélération nominale du mode continue (caractéristique)
        /// Définit la vitesse nominale du mode continue (caractéristique)
        /// Définit le nombre de pas de décélération du mode continue (caractéristique)
        /// \param Vitesse 
        /// \param Acceleration
        /// \param NbPasDeceleration
        /// \returns none
        void init(int32_t Vitesse, uint32_t Acceleration, uint32_t NbPasDeceleration);

        void calageStart (int32_t vitesseCalage);

        void changeSpeed (int32_t vitesseNouvelle);

        uint32_t PositionCourante(void);
        
        void stop(void);
        
        bool estIlEnRoute(void);
        void parcoursCetteDistance (int32_t NbPas);
        void parcoursCetteDistance (int32_t NbPas, int32_t Vitesse, int32_t Acceleration);
        void parcoursCetteDistance_Position_Go(void);
        int32_t getDistanceParcourue(void);

        void Avance_Recule_callback(void);

        void Affinage_Deceleration(void);

        void reset_PositionAbsolue(void);

        int32_t get_PositionAbsolue(void);

        uint8_t getId(void) { return IdServoMoteur_; }
    private:

        void Avance_Recule_PetitDistance (int32_t NbPas, int32_t Vitesse, int32_t Acceleration);

        STSServoDriver *servo_;
        uint8_t IdServoMoteur_ = 0x00;
        int32_t VitesseNominale_;
        uint32_t Acceleration_;
        etatMoteur_t EtatMoteur_;
        uint32_t PositionMoteur_Target_Finale_;
        uint32_t PositionMoteur_Target_Intermediaire_;
        uint32_t PositionMoteur_Courante_;
        int32_t NbPasTarget_;
        uint8_t NbTours_A_Parcourir_;
        uint8_t NbTours_Parcourus_;
        direction_t Avance_Recule_;
        uint32_t NbPasDeceleration_;
        uint8_t NbItDemarageMode_;
        int32_t NbPasAbsolu_;
        int32_t NbTourAbsolu_;
        int32_t Position_Finale_;
        uint8_t CallBack_NbLecture_;
};
#endif
