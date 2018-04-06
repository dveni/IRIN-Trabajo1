#ifndef IRI1CONTROLLER_H_
#define IRI1CONTROLLER_H_

/******************************************************************************/
/******************************************************************************/

#include "controller.h"

/******************************************************************************/
/******************************************************************************/

class CIri1Controller : public CController
{
public:

    CIri1Controller (const char* pch_name, CEpuck* pc_epuck, int n_wrtie_to_file);
    ~CIri1Controller();
    void SimulationStep(unsigned n_step_number, double f_time, double f_step_interval);

private:
		/* ROBOT */
    CEpuck* m_pcEpuck;
   
	 	/* SENSORS */
		CWheelsActuator* m_acWheels;
    CEpuckProximitySensor* m_seProx;
		CRealLightSensor* m_seLight;
		CContactSensor* m_seContact;
		CGroundSensor* m_seGround;
		CGroundMemorySensor* m_seGroundMemory;
		CBatterySensor* m_seBattery;   

		CRealBlueLightSensor* m_seBlueLight;
		CRealRedLightSensor* m_seRedLight;
		CBlueBatterySensor* m_seBlueBattery;
		CRedBatterySensor* m_seRedBattery;
		

		/* Global Variables */
		double 		m_fLeftSpeed;
		double 		m_fRightSpeed;
		double**	m_fActivationTable;
		int 			m_nWriteToFile;
		double 		m_fTime;
		// Si  parada = 0 no se para el robot, si parada es igual a 1 se para el robot
		double 	 	parada;
		double 		carga_actual;
		double 		carga_lastStep;
		double 		flagstop;
		//Inhibitor
  		double 		AvoidInhibitor;
		double    fBattToForageInhibitor;
	    double		followScentInhibitor;	
	    double		ofrendaInhibitor;

	    	int 		mochila;
 		//ofrenda
		double  honey;
		double carga_lastStep2;
		double carga_actual2;
		double flag_god;
		
		/* Functions */


		void ExecuteBehaviors ( void );
		void Coordinator ( void );

		void ObstacleAvoidance ( unsigned int un_priority );
		void Navigate ( unsigned int un_priority );
		void TakeABreak ( unsigned int un_priority );
		void Forage ( unsigned int un_priority );
		void FollowScent (unsigned int un_priority);
		void Ofrenda(unsigned int un_priority);
		void Die(unsigned int un_priority);
		double* calcDirection(double* light);
};

#endif
