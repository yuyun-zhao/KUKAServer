#pragma once
namespace KUKA {

	extern double EEFPos_[];    // the end effector positions/orientation of the LBRMed. It's the position and orientation of the media flange of the robot.
	extern double ExternalTorque_[];  // external torque (due to external forces) at joints of the robot.
	extern double jPos_[];      // the joints positions of the LBRMed
	extern double EEFForce_[]; // the endeffector force of the LBRMed. The force is measured at the media flange of the robot.
	extern double MeasuredTorque_[];   // (raw data) as measured by the torque sensors from the joints
	extern double EEFMoment_[]; // the end effector moment of the LBRMed, it's measured at the media flange of the robot.
	extern bool thread_beginflag_; 
	extern bool endflag_;   
	extern bool handguiding_endflag_; 
}

