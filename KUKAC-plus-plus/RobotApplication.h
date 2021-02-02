#pragma once
//#include <Eigen/Dense>
#include <string>
#include "stringProcess.h"
#include "Socket.h"

namespace KUKA{

	class LBRMed{

	private:
		std::string ip_;
		int port_;
		double Teftool[6];
		SocketClient *connection_;



		//************************ judge whether the server has received the cmd************************
		void judgeServerCmd();  



		//************************UPDATE************************
		void updateEEFPos(double EEFpos[6]);
		void updateEEFOffset(double EEFOffset[3]);
		void updateEEFPosServo(double EEFpos[6]);
		void updateEEFPosCIRC(double auxiliaryPos[6],double endPos[6]);
		
		void updateJointPos(double jointpos[7]);
		void updateJointPosServo(double jointpos[7]);
		
		void updatecVel(double cVel);
		void updateJRelVel(double jRelVel);	
		void updateJRelVelServo(double jRelVelServo);
		void updateJointVelocity(double jvel[7]);
		void updateJointVelocityServo(double jvel[7]);



	    //************************Getters methods************************
		// getEEFOrientationQuat
		// getEEFOrientationR
		static void getEEFPos(SocketClient  *);
		static void getEEFForce(SocketClient *);
		static void getEEFMoment(SocketClient *);
		static void getJointsExternalTorques(SocketClient *);
		static void getJointsMeasuredTorques(SocketClient *);
		static void getJointsPos(SocketClient *);



	public:

		LBRMed(std::string ip, int port);
		static void initialize();
		void attachToolToFlange(double Teftool[6]);



		//************************networking methods************************
		void clientConnectionEstablish();
		void net_serverTurnOff();
		static unsigned int __stdcall onlyreceiveThread(void * pThis);
		static void net_serverReceiveTurnOff(SocketClient *receiveClient);



		//************************HandGuiding************************
		void startHandGuiding();
		void endHandGuiding();



		//************************Servo************************
		void startServoMode();
		void endServoMode();



		//************************MOTION************************
		void movePTPinJStoOrigin(double vel);
		void movePTPinJS(double jointpos[7], double relVel);
		void movePTPinCS(double cartpos[6], double vel);
		void moveLIN(double Pos[6], double vel);
		void moveLINRelBase(double pos[3], double relVel);
		void moveLINRelEEF(double pos[3], double relVel);
		void moveLINServo(double endpos[6],double jRelVel);
		void moveCIRC(double auxiliaryPos[6], double endPos[6], double cVel);



	};
} 