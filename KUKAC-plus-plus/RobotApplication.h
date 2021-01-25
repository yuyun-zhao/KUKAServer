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
		void judgeServerCmdOnce();  // judge whether the server has received the cmd
		void judgeServerCmdTwice(); // judge whether the server has  completed  running the cmd



		//************************UPDATE************************
		void updateEEFPos(double EEFpos[6]);
		void updateEEFPosServo(double EEFpos[6]);
		void updateEEFPosCIRC(double auxiliaryPos[6],double endPos[6]);
		
		void updateJointPos(double jointpos[7]);
		void updateJointPosServo(double jointpos[7]);

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



		//************************MOTION************************
		void movePTPinJStoOrigin(double vel);
		void movePTPinJS(double jointpos[7], double relVel);
		void movePTPinCS(double cartpos[6], double vel);
		void moveLIN(double Pos[6], double vel);
		void moveLINRelBase(double pos[3], double relVel);
		void moveLINRelEEF(double pos[3], double relVel);
		void movePTPTransportPositionJointSpace(double vel);
		void moveCIRC(double auxiliaryPos[6], double endPos[6], double cVel);



		////////////////////////////////////////////‘› ±≤ª”√///////////////////////////////////////////////////////




		//************************Getters methods************************
		void getSimulateJoints(double target[6], double joints[7]);
		bool getPin10State();
		bool getPin13State();
		bool getPin16State();
		bool getPin3State();
		bool getPin4State();



		//************************Setters methods************************
		void setBlueOff();
		void setBlueOn();
		void setPin11Off();
		void setPin11On();
		void setPin12Off();
		void setPin12On();
		void setPin1Off();
		void setPin1On();
		void setPin2Off();
		void setPin2On();



		//        //************************General porpuse methods************************
		//        getDHMatrix
		//        gen_partialJacobean
		//        gen_DirectKinematics
		//        gen_InverseKinematics
		//        gen_DirectDynamics
		//        gen_InverseDynamics
		//        gen_GravityVector
		//        gen_CentrifugalMatrix
		//        gen_CoriolisMatrix
		//        gen_MassMatrix
		//        gen_NullSpaceMatrix
		//        normalizeColumns



		//         //************************Interaction functions************************
		//         moveWaitForDTWhenInterrupted
		//         performEventFunctionAtDoubleHit



		//         //************************Nonblocking************************
		//         nonBlocking_isGoalReached
		//         nonBlockingCheck_WithFeedback
		//         nonBlocking_movePTPArcXY_AC
		//         nonBlocking_movePTPArcXZ_AC
		//         nonBlocking_movePTPArcYZ_AC
		//         nonBlocking_movePTPArc_AC
		//         nonBlocking_movePTPCirc1OrintationInter
		//         nonBlocking_movePTPHomeJointSpace
		//         nonBlocking_movePTPJointSpace
		//         nonBlocking_movePTPLineEEF
		//         nonBlocking_movePTPTransportPositionJointSpace



		//         //************************PTP motion functions ************************
		//         movePTPArc_AC
		//         movePTPArcXY_AC
		//         movePTPArcXZ_AC
		//         movePTPArcYZ_AC



		//         //  % PTP ellipse
		//         movePTPEllipse
		//         movePTPEllipse_XY
		//         movePTPEllipse_XZ
		//         movePTPEllipse_YZ
		//         //   % conditional
		//         movePTP_ConditionalTorque_ArcXY_AC
		//         movePTP_ConditionalTorque_ArcXZ_AC
		//         movePTP_ConditionalTorque_ArcYZ_AC
		//         movePTP_ConditionalTorque_Arc_AC
		//         movePTP_ConditionalTorque_Circ1OrintationInter
		//         movePTP_ConditionalTorque_HomeJointSpace
		//         movePTP_ConditionalTorque_JointSpace
		//         movePTP_ConditionalTorque_LineEEF
		//         movePTP_ConditionalTorque_LineEefRelBase
		//         movePTP_ConditionalTorque_TransportPositionJointSpace
		//         //************************ Soft realtime************************
		//         realTime_moveOnPathInJointSpace
		//         realTime_startSmartServoCartesian
		//         realTime_startSmartServoJoints
		//         realTime_startImpedanceJoints
		//         realTime_startVelControlJoints
		//         realTime_stopSmartServoCartesian
		//         realTime_stopSmartServoJoints
		//         realTime_stopImpedanceJoints
		//         realTime_stopVelControlJoints
	};
} 