#include <iostream>
#include <iostream>
#include <sstream>
#include <fstream>
#include <process.h>
#include "RobotApplication.h"
#include "globalVars.h"
#include "stdafx.h"

KUKA::LBRMed::LBRMed(std::string ip, int port) : connection_(NULL)
{
	this->ip_ = ip;
	this->port_ = port;

	//this->Teftool[0] = Teftool[0];
	//this->Teftool[1] = Teftool[1];
	//this->Teftool[2] = Teftool[2];
	//this->Teftool[3] = Teftool[3];
	//this->Teftool[4] = Teftool[4];
	//this->Teftool[5] = Teftool[5];
}

void KUKA::LBRMed::initialize() {

	KUKA::EEFPos_[6] = { 0.0 };
	KUKA::jPos_[7] = { 0.0 };
	KUKA::EEFForce_[3] = { 0.0 };
	KUKA::MeasuredTorque_[7] = { 0.0 };
	KUKA::ExternalTorque_[7] = { 0.0 };
	KUKA::EEFMoment_[3] = { 0.0 };
	KUKA::thread_beginflag_ = false;
	KUKA::endflag_ = false;
	KUKA::handguiding_endflag_ = false;

}

void  KUKA::LBRMed::attachToolToFlange(double Teftool[6]) {
	std::string cmd = "TFtrans_" + std::to_string(Teftool[0]) + "_" + std::to_string(Teftool[1]) + "_" +
									 std::to_string(Teftool[2]) + "_" + std::to_string(Teftool[3]) + "_" +
										std::to_string(Teftool[4]) + "_" + std::to_string(Teftool[5]) + "_";
	this->connection_->write(cmd);
	judgeServerCmdOnce();
}



//************************networking methods************************
void  KUKA::LBRMed::clientConnectionEstablish(){

	try{
		connection_ = new SocketClient(this->ip_, this->port_);
		//attachToolToFlange(this->Teftool);
	} catch (const char *s){	
		std::cerr << s << std::endl;
	} catch (std::string s){
		std::cerr << s << std::endl;
	}catch (...){
		std::cerr << "unhandled exception\n";
	}
	Sleep(300);
}
void KUKA::LBRMed::net_serverTurnOff(){
	this->connection_->write("END");
	KUKA::endflag_ = true;
}
void KUKA::LBRMed::net_serverReceiveTurnOff(SocketClient *receiveClient){
	receiveClient->write("END");
}



//************************THread************************
unsigned int __stdcall KUKA::LBRMed::onlyreceiveThread(void * pThis){

	std::string receiveIP = "172.31.1.147";
	int receivePort = 30003;

	try{
		SocketClient *receiveClient = new SocketClient(receiveIP, receivePort);
		Sleep(300);

		while (KUKA::endflag_ == false)
		{

			getEEFPos(receiveClient);
			getEEFForce(receiveClient);
			getEEFMoment(receiveClient);
			getJointsExternalTorques(receiveClient);
			getJointsPos(receiveClient);
			getJointsMeasuredTorques(receiveClient);

			Sleep(100);
			KUKA::thread_beginflag_ = true; // thread begain.
		}
		net_serverReceiveTurnOff(receiveClient);
		return 0;
	} catch (const char *s){
		std::cerr << s << std::endl;
		return 0;
	} catch (std::string s){
		std::cerr << s << std::endl;
		return 0;
	} catch (...){
		std::cerr << "unhandled exception\n";
		return 0;
	}
}



//************************UPDATE************************
void  KUKA::LBRMed::updateEEFPos(double EEFpos[6]){
	
	std::string cmd = "UPDATE_EEFPos_" + std::to_string(EEFpos[0]) + "_" + std::to_string(EEFpos[1]) + "_" + 
											std::to_string(EEFpos[2]) + "_" + std::to_string(EEFpos[3]) + "_" + 
											   std::to_string(EEFpos[4]) + "_" + std::to_string(EEFpos[5]) + "_";
	this->connection_->write(cmd);

	judgeServerCmdOnce();
}
void  KUKA::LBRMed::updateEEFOffset(double EEFOffset[3]) {
	std::string cmd = "UPDATE_EEFOffset_" + std::to_string(EEFOffset[0]) + "_" + std::to_string(EEFOffset[1]) + "_" +
											   std::to_string(EEFOffset[2]);
	this->connection_->write(cmd);

	judgeServerCmdOnce();
}
void  KUKA::LBRMed::updateEEFPosServo(double EEFpos[6]){

	std::string cmd = "UPDATE_EEFPosServo_" + std::to_string(EEFpos[0]) + "_" + std::to_string(EEFpos[1]) + "_" + 
									             std::to_string(EEFpos[2]) + "_" + std::to_string(EEFpos[3]) + "_" + 
									                std::to_string(EEFpos[4]) + "_" + std::to_string(EEFpos[5]) + "_";
	this->connection_->write(cmd);
}
void  KUKA::LBRMed::updateEEFPosCIRC(double auxiliaryPos[6], double endPos[6]) {

	std::string cmd1 = "UPDATE_EEFPosCirc1_" + std::to_string(auxiliaryPos[0]) + "_" + std::to_string(auxiliaryPos[1]) + "_" +
											      std::to_string(auxiliaryPos[2]) + "_" + std::to_string(auxiliaryPos[3]) + "_" +
												  	 std::to_string(auxiliaryPos[4]) + "_" + std::to_string(auxiliaryPos[5]) + "_";
	this->connection_->write(cmd1);

	std::string cmd2 = "UPDATE_EEFPosCirc2_" + std::to_string(endPos[0]) + "_" + std::to_string(endPos[1]) + "_" +
										  		  std::to_string(endPos[2]) + "_" + std::to_string(endPos[3]) + "_" +
													 std::to_string(endPos[4]) + "_" + std::to_string(endPos[5]) + "_";
	this->connection_->write(cmd2);
}

void  KUKA::LBRMed::updateJointPos(double jointpos[7]){
	//  e.g. "jp_0_0.3491_0_-1.2217_0_1.5708_0_"
	std::string cmd = "UPDATE_jPos_" + std::to_string(jointpos[0]) + "_" + std::to_string(jointpos[1]) + "_" + 
								          std::to_string(jointpos[2]) + "_" + std::to_string(jointpos[3]) + "_" + 
								             std::to_string(jointpos[4]) + "_" + std::to_string(jointpos[5]) + "_" + 
								                std::to_string(jointpos[6]) + "_";
	this->connection_->write(cmd);
	judgeServerCmdOnce();
}
void  KUKA::LBRMed::updateJointPosServo(double jointpos[7]){
	//  e.g. "jp_0_0.3491_0_-1.2217_0_1.5708_0_"
	std::string cmd = "UPDATE_jPosServo_" + std::to_string(jointpos[0]) + "_" + std::to_string(jointpos[1]) + "_" + 
										       std::to_string(jointpos[2]) + "_" + std::to_string(jointpos[3]) + "_" +
							                	  std::to_string(jointpos[4]) + "_" + std::to_string(jointpos[5]) + "_" + 
									                 std::to_string(jointpos[6]) + "_";
	this->connection_->write(cmd);
}

void  KUKA::LBRMed::updateJRelVel(double jRelVel) {
	std::string cmd = "UPDATE_jRelVel_" + std::to_string(jRelVel) + "_";
	this->connection_->write(cmd);
}
void  KUKA::LBRMed::updatecVel(double cVel) {
	std::string cmd = "UPDATE_cVel_" + std::to_string(cVel) + "_";
	this->connection_->write(cmd);
}
void  KUKA::LBRMed::updateJointVelocity(double jvel[7]){
	std::string cmd = "UPDATE_jVel_" + std::to_string(jvel[0]) + "_" + std::to_string(jvel[1]) + "_" + 
									      std::to_string(jvel[2]) + "_" + std::to_string(jvel[3]) + "_" + 
								             std::to_string(jvel[4]) + "_" + std::to_string(jvel[5]) + "_" + 
										        std::to_string(jvel[6]) + "_";
	this->connection_->write(cmd);
	judgeServerCmdOnce();
}
void  KUKA::LBRMed::updateJointVelocityServo(double jvel[7]) {
	std::string cmd = "UPDATE_jVelSerco" + std::to_string(jvel[0]) + "_" + std::to_string(jvel[1]) + "_" +
											  std::to_string(jvel[2]) + "_" + std::to_string(jvel[3]) + "_" +
												 std::to_string(jvel[4]) + "_" + std::to_string(jvel[5]) + "_" +
													std::to_string(jvel[6]) + "_";
	this->connection_->write(cmd);
}


//************************Getters methods************************
// getEEFOrientationQuat
// getEEFOrientationR
void  KUKA::LBRMed::getEEFPos(SocketClient *receiveClient) {

	receiveClient->write("QUERY_EEF_pos");
	std::string msg = receiveClient->read();
	std::vector<std::string> EEFPos = stringProcess::split_vector(msg, "_");
	KUKA::EEFPos_[0] = atof(EEFPos[0].c_str());
	KUKA::EEFPos_[1] = atof(EEFPos[1].c_str());
	KUKA::EEFPos_[2] = atof(EEFPos[2].c_str());
	KUKA::EEFPos_[3] = atof(EEFPos[3].c_str());
	KUKA::EEFPos_[4] = atof(EEFPos[4].c_str());
	KUKA::EEFPos_[5] = atof(EEFPos[5].c_str());

		std::ofstream fout("EEFPos.txt");
		
		fout << atof(EEFPos[0].c_str()) << " ";
		fout << atof(EEFPos[1].c_str()) << " ";
		fout << atof(EEFPos[2].c_str()) << " ";
		fout << atof(EEFPos[3].c_str()) << " ";
		fout << atof(EEFPos[4].c_str()) << " ";
		fout << atof(EEFPos[5].c_str()) << " ";
		fout << std::endl;
		fout.close(); 	
}
void  KUKA::LBRMed::getEEFForce(SocketClient *receiveClient) {

	receiveClient->write("QUERY_EEF_force");
	std::string msg = receiveClient->read();
	std::vector<std::string> EEFForce = stringProcess::split_vector(msg, "_");
	KUKA::EEFForce_[0] = atof(EEFForce[0].c_str());
	KUKA::EEFForce_[1] = atof(EEFForce[1].c_str());
	KUKA::EEFForce_[2] = atof(EEFForce[2].c_str());
}
void  KUKA::LBRMed::getEEFMoment(SocketClient *receiveClient) {

	receiveClient->write("QUERY_EEF_moment");
	std::string msg = receiveClient->read();
	std::vector<std::string> EEFMoment = stringProcess::split_vector(msg, "_");
	KUKA::EEFMoment_[0] = atof(EEFMoment[0].c_str());
	KUKA::EEFMoment_[1] = atof(EEFMoment[1].c_str());
	KUKA::EEFMoment_[2] = atof(EEFMoment[2].c_str());
}
void  KUKA::LBRMed::getJointsExternalTorques(SocketClient *receiveClient) {

	receiveClient->write("QUERY_Torques_ext_J");
	std::string msg = receiveClient->read();
	std::vector<std::string> ExternalTorque = stringProcess::split_vector(msg, "_");
	KUKA::ExternalTorque_[0] = atof(ExternalTorque[0].c_str());
	KUKA::ExternalTorque_[1] = atof(ExternalTorque[1].c_str());
	KUKA::ExternalTorque_[2] = atof(ExternalTorque[2].c_str());
	KUKA::ExternalTorque_[3] = atof(ExternalTorque[3].c_str());
	KUKA::ExternalTorque_[4] = atof(ExternalTorque[4].c_str());
	KUKA::ExternalTorque_[5] = atof(ExternalTorque[5].c_str());
	KUKA::ExternalTorque_[6] = atof(ExternalTorque[6].c_str());
}
void  KUKA::LBRMed::getJointsMeasuredTorques(SocketClient* receiveClient) {

	receiveClient->write("QUERY_Torques_m_J");
	std::string msg = receiveClient->read();
	std::vector<std::string> MeasuredTorque = stringProcess::split_vector(msg, "_");
	KUKA::MeasuredTorque_[0] = atof(MeasuredTorque[0].c_str());
	KUKA::MeasuredTorque_[1] = atof(MeasuredTorque[1].c_str());
	KUKA::MeasuredTorque_[2] = atof(MeasuredTorque[2].c_str());
	KUKA::MeasuredTorque_[3] = atof(MeasuredTorque[3].c_str());
	KUKA::MeasuredTorque_[4] = atof(MeasuredTorque[4].c_str());
	KUKA::MeasuredTorque_[5] = atof(MeasuredTorque[5].c_str());
	KUKA::MeasuredTorque_[6] = atof(MeasuredTorque[6].c_str());
}
void  KUKA::LBRMed::getJointsPos(SocketClient* receiveClient) {

	receiveClient->write("QUERY_JointPosition");
	std::string msg = receiveClient->read();
	std::vector<std::string> jPos = stringProcess::split_vector(msg, "_");
	KUKA::jPos_[0] = atof(jPos[0].c_str());
	KUKA::jPos_[1] = atof(jPos[1].c_str());
	KUKA::jPos_[2] = atof(jPos[2].c_str());
	KUKA::jPos_[3] = atof(jPos[3].c_str());
	KUKA::jPos_[4] = atof(jPos[4].c_str());
	KUKA::jPos_[5] = atof(jPos[5].c_str());
	KUKA::jPos_[6] = atof(jPos[6].c_str());
}
void  KUKA::LBRMed::getSimulateJoints(double target[6], double joints[7]) {

	updateEEFPos(target);
	for (size_t i = 0; i < 6; i++)std::cout << target[i] << std::endl;

	std::cout << "00000000000000" << std::endl;
	this->connection_->write("simuJoints");
	std::cout << "11111111111111" << std::endl;
	std::string msg = this->connection_->read();
	std::cout << "22222222222222" << std::endl;
	std::cout << msg << std::endl;
	if (msg.length() < 8) {

		std::cout << "wrong" << std::endl;
		for (size_t i = 0; i < 7; i++)joints[i] = 110;

		return;
	}

	std::vector<std::string> Joint = stringProcess::split_vector(msg, "_");
	joints[0] = atof(Joint[0].c_str());
	joints[1] = atof(Joint[1].c_str());
	joints[2] = atof(Joint[2].c_str());
	joints[3] = atof(Joint[3].c_str());
	joints[4] = atof(Joint[4].c_str());
	joints[5] = atof(Joint[5].c_str());
	joints[6] = atof(Joint[6].c_str());
	std::cout << "33333333333333" << std::endl;
	std::string ack = this->connection_->read();
}



//************************HandGuiding************************
void  KUKA::LBRMed::startHandGuiding() {
	this->connection_->write("startHandguiding");
	judgeServerCmdOnce();
}
void  KUKA::LBRMed::endHandGuiding(){
	this->connection_->write("endHandGuilding");
	judgeServerCmdOnce();
}



//************************MOTION************************
void  KUKA::LBRMed::movePTPinJStoOrigin(double relVel){

	updateJRelVel(relVel);

	judgeServerCmdOnce();

	double jointPos[7] = { 0.0 };
	updateJointPosServo(jointPos);

	this->connection_->write("MOTION_PTPinJS");

	//judgeServerCmdTwice();
	//	Sleep(1);
}
void  KUKA::LBRMed::movePTPinJS(double jointPos[7], double relVel){

	updateJRelVel(relVel);
	judgeServerCmdOnce();

	updateJointPosServo(jointPos);
	connection_->write("MOTION_PTPinJS");

	//judgeServerCmdTwice();
	//	Sleep(1);
}
void  KUKA::LBRMed::movePTPinCS(double cartPos[6], double vel){

	updateJRelVel(vel);
	judgeServerCmdOnce();

	updateEEFPos(cartPos);
	connection_->write("MOTION_PTPinCS");

	//judgeServerCmdOnce();
	//	Sleep(1);
}
void  KUKA::LBRMed::moveLIN(double Pos[6], double vel){

	updatecVel(vel);
	judgeServerCmdOnce();

	updateEEFPos(Pos);
	this->connection_->write("MOTION_LIN");
	//judgeServerCmdTwice();
	//	Sleep(1);
}
void  KUKA::LBRMed::moveLINRelBase(double Offset[3], double vel){

	updatecVel(vel);
	judgeServerCmdOnce();

	updateEEFOffset(Offset);
	this->connection_->write("MOTION_LINRelBase");
	//judgeServerCmdTwice();
	//	Sleep(1);
}
void  KUKA::LBRMed::moveLINRelEEF(double Offset[3], double vel){

	updatecVel(vel);
	judgeServerCmdOnce();

	updateEEFOffset(Offset);
	this->connection_->write("MOTION_LINRelEEF");
	judgeServerCmdTwice();
	//	Sleep(1);
}
void  KUKA::LBRMed::moveCIRC(double auxiliaryPos[6], double endPos[6], double jRelVel) {

	updateJRelVel(jRelVel);
	judgeServerCmdOnce();

	updateEEFPosCIRC(auxiliaryPos, endPos);
	this->connection_->write("MOTION_CIRC");
	judgeServerCmdTwice();
}

void  KUKA::LBRMed::movePTPTransportPositionJointSpace(double relVel){

	std::string cmd = "jRelVel_" + std::to_string(relVel) + "_"; 
	this->connection_->write(cmd);
	judgeServerCmdOnce();

	double jointpos[7] = { 0.0 };
	jointpos[1] = 0.4363; // 25*pi/180;
	jointpos[3] = 1.5708; // 90*pi/180;
	updateEEFPos(jointpos);
	this->connection_->write("doPTPinJS");
	judgeServerCmdTwice();
	//	Sleep(1);
}



//************************ judge whether the server has received the cmd************************
void  KUKA::LBRMed::judgeServerCmdOnce() {
	//  judge whether the server has received the cmd
	bool flag = false;
	std::string msg;

	while (flag == false) {

		msg = " ";
		msg = connection_->read();

		if (msg == " ") {
			continue;
		} else if (stringProcess::checkAcknowledgment(msg) == 1) {
			flag = true;
		} else if (stringProcess::checkAcknowledgment(msg) == 0) {
			net_serverTurnOff();
			connection_->Close();
			std::cerr << "error happened" << std::endl;
			break;
		}
	}
}
void  KUKA::LBRMed::judgeServerCmdTwice() {
	//  judge whether the server has received the cmd
	bool flag = false;
	std::string msg;
	msg = connection_->read();

	flag = true;
	//  judge whether the server has completed running the cmd
	while (flag == true) {

		msg = " ";
		msg = connection_->read();

		if (msg == " ") {
			continue;
		} else if (stringProcess::checkAcknowledgment(msg) == 1) {
			flag = false;
		} else if (stringProcess::checkAcknowledgment(msg) == 0) {
			net_serverTurnOff();
			connection_->Close();
			std::cerr << "error happened" << std::endl;
			break;
		}
	}
}



////////////////////////////////////////////ÔÝÊ±²»ÓÃ///////////////////////////////////////////////////////



//************************Setters methods************************
void  KUKA::LBRMed::setBlueOff() {
	this->connection_->write("blueOff");
	judgeServerCmdOnce();
}
void  KUKA::LBRMed::setBlueOn() {
	this->connection_->write("blueOn");
	judgeServerCmdOnce();
}
void  KUKA::LBRMed::setPin11Off() {
	this->connection_->write("pin11off");
	judgeServerCmdOnce();
}
void  KUKA::LBRMed::setPin11On() {
	this->connection_->write("pin11on");
	judgeServerCmdOnce();
}
void  KUKA::LBRMed::setPin12Off() {
	this->connection_->write("pin12off");
	judgeServerCmdOnce();
}
void  KUKA::LBRMed::setPin12On() {
	this->connection_->write("pin12on");
	judgeServerCmdOnce();
}
void  KUKA::LBRMed::setPin1Off() {
	this->connection_->write("pin1off");
	judgeServerCmdOnce();
}
void  KUKA::LBRMed::setPin1On() {
	this->connection_->write("pin1on");
	judgeServerCmdOnce();
}
void  KUKA::LBRMed::setPin2Off() {
	this->connection_->write("pin2off");
	judgeServerCmdOnce();
}
void  KUKA::LBRMed::setPin2On() {
	this->connection_->write("pin2on");
	judgeServerCmdOnce();
}
// GetInertiaMatrix5
bool  KUKA::LBRMed::getPin13State() {

	this->connection_->write("getPin13");
	std::string msg = this->connection_->read();

	std::string temp;
	temp.assign(msg, 0, 1);
	if (temp.compare("1") == 1)return true;
	else return 0;
}
bool  KUKA::LBRMed::getPin10State() {

	this->connection_->write("getPin10");
	std::string msg = this->connection_->read();

	std::string temp;
	temp.assign(msg, 0, 1);
	if (temp.compare("1") == 1)return true;
	else return 0;
}
bool  KUKA::LBRMed::getPin16State() {

	this->connection_->write("getPin16");
	std::string msg = this->connection_->read();

	std::string temp;
	temp.assign(msg, 0, 1);
	if (temp.compare("1") == 1)return true;
	else return 0;
}
bool  KUKA::LBRMed::getPin3State() {

	this->connection_->write("getPin3");
	std::string msg = this->connection_->read();

	std::string temp;
	temp.assign(msg, 0, 1);
	if (temp.compare("1") == 1)return true;
	else return 0;
}
bool  KUKA::LBRMed::getPin4State() {

	this->connection_->write("getPin4");
	std::string msg = this->connection_->read();

	std::string temp;
	temp.assign(msg, 0, 1);
	if (temp.compare("1") == 1)return true;
	else return 0;
}



//
//        //************************General porpuse methods************************
//        gen_CentrifugalMatrix
//        gen_CoriolisMatrix
//        gen_DirectDynamics
//        directKinematics
//        gen_DirectKinematics
//        gen_GravityVector
//        gen_InverseDynamics
//        gen_InverseKinematics
//        gen_MassMatrix
//        gen_NullSpaceMatrix
//        gen_partialJacobean
//        getDHMatrix
//        normalizeColumns



//         //************************Interaction functions************************
//         moveWaitForDTWhenInterrupted
//         performEventFunctionAtDoubleHit



//         startPreciseHandGuiding
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
//         movePTPCirc1OrintationInter
//         movePTPCirc1OrintationInterCheck
//



//                                  //  % PTP ellipse
//         movePTPEllipse
//         movePTPEllipse_XY
//         movePTPEllipse_XZ
//         movePTPEllipse_YZ
//                                   //   % conditional
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
//         realTime_startDirectServoCartesian
//         realTime_startDirectServoJoints
//         realTime_startImpedanceJoints
//         realTime_startVelControlJoints
//         realTime_stopDirectServoCartesian
//         realTime_stopDirectServoJoints
//         realTime_stopImpedanceJoints
//         realTime_stopVelControlJoints