#include <iostream>
#include <process.h>
#include "stdafx.h"
#include "RobotApplication.h"
#include "globalVars.h"

int main()
{
	KUKA::LBRMed MEDBOT("172.31.1.147", 30001);
	MEDBOT.initialize();

	_beginthreadex(NULL, 0, KUKA::LBRMed::onlyreceiveThread, NULL, 0, NULL);
	MEDBOT.clientConnectionEstablish();

	do {
		Sleep(100); 
	} while (KUKA::thread_beginflag_ == false);//do nothing,just wait



	//*****************************************HandGuiding*****************************************
	//int k;
	//std::cout << "step1: To start HandGuilding, please input 1 : ";
	//std::cin >> k;

	//Sleep(3000);//留出时间去扶KUKA

	//while (k != 1) {
	//	std::cout << "input 1 to start HandGuiding" << std::endl;
	//	std::cin >> k;
	//}
	//MEDBOT.startHandGuiding();

	//Sleep(1000);

	//std::cout << "step2: To end HandGuiding, please input 0: ";
	//std::cin >> k;
	//while (k != 0) {
	//	std::cout << "input 0 to end HandGuiding." << std::endl;
	//	std::cin >> k;
	//}
	//MEDBOT.endHandGuiding();
	//Sleep(10000);
	


	////*****************************************PTPLIN*****************************************
	//int j;
	//std::cout << "step3: To start LIN, please input 2: ";
	//std::cin >> j;
	//while (j != 2) {
	//	std::cout << "input 2 to start LIN." << std::endl;
	//	std::cin >> j;
	//}

	//double EEFPos[6] = { 0.0 };
	//std::cout << "please input the end position of end effector:";
	//for (int i = 0; i < 6; ++i) 
	//	std::cin >> EEFPos[i];

	//double vel = 0.0;
	//std::cout << "please input velocity:";
	//std::cin >> vel;

	//MEDBOT.moveLIN(EEFPos, vel);



	////*****************************************PTPinJS*****************************************
	//int m;
	//std::cout << "step4: To start PTPinJS, please input 3:";
	//std::cin >> m;
	//while (m != 3) {
	//	std::cout << "input 3 to start PTPinJS.";
	//	std::cin >> m;
	//}
	//
	//double jointpos[7] = { 0.0 };
	//std::cout << "please input the end joint position:";
	//for (int i = 0; i < 7; ++i)
	//	std::cin >> jointpos[i];//reference:jointpos[7] = { 0, 0.3491, 0, -1.2217, 0, 1.5708, 0 }

	//double relVel = 0.0;
	//std::cout << "please input relative velocity:";
	//std::cin >> relVel;
	//
	//MEDBOT.movePTPinJS(jointpos, relVel);



	////*****************************************PTPinCS*****************************************
	/*int h;
	std::cout << "step5: To start PTPinCS, please input 5:";
	std::cin >> h;
	while (h != 5) {
		std::cout << "input 5 to start PTPinCS.";
		std::cin >> h;
	}
	
	double cartPos[6] = { 0.0 };
	std::cout << "please input the end cartPos:";
	for (int i = 0; i < 6; ++i)
		std::cin >> cartPos[i];

	double jRelVel = 0.0;
	std::cout << "please input joint relative velocity:";
	std::cin >> jRelVel;
	
	MEDBOT.movePTPinCS(cartPos, jRelVel);
	
*/

	////*****************************************PTPinJStoOrigin*****************************************
	//int n;
	//std::cout << "step5: To move PTP in joint space to the origin, please input 4:";
	//std::cin >> n;
	//while (n != 4) {
	//	std::cout << "input 4 to movePTPinJStoOrigin.";
	//	std::cin >> n;
	//}
	//
	//double jrelVel;
	//std::cout << "please input the joint relative velocity:";
	//std::cin >> jrelVel;

	//MEDBOT.movePTPinJStoOrigin(jrelVel);




	////*****************************************LINRelBase(EEF)*****************************************
	//Sleep(1000);

	//double Offset1[3] = { 50, 50, 50 };
	//double cVel1 = 10;
	//MEDBOT.moveLINRelBase(Offset1, cVel1);

	//Sleep(5000);

	//double Offset2[3] = { 50, 50, 50 };
	//double cVel2 = 10;
	//MEDBOT.moveLINRelEEF(Offset2, cVel2);



	////*****************************************CIRC*****************************************
	//Sleep(1000);

	//double auxiliaryPos[6] = {610, 0, 400, 3.1415926535, 0, 3.1415926535 };
	//double endPos[6] = {610, 0, 600,3.1415926535, 0, 3.1415926535 };
	//double jRelVelcirc = 0.1;
	//MEDBOT.moveCIRC(auxiliaryPos, endPos, jRelVelcirc);




	//*****************************************Servo*****************************************
	MEDBOT.startServoMode();

	double endPos1[6] = { 610, 200, 400, 3.1415, 0.5, 3.1415 }; 
	MEDBOT.moveLINServo(endPos1, 0.1);

	Sleep(10000);

	/*double endPos2[6] = { 500, 200, 400, 3.14 , 0, 3.14 };
	MEDBOT.moveLINServo(endPos2, 0.2);*/

	MEDBOT.endServoMode();



	///turn off the server	
	MEDBOT.net_serverTurnOff();

	system("pause");

}
