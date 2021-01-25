#include "stdafx.h"
#include "SocketClient.h"
#include <iostream>
#pragma comment(lib, "ws2_32.lib")  //加载 ws2_32.dll

KUKA::SocketClient::SocketClient(std::string ip, int port)
{
	this->IP = ip.c_str();
	this->_port = port;
	//创建套接字
	
}


//LBRMed::SocketClient::~SocketClient()
//{
//	//关闭套接字
//	closesocket(clientSocket);
//
//	//终止使用 DLL
//	WSACleanup();
//}

void KUKA::SocketClient::connectSocket() {

	WORD sockVersion = MAKEWORD(2, 2);
	WSADATA data;
	WSAStartup(sockVersion, &data);
	 clientSocket = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
		if (clientSocket == INVALID_SOCKET) {
			std::cout << "Socket error" << std::endl;
		}
		sockaddr_in sock_in;
		sock_in.sin_family = AF_INET;
		sock_in.sin_port = htons(_port);
		sock_in.sin_addr.S_un.S_addr = inet_addr(IP);
		if (connect(clientSocket, (sockaddr*)&sock_in, sizeof(sock_in)) == SOCKET_ERROR) {
			std::cout << "Connect error" << std::endl;
		}

}

void KUKA::SocketClient::SendLine(std::string s) {

	int length = s.length();
	s = s + (char)(10);      //s.push_back('/n');

	const char *buffff = s.c_str();
	send(clientSocket, buffff, strlen(buffff), 0);

}

std::string KUKA::SocketClient::ReceiveLine(){

	std::string ret = " ";
	// recv函数接收数据到缓冲区buffer中
	char buffer[1000];
	memset(&buffer, 0, sizeof(buffer));
	if (recv(clientSocket, buffer, 1000, 0) < 0)
	{
		perror("Server Recieve Data Failed:");
		//  break;
	}
	ret = buffer;
	return ret;

}

void KUKA::SocketClient::diconnect() {

}
