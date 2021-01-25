#pragma once
#include <string>
#include <WinSock2.h>

namespace KUKA {
	class SocketClient{
		
	private:
		SOCKET clientSocket;
		const char* IP;
		int _port;
	public:
		SocketClient(std::string IP,int _port);
		~SocketClient() {};

		void connectSocket();
		void SendLine(std::string s);
		std::string ReceiveLine();
		void diconnect();
	};
}

