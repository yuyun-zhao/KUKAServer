#pragma warning(disable:4267)
#pragma warning(disable:4996)
#include "stdafx.h"
#include "Socket.h"
#include <iostream>
#include <cstring>

int Socket::nofSockets_ = 0;

void Socket::Start(){

	if (!nofSockets_){
		WSADATA info;
		if (WSAStartup(MAKEWORD(2, 0), &info))throw "Could not start WSA";	
	}
	++nofSockets_;
}

void Socket::End(){
	WSACleanup();
}

Socket::Socket() : s_(0){

	Start();
	// UDP: use SOCK_DGRAM instead of SOCK_STREAM
	s_ = socket(AF_INET, SOCK_STREAM, 0);

	if (s_ == INVALID_SOCKET)throw "INVALID_SOCKET";

	refCounter_ = new int(1);
}

Socket::Socket(SOCKET s) : s_(s){
	Start();
	refCounter_ = new int(1);
};

Socket::~Socket(){

	if (!--(*refCounter_)){
		Close();
		delete refCounter_;
	}

	--nofSockets_;
	if (!nofSockets_) End();
}

Socket::Socket(const Socket &o){

	refCounter_ = o.refCounter_;
	(*refCounter_)++;
	s_ = o.s_;

	nofSockets_++;
}

Socket &Socket::operator=(Socket &o)
{
	(*o.refCounter_)++;

	refCounter_ = o.refCounter_;
	s_ = o.s_;

	nofSockets_++;

	return *this;
}

void Socket::Close(){
	closesocket(s_);
}

std::string Socket::read(){ 

	/*std::string ret = " ";*/
	// recv函数接收数据到缓冲区buffer中
	char buffer[200] = {'0'};
	//memset(&buffer, 0, sizeof(buffer));
	if (recv(s_, buffer, 200, 0) < 0){
		perror("Server Recieve Data Failed:");
		//  break;
	}
	/*ret = buffer;	
	return ret;*/
	return buffer;
}

void Socket::write(std::string s){ 

	int length = s.length();
	s = s + (char)(10);      //'/n'ASCII码为10

	const char *buf = s.c_str();
	send(s_, buf, strlen(buf), 0);
}

SocketServer::SocketServer(int port, int connections, TypeSocket type)
{
	sockaddr_in sa;

	memset(&sa, 0, sizeof(sa));

	sa.sin_family = PF_INET;
	sa.sin_port = htons(port);
	s_ = socket(AF_INET, SOCK_STREAM, 0);
	if (s_ == INVALID_SOCKET)
	{
		throw "INVALID_SOCKET";
	}

	if (type == NonBlockingSocket)
	{
		u_long arg = 1;
		ioctlsocket(s_, FIONBIO, &arg);
	}

	/* bind the socket to the internet address */
	if (bind(s_, (sockaddr *)&sa, sizeof(sockaddr_in)) == SOCKET_ERROR)
	{
		closesocket(s_);
		throw "INVALID_SOCKET";
	}

	listen(s_, connections);
}

Socket *SocketServer::Accept()
{
	SOCKET new_sock = accept(s_, 0, 0);
	if (new_sock == INVALID_SOCKET)
	{
		int rc = WSAGetLastError();
		if (rc == WSAEWOULDBLOCK)
		{
			return 0; // non-blocking call, no request pending
		}
		else
		{
			throw "Invalid Socket";
		}
	}

	Socket *r = new Socket(new_sock);
	return r;
}

SocketClient::SocketClient(const std::string &host, int port) : Socket()
{
	std::string error;

	hostent *he;
	if ((he = gethostbyname(host.c_str())) == 0){
		error = strerror(errno);
		throw error;
	}

	sockaddr_in addr;
	addr.sin_family = AF_INET;
	addr.sin_port = htons(port);
	addr.sin_addr = *((in_addr *)he->h_addr);
	memset(&(addr.sin_zero), 0, 8);

	if (::connect(s_, (sockaddr *)&addr, sizeof(sockaddr)) == SOCKET_ERROR){
		error = strerror(WSAGetLastError());
		throw error;
	}
}

SocketSelect::SocketSelect(Socket const *const s1, Socket const *const s2, TypeSocket type)
{
	FD_ZERO(&fds_);
	FD_SET(const_cast<Socket *>(s1)->s_, &fds_);
	if (s2)
	{
		FD_SET(const_cast<Socket *>(s2)->s_, &fds_);
	}

	TIMEVAL tval;
	tval.tv_sec = 0;
	tval.tv_usec = 1;

	TIMEVAL *ptval;
	if (type == NonBlockingSocket)
	{
		ptval = &tval;
	}
	else
	{
		ptval = 0;
	}

	if (select(0, &fds_, (fd_set *)0, (fd_set *)0, ptval) == SOCKET_ERROR)
		throw "Error in select";
}

bool SocketSelect::Readable(Socket const *const s)
{
	if (FD_ISSET(s->s_, &fds_))
		return true;
	return false;
}
