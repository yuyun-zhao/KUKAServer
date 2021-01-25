#pragma warning(disable:4267)
#pragma warning(disable:4715)
#pragma warning(disable:4172)
#include "stdafx.h"
#include "stringProcess.h"
#include <iostream>


std::map<int, std::vector<std::string>> &stringProcess::split_map(std::string str, std::string spacer)
{
	// fisrtly, split receive buffer data to string map
	std::vector<std::string> v;
	std::map<int, std::vector<std::string>> m;
	v.clear();
	m.erase(m.begin(), m.end());
	int count = 0;

	int pos1, pos2;
	int len = spacer.length(); //记录分隔符的长度
	pos1 = 0;
	pos2 = str.find(spacer);
	while (pos2 != std::string::npos)
	{
		if (pos2 != 0)
		{
			v.push_back(str.substr(pos1, pos2 - pos1));
			m[count] = v;
			v.pop_back();
			count++;
		}

		pos1 = pos2 + len;
		pos2 = str.find(spacer, pos1); // 从str的pos1位置开始搜寻spacer
	}
	if (pos1 != str.length())
	{
		//分割最后一个部分
		v.push_back(str.substr(pos1));
		m[count] = v;
		v.pop_back();
	}
	return m;
}

std::vector<std::string> stringProcess::split_vector(std::string str, std::string spacer)
{
	// secendly, split string map data to string vector
	std::vector<std::string> v;

	int pos1, pos2;
	int len = spacer.length(); //记录分隔符的长度
	pos1 = 0;
	pos2 = str.find(spacer);
	while (pos2 != std::string::npos)
	{
		if (pos2 != 0)
		{
			v.push_back(str.substr(pos1, pos2 - pos1));
			pos1 = pos2 + len;
			pos2 = str.find(spacer, pos1); // 从str的pos1位置开始搜寻spacer
		}
		else
		{
			pos1 = pos2 + len;
			pos2 = str.find(spacer, pos1); // 从str的pos1位置开始搜寻spacer
		}
	}
	if (pos1 != str.length()) //分割最后一个部分
		v.push_back(str.substr(pos1));

	//string  to  double

	return v;
}


bool stringProcess::checkAcknowledgment(std::string message){

	std::string ack = "done";
	std::string nack = "error";
	int length = message.length();

	//when message==nack
	if (length == 6) {
		std::string tmp;
		tmp.assign(message, length - 6, 5);
		if (tmp.compare(nack) == 0){
		
			std::cout << "Error, robot can not perform the operation, Check the touchpad of the robot for more info" << std::endl;
			return false;
		}
	}
	//when message==ack
	else if (length == 5) {
		std::string tmp_ak;
		tmp_ak.assign(message, length - 5, 4);

		if (tmp_ak.compare(ack) == 0)return true;	
	}
	//else
		//std::cout << "unknown message:" << message << "!"<<std::endl;
}
