#pragma once

#include <vector>
#include <map>
#include <iterator>
#include <sstream>
#include <string>

class stringProcess
{

private:
	// fisrtly, split receive buffer data to string map
	std::map<int, std::vector<std::string>> &split_map(std::string str, std::string spacer);

public:
	// secendly2, split string map data to string vector
	static	std::vector<std::string> split_vector(std::string str, std::string spacer);

	static bool checkAcknowledgment(std::string message); //check the return message from server
};
