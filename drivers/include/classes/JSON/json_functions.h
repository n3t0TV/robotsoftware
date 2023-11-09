#include "libraries/json.hpp"
#include <iostream>
#include <fstream>
using json = nlohmann::json;

json OpenJSONFile(const char * file)
{ 
	json j;
	std::ifstream i(file);
    if (i.is_open())
    {
		if (j.accept(i))
		{
			std::ifstream i(file);
			json j;
			i >> j;
			return j;
		}
		else
			ROS_ERROR("Invalid JSON file");
	}
	else
		ROS_ERROR("This File cannot be opened");
	return j;
}
