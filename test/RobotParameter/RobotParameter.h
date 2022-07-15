#pragma once

#include <string>
#include <fstream>
#include <iostream>
#include "../GlobalParams.h"
#include "../json.hpp"

extern double g_tool_mirror[];
extern double g_tool_table[];
extern double g_tool_water_press[];
extern double g_tool_toilet[];
extern double g_tool_toilet_outer_ring[];

extern double g_tool_default[];
extern double g_tool_current[];

namespace RobotParameter
{
	void readRobotDynamicsFileJson(const std::string filename);
	void readEncoderTurnsFileJson(const std::string filename);
	void writeEncoderTurnsFileJson(const std::string filename,int* encoder_turns);
}
