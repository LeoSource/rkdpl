#include "RobotParameter.h"

using namespace std;
using nlohmann::json;

//全局变量改为放在.cpp里，GlobalParams.h里去申明；
double l1 = 0.111, l2 = 0.0935, l3 = 0.066, l4 = 0.068;
double h = 0.403, w = 0.518;
double g_yaw0_x0 = l2-l3;
double g_dh[] = { 0,0,0,0,
                pi/2,0,0,0,
                0,l2,-l1,pi/2,
                pi/2,0,0,pi/2,
                0,-l4,-l3,pi/2 };
int g_type[] = { 0,1,0,1,0};
double g_offset[] = { 0, h, pi/2, w, 0 };
double g_qlimit[] = { -pi/2, pi/2, 0.01, 0.78, -pi/2, pi/2, 0.01, 0.53, -2*pi, 2*pi};
double g_cycle_time = 0.005;
double g_stowed_jpos[] = { 0,0.6,0,0,0 };
double g_stowed_cpos[] = { 0,0.7,0.9 };
double g_return_err[] = { 0.01, 0.007, 0.01, 0.007, 0.01, 0.01 };

double g_jvmax_default[] = {pi/8, pi/8, 0.8*pi/4, pi/4, pi/4, pi/4};
double g_jamax_default[] = {pi/4, pi/4, 2 * pi/4, pi/4, pi/4, pi/4};
double g_cvmax_default[] = {0.4, 0.6};
double g_camax_default[] = {0.8, 1.2};

double g_jvmax[] = {pi/8*4, pi/8*4, 0.8*pi/4*4, pi/4*4, pi/4*4, pi/4*4};
double g_jamax[] = {pi/4*4, pi/4*4, 2 * pi/4*4, pi/4*4, pi/4*4, pi/4*4};
double g_jjmax[] = { pi / 4 * 4*2, pi / 4 * 4*2, 2 * pi / 4 * 4*2, pi / 4 * 4*2, pi / 4 * 4*2, pi / 4 * 4*2 };
double g_cvmax[] = {0.4, 0.6};
double g_camax[] = {0.8, 1.2};
double g_cjmax[] = { 0.8*2, 1.2*2 };

double g_tool_mirror[] = {0, 0, 0.39, 0.0 *pi/180, 0, 0};
double g_tool_table[] = {0, 0, 0.39, 0.0 *pi/180, 0, 0};
double g_tool_water_press[] = {0, 0, 0.29, 0.0 *pi/180, 0, 0};
double g_tool_toilet[] = {0, 0, 0.47, 0.0 *pi/180, 0, 0};
double g_tool_toilet_outer_ring[] = {0, 0, 0.47, 0.0 *pi/180, 0, 0};

double g_tool_default[] = {0, 0, 0.215, 0, 0, 0};
double g_tool_current[] = {0, 0, 0.215, 0, 0, 0};

double a3 = 0.51, a4 = 0.51, d1 = 0.048, d4 = 0.11, d5 = 0.08662, d6 = 0.035;
double g_dh_ur[] = { 0,  d1,  0,  0,
0,  0,  0,  -pi / 2,
0,  0,  a3,  0,
0,  d4,  a4, 0,
0,  d5,  0,  -pi / 2,
0,  d6,  0,  pi / 2 };
int g_type_ur[] = { 0,0,0,0,0,0 };

double g_offset_ur[] = { 0, -pi / 2, pi / 2, -pi / 2, 0, 0 };
double g_qlimit_ur[] = { -3*pi/2.0, 3*pi/2.0, -pi/2, 160.0*pi/180.0,
					-250.0*pi/180.0, 75*pi/180.0, -pi, pi, -pi, pi,-2 * pi, 2 * pi };

double g_barycenter_params[] = { 3.63014318813,
0.0856625559067,
1.76927811464,
0.0436696114909,
-0.659605979876,
0.0126419556228,
0.0794358297374,
0.242725858588,
0.00665947433362,
-0.0693229602896,
0.0794358297374,
0.24960659016,
-0.0170509551675,
-0.0366932034799,
0.0693229602896,
0.25203289377
};

double g_fric_params[] = { 5.9438,12.0192,5.9438,12.0192,5.9438,12.0192,
3.8875,4.5064,3.8875,4.5064,3.8875,4.5064 };

double g_grav_scale[] = { 1,1,1,1,1,1 };
double g_fric_scale[] = { 0.5,0.5,0.5,0.5,0.5,0.5 };
double g_tau_external_base[] = { 10,10,10,6,6,3 };
double g_tau_external_limit[] = { 15,15,15,7,7,7 };
double g_encoder_offset[] = { 91211, 479800, 390552, 190190, 339732,489121 };
double g_encoder_resolution[] = { 524288,524288,524288,524288,524288,524288 };
double g_encoder_dir[] = { 1, 1, -1, 1, 1,1 };
double g_rated_torque[] = { 52, 52, 52, 10, 10, 10 };//Nm
double g_rated_current[] = { 18, 18, 18, 6, 6, 6 };//A
double g_torque_coeff[] = { 0.088,0.088,0.088,0.075,0.075,0.075 };
double g_gear_ratio[] = { 101,101,101,101,101,101 };
double g_gear_efficiency[] = { 0.68, 0.68, 0.68, 0.68, 0.68, 0.68 };
int g_encoder_turns[] = { 1,1,1,1,1,1 };

vector<vector<double>> g_dyn_iden_params_half1hz{ {0} };
vector<vector<double>> g_dyn_iden_params_1hz{ {0} };

//测试读取json文件修改默认全局参数
namespace RobotParameter
{
	void readRobotDynamicsFileJson(const std::string filename)
	{
		ifstream in(filename, ios::binary);
		if (!in.is_open())
		{
			std::cout<<"Error opening robot dynamics file"<<std::endl;
			return;
		}
		auto j = json::parse(in);
		j.at("gravity_parameters").get_to(g_barycenter_params);
		j.at("friction_parameters").get_to(g_fric_params);
		j.at("gravity_scale").get_to(g_grav_scale);
		j.at("friction_scale").get_to(g_fric_scale);
		std::cout<<"load robot dynamics parameters successfully"<<std::endl;
	}

	void readEncoderTurnsFileJson(const std::string filename)
	{
		ifstream in(filename, ios::binary);
		if (!in.is_open())
		{
			std::cout<<"Error opening encoder turns file"<<std::endl;
			return;
		}
		auto j = json::parse(in);
		j.at("encoder_turns").get_to(g_encoder_turns);
		in.close();
		std::cout<<"load encoder turns parameters successfully"<<std::endl;
	}

	void writeEncoderTurnsFileJson(const std::string filename, int* encoder_turns)
	{
		ofstream ofile;
		ofile.open(filename, ios::out|ios::trunc);
		if (!ofile.is_open())
		{
			std::cout<<"Error loading encoder turns file"<<std::endl;
			return;
		}
		json j = { {"encoder_turns",{ encoder_turns[0],encoder_turns[1],
			encoder_turns[2],encoder_turns[3],encoder_turns[4],encoder_turns[5] } } };
		ofile<<j<<std::endl;
		ofile.close();
		std::cout<<"update encoder turns parameters successfully"<<std::endl;
	}
}

