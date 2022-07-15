#pragma once

#include "RobotMath.h"
#include <vector>

extern double l1, l2, l3, l4;
extern double h, w;
extern double g_yaw0_x0;
extern double g_dh[];
extern int g_type[];
extern double g_offset[];
extern double g_qlimit[];
extern double g_cycle_time ;
extern double g_stowed_jpos[];
extern double g_stowed_cpos[] ;
extern double g_return_err[] ;

extern double g_jvmax_default[];
extern double g_jamax_default[];
extern double g_cvmax_default[];
extern double g_camax_default[];

extern double g_jvmax[];
extern double g_jamax[];
extern double g_jjmax[];
extern double g_cvmax[];
extern double g_camax[];
extern double g_cjmax[];

//theta d a alpha
extern double a3, a4, d1, d4, d5, d6;
extern double g_dh_ur[];
extern int g_type_ur[];
extern double g_offset_ur[];
extern double g_qlimit_ur[];

extern double g_barycenter_params[];
extern double g_fric_params[];
extern double g_grav_scale[];
extern double g_fric_scale[];
extern double g_tau_external_base[];
extern double g_tau_external_limit[];
extern double g_encoder_offset[];
extern double g_encoder_resolution[];
extern double g_encoder_dir[];
extern double g_rated_torque[];
extern double g_rated_current[];
extern double g_torque_coeff[];
extern double g_gear_ratio[];
extern double g_gear_efficiency[];
extern int g_encoder_turns[];

extern std::vector<std::vector<double>> g_dyn_iden_params_half1hz;
extern std::vector<std::vector<double>> g_dyn_iden_params_1hz;