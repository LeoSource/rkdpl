#include <iostream>
#include <iomanip>
#include <fstream>
#include <algorithm>
#include <time.h>
#include <map>
#include <thread>
#include "json.hpp"
#include "GlobalParams.h"
#include "TaskTrajPlanner.h"
#include "PlanTools.h"
#include "ForceControl.h"
#include "DynIdenTrajPlanner.h"
#include "JointDataProcessor.h"
#include "MirrorCleanPlanner.h"
#include "LspbPlanner.h"
#ifdef _WIN32
#include <direct.h>
#include <io.h>
#include "RobotParameter\RobotParameter.h"
#elif __linux__
#include <sys/stat.h>
#include <unistd.h>
#include "RobotParameter/RobotParameter.h"
#endif

#pragma warning(disable : 4996)
#define lzx
//#define wyc
//#define xnd

using namespace std;
using namespace Eigen;
using nlohmann::json;

char operation[10] = {};

enum ID_test
{
	dhmodel = 0,
	jtrajpoly = 1,
	jtrajlspb = 2,
	ctrajline = 3,
	ctrajarctrans = 4,
	ctrajpoly = 5,
	ctrajbspline = 6,
	jacobian = 7,
	iksolver = 8,
	bspline = 9,
	other = 10,
	simulation = 11,
	dynamics = 12,
	fricidentraj = 13,
	dynidentraj = 14,
	jdataprocessor = 15,
	cleanmirror = 16,
	planner = 17
};

using PtrTest = void(*)();
struct TestInfo
{
	ID_test testID;
	PtrTest testfunc;
};


UrRobot CreatUrRobot()
{
	Matrix<double, 6, 4, RowMajor> mdh_table(&g_dh_ur[0]);
	Matrix<int, 6, 1> jnt_type(&g_type_ur[0]);
	Matrix<double, 6, 1> offset(&g_offset_ur[0]);
	RobotTools::Pose pose_tool;
	//pose_tool.rot = RobotTools::RotX(-30*D2R);
	//pose_tool.pos<<0, 0.058, 0.398;
	pose_tool.rot = RobotTools::RotX(0);
	pose_tool.pos<<0, 0, 0;
	Matrix<double, 6, 2, RowMajor> qlimit(&g_qlimit_ur[0]);
	UrRobot rbt(mdh_table, jnt_type, offset, pose_tool);
	rbt.SetJntLimit(qlimit);

	return rbt;
}

void TestLspb()
{
	//LspbPlanner lspb(-10, 3, 4, 3, Vector2d(-2, 3));
	LspbPlanner lspb(0, 0.5, 0.5, Vector2d(0.5, 0));
	string file_name = "F:/0_project/rkdpl/mirrortask_jpos1.csv";
	ofstream ofile;
	ofile.open(file_name, ios::out|ios::trunc);
	if (!ofile.is_open())
	{
		cout<<"failed to open the file"<<endl;
	}
	else
	{
		for (int idx = 0; idx<2000; idx++)
		{
			double t = idx*g_cycle_time;
			auto avp = lspb.GenerateMotion(t);
			ofile<<avp.pos<<","<<avp.vel<<","<<avp.acc<<endl;
		}
	}
}

void TestBSpline()
{
	double tf = 60;
	double dt = 0.01;
	int np = (int)(tf/dt)+1;
	int npts[] = { 15,12,10,8,6,4 };
	double center[] = { 0,0,0,0,0,0, 0,0,0,0,0,0, 0,-0.06,-0.12,-0.18,-0.24,-0.3 };
	double elli_params[] = { 0.7, 0.6, 0.5, 0.4, 0.3, 0.2, 0.5, 0.4, 0.3, 0.2, 0.1, 0.05 };
	Matrix<double, 3, 6, RowMajor> origin(center);
	Matrix<double, 2, 6, RowMajor> elli(elli_params);
	MatrixXd via_pos;
	via_pos.setZero(3, 61);
	vector<Vector3d> vp;
	for (int midx = 0; midx<sizeof(npts)/sizeof(npts[0]); midx++)
	{
		VectorXd theta;
		theta.setLinSpaced(npts[midx]+1, 0, 2*pi);
		for (int nidx = 0; nidx<theta.size(); nidx++)
		{
			Vector3d tmp_pos;
			tmp_pos<<elli(0, midx)*cos(theta(nidx)), elli(1, midx)*sin(theta(nidx)), 0;
			tmp_pos += origin.col(midx);
			vp.push_back(tmp_pos);
		}
	}
	for (int idx = 0; idx<vp.size(); idx++)
	{
		via_pos.col(idx) = vp[idx];
	}
	//MatrixXd via_pos;
	//via_pos.setZero(3, 5);
	//via_pos<<1, 2, 3, 4, 5, 2, 3, -3, 4, 5, 0, 0, 0, 0, 0;
	//CubicBSplinePlanner bspline(&via_pos, "approximation", 60);
	//cout<<bspline.CalcBSplineCoeff(3,5,0.8)<<endl;
	//CLineAVP avp = bspline.GenerateMotion(0.1, 0.5, 0.1);
	//cout<<avp.pos<<endl;
	//cout<<avp.vel<<endl;
	//cout<<avp.acc<<endl;
	const char* file_name = "C:/00Work/01projects/XProject/src/data/test_bspline.csv";
	ofstream ofile;
	ofile.open(file_name, ios::out|ios::trunc);
	if (!ofile.is_open())
	{
		cout<<"failed to open the file"<<endl;
	}
	else
	{
		for (int idx = 0; idx<np; idx++)
		{
			Vector6d cpos;
			//bspline.GeneratePath(cpos);

			ofile<<cpos<<endl;
		}
	}
	cout<<"succeed to save the data"<<endl;

}

void Testother()
{


	std::string folder_name = "test_folder";
#ifdef _WIN32
	system(("md "+folder_name).c_str());
	int flag = access(folder_name.c_str(), 0);
	system(("rd "+folder_name).c_str());
	flag = access(folder_name.c_str(), 0);
	mkdir(folder_name.c_str());
	flag = access(folder_name.c_str(), 0);
	rmdir(folder_name.c_str());
	flag = access(folder_name.c_str(), 0);
#elif __linux__
	system(("mkdir "+folder_name).c_str());
	int flag = access(folder_name.c_str(), 0);
	system(("rm -rf "+folder_name).c_str());
	flag = access(folder_name.c_str(), 0);
	mkdir(folder_name.c_str(), S_IRUSR|S_IWUSR|S_IXUSR|S_IRWXG|S_IRWXO);
	flag = access(folder_name.c_str(), 0);
	rmdir(folder_name.c_str());
	flag = access(folder_name.c_str(), 0);
#endif

	VectorXd qmin;
	qmin.setZero(3);
	qmin.setConstant(1);
	VectorXd qmax;
	qmax.setZero(3);
	qmax.setConstant(5);
	VectorXd q;
	q.setZero(3);
	q<<0, 7, 3;
	cout<<q<<endl;
	MathTools::LimitVector(qmin, &q, qmax);
	cout<<q<<endl;

	int a[2][3] = { 1,2,3,4,5,6 };
	ofstream ofile;
	ofile.open("result.csv", ios::out|ios::trunc);
	ofile<<"First,Second,Third"<<endl;
	for (int i = 0; i < 2; i++)
	{
		for (int j = 0; j < 3; j++)
		{
			ofile<<a[i][j]<<",";
		}
		ofile<<"\n";
	}
	ofile.close();
}

void Input()
{
	while (true)
	{
		cin>>operation;
		std::this_thread::sleep_for(std::chrono::seconds(1));
	}
}

void Simulation()
{
	UrRobot urrbt = CreatUrRobot();
	Vector6d q_fdb;
	q_fdb<<0, 0., 0, -0, -pi/2, 0;//initial joint position
	TaskTrajPlanner tasktraj_planner(&urrbt, q_fdb,g_cycle_time,g_jvmax,g_jamax,g_jjmax, g_cvmax,g_camax, g_cjmax);
	tasktraj_planner._task_completed = true;
	MatrixXd via_pos, vision_pos;
	urrbt.UpdateJntHoldPos(q_fdb);

	fstream ofile;
	time_t timer = time(NULL);
	char time_format[1024];
	strftime(time_format, 1024, "%Y-%m-%d_%H.%M.%S", localtime(&timer));
	string timestring(time_format);
	string file_name = "../data/simulation2_"+timestring+".txt";
	ofile.close();
	ofile.open(file_name, ios::out|ios::trunc);

	std::thread thread_input(Input);

	if (!ofile.is_open())
	{
		cout<<"failed to open the file"<<endl;
	}
	else
	{
		cout<<"start to simulation"<<endl;
		while (true)
		{
			Vector6d q_cmd;
			if (!tasktraj_planner._task_completed)
			{
				if (strcmp(operation, "mirror")==0)
				{
					Pose pose0 = urrbt.FKSolveTool(q_fdb);
					Vector3d rpy0 = RobotTools::Tr2FixedZYX(pose0.rot);
					tasktraj_planner.Reset(q_fdb);
					Vector6d pos_rpy1, pos_rpy2, pos_rpy3, pos_rpy4;
					pos_rpy1<< 0, 0, 0, 0, pi / 2, 0;
					pos_rpy2<<0, pi/2, -pi/2, 0, 0, 0;
					pos_rpy3<< -pi, pi / 2, -pi / 2, 0, 0, 0;
					pos_rpy4 << pi, -pi / 2, pi / 3, pi, pi, pi;
					via_pos.resize(6, 4);
					via_pos<<pos_rpy1, pos_rpy2, pos_rpy3, pos_rpy4;
					tasktraj_planner.AddTraj(&via_pos, eJointSpace, true);
					*operation = {};
				}
				else if (strcmp(operation, "table")==0)
				{

					Pose pose0 = urrbt.FKSolveTool(q_fdb);
					Vector3d rpy0 = RobotTools::Tr2FixedZYX(pose0.rot);
					tasktraj_planner.Reset(q_fdb);
					Vector6d pos_rpy1, pos_rpy2, pos_rpy3, pos_rpy4;
					pos_rpy1<<0.8, 0, 0.23, -pi, pi/6, 0;
					pos_rpy2<<0.8, 0.4, 0.23, -5*pi/6, 0, 0;
					pos_rpy3<<0.4, 0.4, 0.23, -pi, -pi/6, 0;
					pos_rpy4<<0.4, 0, 0.23, -7*pi/6, 0, 0;
					via_pos.resize(6, 4);
					via_pos << pos_rpy1, pos_rpy2, pos_rpy3, pos_rpy4;
					tasktraj_planner.AddTraj(&via_pos, eCartesianSpace, true);
					*operation = {};
				}
				else if (strcmp(operation, "basin")==0)
				{
					tasktraj_planner.Reset(q_fdb);
					vision_pos.resize(6, 2);
					Vector6d pos1, pos2;
					pos1 << 38 * pi / 180, -4 * pi / 180, 0.6 * pi / 180, 46 * pi / 180, -61 * pi / 180, 26 * pi / 180,
						pos2 << 2.7 * pi / 180, -22 * pi / 180, 13 * pi / 180, 58 * pi / 180, -89 * pi / 180, 0.85 * pi / 180;
					vision_pos << pos1, pos2;
					tasktraj_planner.AddTraj(&vision_pos, eJointSpace, true);
					Vector6d pos_rpy1, pos_rpy2, pos_rpy3, pos_rpy4;
					pos_rpy1 << 0.8, 0, 0.23, -pi, pi / 6, 0;
					pos_rpy2 << 0.8, 0.4, 0.23, -5 * pi / 6, 0, 0;
					pos_rpy3 << 0.4, 0.4, 0.23, -pi, -pi / 6, 0;
					pos_rpy4 << 0.4, 0, 0.23, -7 * pi / 6, 0, 0;
					vision_pos.resize(6, 4);
					vision_pos << pos_rpy1, pos_rpy2, pos_rpy3, pos_rpy4;

					tasktraj_planner.AddTraj(&vision_pos, eCartesianSpace, false);
					vision_pos.resize(6, 3);
					vision_pos << pos_rpy1, pos_rpy2, pos_rpy3;
					tasktraj_planner.AddTraj(&vision_pos, eCartesianArc, false);

					*operation = {};
				}
				tasktraj_planner.GenerateJPath(q_cmd);
				urrbt.UpdateJntHoldPos(q_cmd);
			}
			else
			{
				if ((strcmp(operation, "mirror")==0)||(strcmp(operation, "table")==0)||(strcmp(operation, "basin")==0))
					tasktraj_planner._task_completed = false;
				q_cmd = urrbt.HoldJntPos();
				std::this_thread::sleep_for(std::chrono::milliseconds(1));
			}
			q_fdb = q_cmd;
			ofile<<q_cmd(0)<<"\t"<<q_cmd(1)<<"\t"<<q_cmd(2)<<"\t"<<q_cmd(3)<<"\t"<<q_cmd(4)<<"\t"<<q_cmd(5)<<endl;
		}
	}
	cout<<"succeed to simulation and save the data"<<endl;
	thread_input.join();
	ofile.close();
}

void SimulationLZX()
{
	// double a[] = { 1,2,3,4,5 };
	//VectorXd aa;
	//aa.resize(5);
	//aa = Map<Matrix<double, 5, 1> >(a);
	//cout<<aa<<endl;
	UrRobot rbt = CreatUrRobot();
	Vector6d q_fdb;
	//q_fdb<<-5.93262, -17.76558, 10.12871, 6.61926, -92.92648, 6.01913;//initial joint position
	q_fdb << 0, 0, 0, 0, -90, 0;
	q_fdb = q_fdb*D2R;
	TaskTrajPlanner task_planner(&rbt,q_fdb,g_cycle_time,g_jvmax,g_jamax,g_jjmax, g_cvmax,g_camax,g_cjmax);
	task_planner._task_completed = true;
	rbt.UpdateJntHoldPos(q_fdb);
	MatrixXd vp(6, 1);
	vp<<-0.103412, -0.310631, 0.226893, 0.115588, -1.62193, -0.110902;
	task_planner.AddTraj(&vp, eJointSpace, true);

	string file_name = "../data/mirrortask_jpos1.csv";
	ofstream ofile;
	ofile.open(file_name, ios::out|ios::trunc);
	std::thread thrad_input(Input);
	int toilet_state = 0;

	if (!ofile.is_open())
	{
		cout<<"failed to open the file"<<endl;
	}
	else
	{
		cout<<"start to simulation"<<endl;
		while (true)
		{
			Vector6d q_cmd;
			Vector6d cpos_cmd;
			if (!task_planner._task_completed)
			{
				MatrixXd vision_pos, via_posrpy;
				if (strcmp(operation, "toiletlid")==0)
				{
					rbt.UpdateTool(Vector3d(0, -0.035, 0.23), Vector3d(0, 0, 0));
					Vector3d a, b, c;
					string toilet_lid = "open";
					if (toilet_lid=="open")
					{
						a<<0.89, 0.178, -0.3627;
						b<<0.87426, -0.19926, -0.36788;
						c<<0.5006, -0.1645, -0.3838;
					}
					else if (toilet_lid=="close")
					{
						a<<0.87, 0.178, -0.3627;
						b<<0.85426, -0.19926, -0.36788;
						c<<0.9232, -0.079565, 0.066379;
					}
					MatrixXd vi_pos;
					vi_pos.resize(3, 3);
					vi_pos<<a, b, c;
					MatrixXd vp;
					MatrixXd tmp_jpos;
					tmp_jpos.resize(6, 1);
					if (toilet_lid=="open")
					{
						// PlanTools::CalcToiletlidPath(vp, &vi_pos, pi/2);
						PlanTools::PlanToiletlidPath(vp,&vi_pos,110*D2R,-110*D2R,90*D2R,0.05);
						tmp_jpos<<-40, 65, 40, -35, -90, 0;
						tmp_jpos = tmp_jpos*D2R;
					}
					else if (toilet_lid=="close")
					{
						PlanTools::PlanToiletlidPath(vp, &vi_pos, -60*D2R, 0*D2R, 150*D2R, 0.05);
						tmp_jpos<<0, -35, 20, 0, -70, 150;
						tmp_jpos = tmp_jpos*D2R;
					}
					task_planner.AddTraj(&tmp_jpos, eJointSpace, true);
					MatrixXd via_p = vp.col(0);
					task_planner.AddTraj(&via_p, eCartesianSpace, false);
					MatrixXd vvip = vp.rightCols(3);
					task_planner.AddTraj(&vvip, eCartesianArc, false);
					*operation = {};
				}
				else if (strcmp(operation, "table")==0)
				{
					Vector3d p1, p2, p3, p4;
					p1<<0.1, -0.5, -0.6;
					p2<<0.1, 0.5, -0.6;
					p3<<0.6, 0.5, -0.6;
					p4<<0.6, -0.5, -0.6;
					vision_pos.resize(3, 4);
					vision_pos<<p1, p2, p3, p4;
					PlanTools::QuadranglePlanner rectplanner;
					rectplanner.PlanGround(via_posrpy, &vision_pos);

					Vector6d posrpy1, posrpy2, posrpy3, posrpy4;
					posrpy1<<0.8, 0, 0.23, -pi, pi/6, 0;
					posrpy2<<0.8, 0.4, 0.23, -5*pi/6, 0, 0;
					posrpy3<<0.4, 0.4, 0.23, -pi, -pi/6, 0;
					posrpy4<<0.4, 0, 0.23, -7*pi/6, 0, 0;
					MatrixXd vpoint(6, 4);
					vpoint<<posrpy1, posrpy2, posrpy3, posrpy4;
					task_planner.AddTraj(&vpoint, eCartesianSpace, true);
					*operation = {};
				}
				else if (strcmp(operation, "mirror")==0)
				{
					rbt.UpdateTool(Vector3d(0, 0.058, 0.398), Vector3d(-30*D2R, 0, 0));
					Vector3d p1, p2, p3, p4;
					p1<<0.78, -0.575, 0.367;
					p2<<0.8247, 0.5257, 0.34;
					p3<<0.8354, 0.557, 0.817;
					p4<<0.8028, -0.591, 0.8663;
					vision_pos.resize(3, 4);
					vision_pos<<p1, p2, p3, p4;
					PlanTools::QuadranglePlanner recplanner;
					recplanner.PlanMirror(via_posrpy, &vision_pos,1);
					//cout<<via_posrpy<<endl;
					task_planner.AddTraj(&via_posrpy, eCartesianSpace, false);
					*operation = {};
				}
				else if (strcmp(operation, "toilet")==0)
				{
					Vector3d p1(0.519909, 0.309211, -0.456355);
					Vector3d p2(0.391413, 0.314306, -0.468476);
					Vector3d p3(0.336517, 0.393079, -0.468602);
					Vector3d p4(0.362503, 0.477856, -0.466479);
					Vector3d p5(0.473135, 0.55325, -0.451316);
					Vector3d p6(0.606719, 0.556308, -0.440232);
					Vector3d p7(0.675789, 0.48262, -0.429367);
					Vector3d p8(0.635475, 0.355141, -0.440339);
					Vector3d p9(0.519909, 0.329211, -0.526355);
					Vector3d p10(0.401413, 0.334306, -0.53);
					Vector3d p11(0.366517, 0.393079, -0.54);
					Vector3d p12(0.392503, 0.477856, -0.536);
					Vector3d p13(0.473135, 0.53325, -0.521316);
					Vector3d p14(0.606719, 0.536308, -0.510232);
					Vector3d p15(0.675789, 0.48262, -0.50);
					Vector3d p16(0.635475, 0.355141, -0.51);
					vision_pos.resize(3, 16);
					vision_pos<<p1, p2, p3, p4, p5, p6, p7, p8, p9, p10, p11, p12, p13, p14, p15, p16;
					MatrixXd vp(3, 8);
					vp<<p1, p2, p3, p4, p5, p6, p7, p8;
					MatrixXd vp1(3, 4), vp2(3, 4), vp3(3, 4);
					vp1<<p1, p2, p3, p4;
					vp2<<p4, p5, p6, p7;
					vp3 = vision_pos.middleCols(8, 4);
					switch (toilet_state)
					{
					case 0:
					{
						task_planner.Reset(q_fdb);
						PlanTools::PlanToiletInlierPath(via_posrpy, &vp, 20*pi/180, "front");
						MatrixXd vpr = via_posrpy.leftCols(1);
						cout<<vpr<<endl;
						task_planner.AddTraj(&vpr, eCartesianSpace, false);
						task_planner.SetPlanningScene(vp, 20*pi/180);
						task_planner.AddTraj(&vp1, eBSpline, true);
						toilet_state += 1;
					}
					break;
					case 1:
					{
						task_planner.Reset(q_fdb);
						task_planner.AddTraj(&vp2, eBSpline, true);
					}
					break;
					}
					//task_planner.AddTraj(&vp3, eBSpline, true);
					*operation = {};
				}
				task_planner.GenerateJPath(q_cmd);
				//task_planner.GenerateCPath(cpos_cmd);
				rbt.UpdateJntHoldPos(q_cmd);
			}
			else
			{
				if ((strcmp(operation, "toiletlid")==0)||(strcmp(operation, "table")==0)
					||(strcmp(operation, "mirror")==0)||(strcmp(operation, "toilet")==0))
					task_planner._task_completed = false;
				q_cmd = rbt.HoldJntPos();
				std::this_thread::sleep_for(std::chrono::milliseconds(1));
			}
			q_fdb = q_cmd;
			ofile<<q_cmd<<endl;
			//ofile<<cpos_cmd<<endl;
		}
	}
}

void TestRobotDynamics()
{
	UrRobot rbt = CreatUrRobot();
	ForceControl force_controller;
	VectorXd barycenter_params = Map<Matrix<double, 16, 1> >(g_barycenter_params);
	VectorXd fric_params = Map<Matrix<double, 12, 1>>(g_fric_params);
	VectorXd grav_scale = Map<Matrix<double, 6, 1>>(g_grav_scale);
	VectorXd fric_scale = Map<Matrix<double, 6, 1>>(g_fric_scale);
	VectorXd tau_external_base = Map<Matrix<double, 6, 1>>(g_tau_external_base);
	VectorXd tau_external_limit = Map<Matrix<double, 6, 1>>(g_tau_external_limit);
	AllDynamicsParams all_params{ barycenter_params,fric_params,grav_scale,
								fric_scale,tau_external_base,tau_external_limit };
	force_controller.CreateForceControl(&rbt, all_params);
	VectorXd q0, dq, tau_fdb;
	q0.setZero(6);
	dq.setZero(6);
	tau_fdb.setZero(6);
	q0<<-20, 26, 13, -70, -90, 0;
	q0 = q0*D2R;
	dq<<0.2, 0, 0, 0, 0, -0.3;

	string file_name = "../data/mirrortask_jpos1.csv";
	ofstream ofile;
	ofile.open(file_name, ios::out|ios::trunc);
	double t = 0;
	if (ofile.is_open())
		cout<<"ssss"<<endl;
	VectorXd pos_comp;
	pos_comp.setZero(6);
	while (t<5)
	{
		VectorXd q = q0+pos_comp;
		force_controller.RobotDynamicsCalculation(q, dq, tau_fdb);
		VectorXd tau_grav = force_controller.GetGravityTorque();
		VectorXd tau_fric = force_controller.GetFrictionTorque();
		pos_comp = force_controller.GetCompliancePosition();
		ofile<<rbt.FKSolveTool(q).pos<<endl;
		t += 0.005;
	}

	//cout<<tau_grav<<endl;
	//cout<<tau_fric<<endl;
	//cout<<pos_comp<<endl;
}

void TestGenerateFricIdenTraj()
{
	string file_name = "../data/mirrortask_jpos1.csv";
	ofstream ofile;
	ofile.open(file_name, ios::out|ios::trunc);

	Vector6d jpos_standby;
	jpos_standby<<0, -35, 20, 65, -90, 0;
	jpos_standby = jpos_standby*D2R;
	UrRobot rbt = CreatUrRobot();
	TaskTrajPlanner task_planner(&rbt, jpos_standby, g_cycle_time, g_jvmax, g_jamax,g_jjmax, g_cvmax, g_camax,g_cjmax);

	int joint_idx = 1;
	Vector6d proj;
	if (joint_idx==1)
		proj<<1, 0, 0, 0, 0, 0;
	else if (joint_idx==5)
		proj<<0, 0, 0, 0, 1, 0;

	VectorXd det_q, vel_scale;
	det_q.setZero(11);
	vel_scale.setZero(11);
	det_q<<10, 20, 30, 50, 70, 90, 110, 120, 130, 140, 150;
	det_q = det_q*D2R;
	vel_scale<<0.05, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0;
	task_planner.Reset(jpos_standby);
	for (int idx = 0; idx<(int)(vel_scale.size()); idx++)
	{
		Vector6d detq_pos = proj*det_q(idx);
		Vector6d detq_neg;
		if (idx==(int)(vel_scale.size())-1)
			detq_neg = -proj*det_q(idx);
		else
			detq_neg = -proj*det_q(idx+1);

		Vector6d q1 = jpos_standby+detq_pos;
		Vector6d q2 = jpos_standby+detq_neg;
		MatrixXd jpoint;
		jpoint.setZero(6, 2);
		jpoint<<q1, q2;
		task_planner.AddTraj(&jpoint, eJointSpace, true, vel_scale(idx), 1);
	}
	while (!task_planner._task_completed)
	{
		Vector6d q_cmd;
		task_planner.GenerateJPath(q_cmd);
		ofile<<q_cmd<<endl;
	}

}

void ReadDynIdenTrajParams(vector<vector<double>>& params,string filename)
{
	params.clear();
	ifstream fp(filename);
	string line;
	while (getline(fp, line))
	{
		string num;
		istringstream readnum(line);
		vector<double>* jnt_traj_params = new vector<double>;
		while (getline(readnum, num, ','))
		{
			double param = atof(num.c_str());
			jnt_traj_params->push_back(param);
		}
		params.push_back(*jnt_traj_params);
		delete jnt_traj_params;
	}
}

void TestGenerateDynIdenTraj()
{
	Vector6d jpos_standby;
	jpos_standby<<0, -35, 20, 65, -90, 0;
	jpos_standby = jpos_standby*D2R;
	Vector6d q0;
	q0.setZero();
	UrRobot rbt = CreatUrRobot();
	TaskTrajPlanner task_planner(&rbt, q0, g_cycle_time, g_jvmax, g_jamax, g_jjmax, g_cvmax, g_camax, g_cjmax);

	cout<<"start to simulation"<<endl;
	string file_name1,file_name2;
#ifdef _WIN32
	file_name1 = "../data/traj_0.5hz.csv";
	file_name2 = "../data/traj_1hz.csv";
#elif __linux__
	file_name1 = "../../data/traj_0.5hz.csv";
	file_name2 = "../../data/traj_1hz.csv";
#endif
	ReadDynIdenTrajParams(g_dyn_iden_params_half1hz,file_name1);
	ReadDynIdenTrajParams(g_dyn_iden_params_1hz, file_name2);
	DynIdenTrajPlanner dynidentraj_planner;
	dynidentraj_planner.Init(g_dyn_iden_params_half1hz, g_dyn_iden_params_1hz, g_cycle_time);
	dynidentraj_planner.Reset();
	dynidentraj_planner.AddTraj(10, 7);
	task_planner.AddTraj(&dynidentraj_planner,eDynIden);

	//string file_name = "D:/00Work/01projects/RobotDynIden/slave/optimal_trajectory/traj_cpp_1hz.csv";
	string file_name = "D:/00Work/01projects/XProject/src/data/mirrortask_jpos1.csv";
	ofstream ofile;
	ofile.open(file_name, ios::out|ios::trunc);
	Vector6d q_cmd = q0;
	while(!task_planner._task_completed)
	{
		task_planner.GenerateJPath(q_cmd);
		ofile<<q_cmd<<endl;
	}
	ofile.close();
	cout<<"complete simulation"<<endl;
}

void TestJointDataProcessor()
{
	Vector6d q;
	q<<1, 2, 3, 4, 5, 6;
	VectorXd qq(6);
	qq = q;
	cout<<qq[3]<<endl;
	RobotParameter::readEncoderTurnsFileJson("../data/encoder_turns.json");
	JointDataProcessor<6> jd_processor;
	ServoParams<6> servo_params;
	memcpy(servo_params.offset, g_encoder_offset, sizeof(servo_params.offset));
	memcpy(servo_params.resolution, g_encoder_resolution, sizeof(servo_params.resolution));
	memcpy(servo_params.dir, g_encoder_dir, sizeof(servo_params.dir));
	memcpy(servo_params.rated_torque, g_rated_torque, sizeof(servo_params.rated_torque));
	memcpy(servo_params.rated_current, g_rated_current, sizeof(servo_params.rated_current));
	memcpy(servo_params.motor_torque_coeff, g_torque_coeff, sizeof(servo_params.motor_torque_coeff));
	memcpy(servo_params.gear_ratio, g_gear_ratio, sizeof(servo_params.gear_ratio));
	memcpy(servo_params.gear_efficiency, g_gear_efficiency, sizeof(servo_params.gear_efficiency));
	memcpy(servo_params.multi_turns, g_encoder_turns, sizeof(servo_params.multi_turns));

	int32_t encoder_pos[6] = { 23554,234266,435364,765343,3547,325326 };
	int32_t encoder_vel[6] = { 0 };
	int16_t motor_tau[6] = { 0 };
	jd_processor.Initialize(servo_params, false, encoder_pos);
	jd_processor.InputFromServo(encoder_pos, encoder_vel, motor_tau);

	static int pre_turns_save[6];
	int turns_save[6];
	jd_processor.GetUpdatedTurns(turns_save);
	for (int idx = 0; idx<6; idx++)
	{
		if (turns_save[idx]!=pre_turns_save[idx])
		{
			RobotParameter::writeEncoderTurnsFileJson("../data/encoder_turns.json", turns_save);
			break;
		}
	}
	memcpy(pre_turns_save, turns_save, sizeof(turns_save));

}

void TestMirrorClean()
{
	UrRobot rbt = CreatUrRobot();
	Vector6d q0;
	q0<<0, -35, 50, -100, -90, 0;
	q0 = q0*D2R;
	TaskTrajPlanner task_planner(&rbt, q0, g_cycle_time, g_jvmax, g_jamax, g_jjmax, g_cvmax, g_camax, g_cjmax);

	MirrorCleanPlanner mirror_planner;
	MirrorType mirror_type = MirrorType::eOctagon;
	MatrixXd via_posrpy_up, via_posrpy_middle, via_posrpy_down;
	if (mirror_type==MirrorType::eRectangle)
	{
		Vector3d p1(0.8, -0.2, 0.45);
		Vector3d p2(0.8, 0.4, 0.45);
		Vector3d p3(0.8, 0.4, 0.81);
		Vector3d p4(0.8, -0.2, 0.81);
		MatrixXd vision_pos(3, 4);
		vision_pos<<p1, p2, p3, p4;
		mirror_planner.SetCleanParams(vision_pos, mirror_type);
		mirror_planner.PlanCleanPath(via_posrpy_up, via_posrpy_middle, via_posrpy_down);
		task_planner.AddTraj(&via_posrpy_middle, RobotTools::TrajType::eCartesianSpace,false);
	}
	else if (mirror_type==MirrorType::eCircle)
	{
		double r = 0.4;
		Vector3d origin(0.8, 0, 0.8);
		Matrix3d rot_circle;
		rot_circle<<0, 0, -1, -1, 0, 0, 0, 1, 0;
		Vector3d p1(origin(0), origin(1)-r, origin(2));
		Vector3d p2(origin(0), origin(1), origin(2)-r);
		Vector3d p3(origin(0), origin(1)+r, origin(2));
		Vector3d p4(origin(0), origin(1), origin(2)+r);
		MatrixXd vision_pos(3, 4);
		vision_pos<<p1, p2, p3, p4;
		mirror_planner.SetCleanParams(vision_pos, MirrorType::eCircle);
		mirror_planner.PlanCleanPath(via_posrpy_up, via_posrpy_middle, via_posrpy_down);
		// plan for up zone
		MatrixXd posrpy_tmp = via_posrpy_up.leftCols(1);
		// task_planner.AddTraj(&posrpy_tmp, RobotTools::TrajType::eCartesianSpace, false);
		// posrpy_tmp = via_posrpy_up.middleCols(1, 3);
		// task_planner.AddTraj(&posrpy_tmp, RobotTools::TrajType::eCartesianArc, false);
		// posrpy_tmp = via_posrpy_up.rightCols(1);
		// task_planner.AddTraj(&posrpy_tmp, RobotTools::TrajType::eCartesianSpace, false);
		// plan for middle zone
		posrpy_tmp = via_posrpy_middle.leftCols(1);
		task_planner.AddTraj(&posrpy_tmp, RobotTools::TrajType::eCartesianSpace, false);
		posrpy_tmp = via_posrpy_middle.middleCols(1, 3);
		task_planner.AddTraj(&posrpy_tmp, RobotTools::TrajType::eCartesianArc, false);
		posrpy_tmp = via_posrpy_middle.middleCols(4, via_posrpy_middle.cols()-4-4);
		task_planner.AddTraj(&posrpy_tmp, RobotTools::TrajType::eCartesianSpace, false);
		posrpy_tmp = via_posrpy_middle.middleCols(via_posrpy_middle.cols()-4, 3);
		task_planner.AddTraj(&posrpy_tmp, RobotTools::TrajType::eCartesianArc, false);
		posrpy_tmp = via_posrpy_middle.rightCols(1);
		task_planner.AddTraj(&posrpy_tmp, RobotTools::TrajType::eCartesianSpace, false);
		// plan for down zone
		posrpy_tmp = via_posrpy_down.leftCols(1);
		task_planner.AddTraj(&posrpy_tmp, RobotTools::TrajType::eCartesianSpace, false);
		posrpy_tmp = via_posrpy_down.middleCols(1, 3);
		task_planner.AddTraj(&posrpy_tmp, RobotTools::TrajType::eCartesianArc, false);
		posrpy_tmp = via_posrpy_down.rightCols(1);
		task_planner.AddTraj(&posrpy_tmp, RobotTools::TrajType::eCartesianSpace, false);
	}
	else if (mirror_type==MirrorType::eEllipse)
	{
		double a = 0.5;
		double b = 0.3;
		Vector3d origin(0.8, 0, 0.8);
		Matrix3d rot_ellipse;
		rot_ellipse<<0, 0, -1, -1, 0, 0, 0, 1, 0;
		Vector3d p1(origin(0), origin(1)-b, origin(2));
		Vector3d p2(origin(0), origin(1), origin(2)-a);
		Vector3d p3(origin(0), origin(1)+b, origin(2));
		Vector3d p4(origin(0), origin(1), origin(2)+a);
		MatrixXd vision_pos(3, 4);
		vision_pos<<p1, p2, p3, p4;
		mirror_planner.SetCleanParams(vision_pos, MirrorType::eEllipse);
		mirror_planner.PlanCleanPath(via_posrpy_up, via_posrpy_middle, via_posrpy_down);
		// plan for up zone
		MatrixXd posrpy_tmp = via_posrpy_up.leftCols(1);
		task_planner.AddTraj(&posrpy_tmp, RobotTools::TrajType::eCartesianSpace, false);
		posrpy_tmp = via_posrpy_up.middleCols(1, 3);
		task_planner.AddTraj(&posrpy_tmp, RobotTools::TrajType::eCartesianArc, false);
		posrpy_tmp = via_posrpy_up.rightCols(1);
		task_planner.AddTraj(&posrpy_tmp, RobotTools::TrajType::eCartesianSpace, false);
		// plan for middle zone
		task_planner.AddTraj(&via_posrpy_middle, RobotTools::TrajType::eCartesianSpace, false);
		// plan for down zone
		posrpy_tmp = via_posrpy_down.leftCols(1);
		task_planner.AddTraj(&posrpy_tmp, RobotTools::TrajType::eCartesianSpace, false);
		posrpy_tmp = via_posrpy_down.middleCols(1, 3);
		task_planner.AddTraj(&posrpy_tmp, RobotTools::TrajType::eCartesianArc, false);
		posrpy_tmp = via_posrpy_down.rightCols(1);
		task_planner.AddTraj(&posrpy_tmp, RobotTools::TrajType::eCartesianSpace, false);
	}
	else if (mirror_type==MirrorType::eRunway)
	{
		Vector3d p1(0.8, -0.2, 0.45);
		Vector3d p2(0.8, 0.4, 0.45);
		Vector3d p3(0.8, 0.4, 0.81);
		Vector3d p4(0.8, -0.2, 0.81);
		MatrixXd vision_pos(3, 4);
		vision_pos<<p1, p2, p3, p4;
		mirror_planner.SetCleanParams(vision_pos, MirrorType::eRunway);
		mirror_planner.PlanCleanPath(via_posrpy_up, via_posrpy_middle, via_posrpy_down);
		// plan for up zone
		MatrixXd posrpy_tmp = via_posrpy_up.leftCols(1);
		task_planner.AddTraj(&posrpy_tmp, RobotTools::TrajType::eCartesianSpace, false);
		posrpy_tmp = via_posrpy_up.middleCols(1, 3);
		task_planner.AddTraj(&posrpy_tmp, RobotTools::TrajType::eCartesianArc, false);
		posrpy_tmp = via_posrpy_up.rightCols(1);
		task_planner.AddTraj(&posrpy_tmp, RobotTools::TrajType::eCartesianSpace, false);
		// plan for middle zone
		task_planner.AddTraj(&via_posrpy_middle, RobotTools::TrajType::eCartesianSpace, false);
		// plan for down zone
		posrpy_tmp = via_posrpy_down.leftCols(1);
		task_planner.AddTraj(&posrpy_tmp, RobotTools::TrajType::eCartesianSpace, false);
		posrpy_tmp = via_posrpy_down.middleCols(1, 3);
		task_planner.AddTraj(&posrpy_tmp, RobotTools::TrajType::eCartesianArc, false);
		posrpy_tmp = via_posrpy_down.rightCols(1);
		task_planner.AddTraj(&posrpy_tmp, RobotTools::TrajType::eCartesianSpace, false);
	}
	else if (mirror_type==MirrorType::eOctagon)
	{
		Vector3d p1(0.8, -0.2, 0.45);
		Vector3d p2(0.8, 0.1, 0.3);
		Vector3d p3(0.8, 0.4, 0.45);
		Vector3d p4(0.8, 0.4, 0.81);
		Vector3d p5(0.8, -0.2, 0.81);
		MatrixXd vision_pos(3, 5);
		vision_pos<<p1, p2, p3, p4, p5;
		mirror_planner.SetCleanParams(vision_pos, MirrorType::eOctagon);
		mirror_planner.PlanCleanPath(via_posrpy_up, via_posrpy_middle, via_posrpy_down);
		task_planner.AddTraj(&via_posrpy_middle, RobotTools::TrajType::eCartesianSpace, false);
		task_planner.AddTraj(&via_posrpy_down, RobotTools::TrajType::eCartesianSpace, false);
	}

	std::cout<<"up zone:"<<std::endl<<via_posrpy_up<<std::endl;
	std::cout<<"left middle zone:"<<std::endl<<via_posrpy_middle.leftCols(10)<<std::endl;
	std::cout<<"right middle zone:"<<std::endl<<via_posrpy_middle.rightCols(10)<<std::endl;
	std::cout<<"down zone:"<<std::endl<<via_posrpy_down<<std::endl;
}

void TestPlanner()
{
	UrRobot rbt = CreatUrRobot();
	Vector6d q_fdb;
	q_fdb << 0, 0, 0, 0, -90, 0;
	q_fdb = q_fdb*D2R;
	TaskTrajPlanner task_planner(&rbt, q_fdb, g_cycle_time, g_jvmax, g_jamax, g_jjmax, g_cvmax, g_camax, g_cjmax);
	Vector6d pos_rpy1, pos_rpy2, pos_rpy3, pos_rpy4;
	pos_rpy1<<0.8, 0, 0.23, -pi, -pi/6, 0;
	pos_rpy2<<0.3, 0.2, 0.23, -pi, -pi/6, 0;
	pos_rpy3<<0.8, 0, 0.1, -pi, -pi/6, 0;
	pos_rpy4<<0.2, 0.6, 0.23, -pi, -pi/6, 0;
	MatrixXd vpr(6, 4);
	vpr<<pos_rpy1, pos_rpy2, pos_rpy3, pos_rpy4;
	task_planner.AddTraj(&vpr, eCartesianSpace, true);

	string file_name = "F:/0_project/rkdpl/mirrortask_jpos1.csv";
	ofstream ofile;
	ofile.open(file_name, ios::out|ios::trunc);
	if (!ofile.is_open())
	{
		cout<<"failed to open the file"<<endl;
	}
	else
	{
		while (!task_planner._task_completed)
		{
			Vector6d pos_cmd;
			task_planner.GenerateCPath(pos_cmd);
			ofile<<pos_cmd<<endl;
			//std::this_thread::sleep_for(std::chrono::milliseconds(1));
		}
	}

}

int main()
{
	map<ID_test, PtrTest> test_map;
	TestInfo config[] =
	{		
		{ other,			Testother },
		{ jtrajlspb,		TestLspb},
		{ bspline,			TestBSpline },
		{ dynamics,			TestRobotDynamics},
		{ fricidentraj,		TestGenerateFricIdenTraj },
		{ dynidentraj,		TestGenerateDynIdenTraj },
		{ jdataprocessor,	TestJointDataProcessor},
		{ cleanmirror,		TestMirrorClean},
		{ planner,			TestPlanner}
	};
	int test_count = sizeof(config)/sizeof(*config);
	while (test_count--)
		test_map[config[test_count].testID] = config[test_count].testfunc;

#if defined(lzx)
	test_map[simulation] = SimulationLZX;
#elif defined(wyc)
	test_map[simulation] = Simulation;
#endif

	ID_test test_mode = jtrajlspb;
	test_map[test_mode]();

	return 0;
}

