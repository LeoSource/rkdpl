/**
* @file		TaskTrajPlanner.h
* @brief	Trajectory plan for all tasks
* @version	2.0.1
* @author	zxliao
* @email	zhixiangleo@163.com
* @date		2021/10/25
**/
#pragma once

#include "UrRobot.h"
#include "DynIdenTrajPlanner.h"
#include <map>

class BaseTrajPlanner;

class TaskTrajPlanner
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
public:
	bool _task_completed;
private:
	int _ntraj;
	int _traj_idx;
	vector<TrajType> _traj_type;
	vector<BaseTrajPlanner*> _segplanner;
	UrRobot* _robot;
	DynIdenTrajPlanner* _dyniden_planner;
	Vector6d _pre_q;
	Pose _pre_trajpose;
	Vector6d _pre_trajq;
	double _cycle_time;
	double *_jvmax, *_jamax, *_jjmax;
	double *_cvmax, *_camax, *_cjmax;
	double _angle,_angle_start,_angle_end;
	using PtrAddMethod = void (TaskTrajPlanner::*)(MatrixXd*,bool,double*,double*);
	std::map<RobotTools::TrajType, PtrAddMethod> _addmethod_map;
private:
	Vector3d _clean_toilet_center;
	double _clean_toilet_slant;

public:
	TaskTrajPlanner() = default;
	TaskTrajPlanner(const TaskTrajPlanner& planner) = delete;
	TaskTrajPlanner& operator=(const TaskTrajPlanner& planner) = delete;

	TaskTrajPlanner(UrRobot* robot_model, Vector6d q0, double cycle_time,
		double* base_jvmax, double* base_jamax, double* base_jjmax, double* base_cvmax, double* base_camax, double* base_cjmax);
	void AddTraj(MatrixXd* via_pos, TrajType traj_type, bool traj_opt,
				double vel_ratio=1, double acc_ratio=1,double angle_start = 0,double angle_end = 0);

	void AddTraj(MatrixXd* via_pos, TrajType traj_type, bool traj_opt,
				double* vmax, double* amax);

	void AddTraj(DynIdenTrajPlanner* dyniden_planner, TrajType traj_type);

	void GenerateJPath(Vector6d& jpos);

	void GenerateCPath(Vector6d& cpos);

	void GenerateBothPath(Vector6d& jpos, Vector6d& cpos);

	void SetPlanningScene(const MatrixXd& vertices, double slant);

	void Reset(Vector6d q0);

	~TaskTrajPlanner() = default;
private:
	void AddJointTraj(MatrixXd* via_pos, bool traj_opt, double* vmax, double* amax);

	void AddCartesianTraj(MatrixXd* via_pos, bool traj_opt, double* vmax, double* amax);

	void AddArcTraj(MatrixXd* via_pos, bool traj_opt, double* vmax, double* amax);

	void AddBSplineTraj(MatrixXd* via_pos, bool traj_opt, double* vmax, double* amax);

	void AddCircleTraj(MatrixXd* via_pos, bool traj_opt, double* vmax, double* amax);

	void AddEllipseTraj(MatrixXd* via_pos, bool traj_opt, double* vmax, double* amax);

	void InitAddMap();

	double CalcBSplineTime(MatrixXd* via_pos, double* vmax);

	Vector3d CalcCleanToiletRPY(Vector3d pos);

	void CheckCartWorkspace(Vector3d cpos);
};

