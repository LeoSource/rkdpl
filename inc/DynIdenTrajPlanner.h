/**
* @file		DynIdenTrajPlanner.h
* @brief	trajectory plan for robot dynamics identification
* @version	1.1.0
* @author	zxliao
* @email	zhixiangleo@163.com
* @date		2022/01/14
**/
#pragma once
#include <vector>
#include "RobotMath.h"

using namespace std;

class DynIdenTrajPlanner
{
public:
	bool _plan_completed;
	vector<Vector6d> _q0;
private:
	vector<MatrixXd*> _traj_params_half1hz;
	vector<MatrixXd*> _traj_params_1hz;
	double _w0;
	int _order;
	VectorXd _q_b[2], _qd_b[2], _qdd_b[2];
	double _t;
	double _cycle_time;
private:
	int _traj_num;
	vector<int> _traj_id;
	int _traj_idx;
	VectorXd _q_base, _qd_base, _qdd_base;
	vector<MatrixXd*> _traj_order_params;

public:
	DynIdenTrajPlanner() = default;

	DynIdenTrajPlanner(const DynIdenTrajPlanner& planner) = delete;

	DynIdenTrajPlanner& operator=(const DynIdenTrajPlanner& planner) = delete;

	void Init(const vector<vector<double>>& params_half1hz,
				const vector<vector<double>>& params_1hz,double cycle_time);

	void AddTraj(const int order,vector<int> traj_id);

	void AddTraj(const int order, const int traj_id);

	void GenerateTraj(Vector6d& jpos);

	void Reset();

	~DynIdenTrajPlanner();

private:
	void InitTrajParams(vector<MatrixXd*>& traj_params, 
						const vector<vector<double>>& params_hz,int order);

	RobotTools::JAVP GenerateJntMotion(double t, int jnt_idx, int traj_idx);

	void GenerateMotion(Vector6d&jpos, double t, int traj_idx);

	void DeleteObject(vector<MatrixXd*>& params);

	void DeleteAllObject();

};



