#pragma once

#include "RobotMath.h"

using namespace std;
using namespace Eigen;

class SVelProfilePlanner
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
public:
	int _dir;
	bool _maxvel_reached;
	bool _maxacc_reached;
	bool _maxdec_reached;
private:
	Vector2d _pos;
	double _v0;
	double _vf;

	double _tf;
	double _t_acc;
	double _t_vel;
	double _t_dec;
	double _t_j1;
	double _t_j2;
	double _vlim;
	double _lim_acc;
	double _lim_dec;

	double _max_vel;
	double _max_acc;
	double _max_jerk;

public:
	SVelProfilePlanner() {}

	SVelProfilePlanner(Vector2d pos, double max_vel, double max_acc, double max_jerk);

	SVelProfilePlanner(Vector2d pos, double max_vel, double max_acc, double max_jerk, double tf);

	SVelProfilePlanner(Vector2d pos, double max_vel, double max_acc, double max_jerk, Vector2d duration);

	SVelProfilePlanner(Vector2d pos, double max_vel, double max_acc, double max_jerk, Vector2d duration, Vector2d vel_con);

	void InitPlanner(Vector2d pos, double max_vel, double max_acc, double max_jerk);

	void InitPlanner(Vector2d pos, double max_vel, double max_acc, double max_jerk, double tf);

	void InitPlanner(Vector2d pos, double max_vel, double max_acc, double max_jerk, Vector2d duration);

	void InitPlanner(Vector2d pos, double max_vel, double max_acc, double max_jerk, Vector2d duration, Vector2d vel_con);

	~SVelProfilePlanner() {}

	RobotTools::JAVP GenerateMotion(double t);

	double GetFinalTime();

	double GetDuratoin();

	double GetDecTime();

	double GetAccTime();

private:
	void TransformPVAJ(Vector2d pos, Vector2d vel, double max_vel, double max_acc, double max_jerk);

	void CalcSVelProfilePara(double h);

	void CalcSVelWithNullVel(double h);

	void UpdateSVelProfilePara(double duration);
};

