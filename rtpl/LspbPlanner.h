/**
* @file		LspbPlanner.h
* @brief	Linear segment with parabolic blend(trapezoid velocity profile)
* @version	1.0.0
* @author	zxliao
* @email	zhixiangleo@163.com
* @date		2021/6/1
**/
#pragma once

#include "RobotMath.h"

using namespace std;
using namespace Eigen;

class LspbPlanner
{
public:
	double _vmax;
	double _amax;
	bool _maxvel_reached;
	int _dir;
private:
	int _np;
	double _t0;
	double _tf;
	double _v0;
	double _vf;
	double _q0;
	double _qf;
	double _ta;
	double _td;

public:
	LspbPlanner() {}

	LspbPlanner(Vector2d pos, double max_vel, double max_acc);

	LspbPlanner(Vector2d pos, double max_vel, double max_acc, double tf);

	LspbPlanner(Vector2d pos, double max_vel, double max_acc, Vector2d duration);

	LspbPlanner(Vector2d pos, double max_vel, double max_acc, Vector2d duration, Vector2d vel_con);

	void InitPlanner(Vector2d pos, double max_vel, double max_acc);

	void InitPlanner(Vector2d pos, double max_vel, double max_acc, double tf);

	void InitPlanner(Vector2d pos, double max_vel, double max_acc, Vector2d duration);

    void InitPlanner(Vector2d pos, double max_vel, double max_acc, Vector2d duration, Vector2d vel_con);

	~LspbPlanner() {}

	/**
	* @brief	generate motion data
	* @author	zxliao
	* @date		2021/6/1
	* @param	t		time
	* @return	avp		position, velocity, acceleration for single axis
	**/
	RobotTools::JAVP GenerateMotion(double t);

	/**
	* @brief	get final time
	* @author	zxliao
	* @date		2021/6/1
	* @return	tf		final time of planner
	**/
	double GetFinalTime();

	/**
	* @brief	get duration
	* @author	zxliao
	* @date		2021/6/1
	* @return	tf-t0	duration of planner
	**/
	double GetDuratoin();

	double GetDecTime();

	double GetAccTime();

private:
	/**
	* @brief	transform to planner's direction
	* @author	zxliao
	* @date		2021/6/1
	* @param	pos			initial and final position
	* @param	vel			initial and final velocity
	* @param	max_vel		constraint of velocity
	* @param	max_acc		constraint of acceleration
	**/
	void TransformPVA(Vector2d pos, Vector2d vel, double max_vel, double max_acc);

	/**
	* @brief	set planner parameters without time constraint
	* @author	zxliao
	* @date		2021/6/1
	* @param	h		position distance
	**/
	void SetNoTimeLimit(double h);

	/**
	* @brief	set planner parameters with time constraint
	* @author	zxliao
	* @date		2021/6/1
	* @param	h			position distance
	* @param	duration	initial and final time
	**/
	void SetTimeLimit(double h, Vector2d duration);

	/**
	* @brief	set planner parameters with velocity constraint
	* @author	zxliao
	* @date		2021/6/1
	* @param	h	position distance
	* @param	tf	final time
	**/
    void SetVelConstraint(double h, double tf);
};

