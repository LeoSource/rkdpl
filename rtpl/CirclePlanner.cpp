/**
* @file		CirclePlanner.h
* @brief	Cartesian Circle trajectory plan
* @version	1.1.0
* @author	wyc
* @date		2021/11/30
**/

#include "CirclePlanner.h"

CirclePlanner::CirclePlanner(Vector6d* pos_rpy1, Vector3d* pos2, Vector3d* pos3,
	bool traj_opt, double* vmax, double* amax, double* jmax, double cycle_time, double angle)
{
	Vector6d pos1 = *pos_rpy1;
	_pos_initial = pos1.head(3);
	_rpy_initial = pos1.tail(3);
	double vel_cons[] = { 0,0,0,0 };
	_cycle_time = cycle_time;
	_angle = angle;
	_variable_attitude = traj_opt;
	InitPosPlanner(pos1.head(3), *pos2, *pos3, vmax[0],
		amax[0], jmax[0], -1, vel_cons);

}

void CirclePlanner::Reset(Vector6d pos, bool variable_attitude)
{
	_t = 0;
	_variable_attitude = variable_attitude;
}

void CirclePlanner::GeneratePath(Vector6d& pos)
{
	RobotTools::CAVP cavp;
	RobotTools::CLineAVP line_avp, rpy_avp;
	line_avp = GeneratePosMotion(_t);
	if (_variable_attitude == true)
		rpy_avp = GenerateRotMotion(_t);
	else
	{
		rpy_avp.pos = _rpy_initial;
		rpy_avp.vel = Vector3d::Zero();
		rpy_avp.acc = Vector3d::Zero();
	}
	_t += _cycle_time;
	cavp = toSpatial(&line_avp, &rpy_avp);
	pos = cavp.pos;
	double tf = _pos_uplanner.GetFinalTime();
	MathTools::LimitMax(tf, _t);
	if (fabs(_t - tf)<EPS3)
		_plan_completed = true;
}

void CirclePlanner::GenerateMotion(Vector6d& pos, Vector6d& vel, Vector6d& acc)
{
	RobotTools::CAVP cavp;
	RobotTools::CLineAVP line_avp, rpy_avp;
	line_avp = GeneratePosMotion(_t);
	if (_variable_attitude == true)
		rpy_avp = GenerateRotMotion(_t);
	else
	{
		rpy_avp.pos = _rpy_initial;
		rpy_avp.vel = Vector3d::Zero();
		rpy_avp.acc = Vector3d::Zero();
	}
	_t += _cycle_time;
	cavp = toSpatial(&line_avp, &rpy_avp);
	pos = cavp.pos;
	vel = cavp.vel;
	acc = cavp.acc;
	double tf = _pos_uplanner.GetFinalTime();
	MathTools::LimitMax(tf, _t);
	if (fabs(_t - tf)<EPS3)
		_plan_completed = true;
}

void CirclePlanner::InitPosPlanner(Vector3d pos1, Vector3d pos2, Vector3d pos3, double line_vmax,
	double line_amax, double line_jmax, double tf, double* vel_cons)
{
	InitCircleInfo(pos1, pos2, pos3);
	if (tf<0)
		_pos_uplanner.InitPlanner(Vector2d(0, _pos_len), line_vmax,
			line_amax, line_jmax, Vector2d(0, -1), Vector2d(vel_cons[0], vel_cons[1]));
	else
		_pos_uplanner.InitPlanner(Vector2d(0, _pos_len), line_vmax,
			line_amax, line_jmax, Vector2d(0, tf), Vector2d(vel_cons[0], vel_cons[1]));

	_tf = _pos_uplanner.GetFinalTime();
	_rpy_start = GenerateRotMotion(0).pos;
	_rpy_end = GenerateRotMotion(_tf).pos;
}

void CirclePlanner::InitCircleInfo(Vector3d pos1, Vector3d pos2, Vector3d pos3)
{
	Vector4d params1 = PointsCoplane(pos1, pos2, pos3);
	Vector4d params2 = RadiusEqual(pos1, pos2);
	Vector4d params3 = RadiusEqual(pos1, pos3);
	double a1 = params1(0), b1 = params1(1), c1 = params1(2), d1 = params1(3);
	double a2 = params2(0), b2 = params2(1), c2 = params2(2), d2 = params2(3);
	double a3 = params3(0), b3 = params3(1), c3 = params3(2), d3 = params3(3);
	_center(0) = -(b1*c2*d3 - b1*c3*d2 - b2*c1*d3 + b2*c3*d1 + b3*c1*d2 - b3*c2*d1) /
		(a1*b2*c3 - a1*b3*c2 - a2*b1*c3 + a2*b3*c1 + a3*b1*c2 - a3*b2*c1);
	_center(1) = (a1*c2*d3 - a1*c3*d2 - a2*c1*d3 + a2*c3*d1 + a3*c1*d2 - a3*c2*d1) /
		(a1*b2*c3 - a1*b3*c2 - a2*b1*c3 + a2*b3*c1 + a3*b1*c2 - a3*b2*c1);
	_center(2) = -(a1*b2*d3 - a1*b3*d2 - a2*b1*d3 + a2*b3*d1 + a3*b1*d2 - a3*b2*d1) /
		(a1*b2*c3 - a1*b3*c2 - a2*b1*c3 + a2*b3*c1 + a3*b1*c2 - a3*b2*c1);
	Vector3d xr = pos1 - _center;
	_radius = xr.norm();
	_theta = 2 * pi;//circle
	Vector3d zr(a1, b1, c1);
	Vector3d yr = zr.cross(xr);
	xr.normalize(); yr.normalize(); zr.normalize();
	_rot << xr, yr, zr;
	_pos_len = _radius*_theta;
}

Vector4d CirclePlanner::PointsCoplane(Vector3d pos1, Vector3d pos2, Vector3d pos3)
{
	double x1 = pos1(0), y1 = pos1(1), z1 = pos1(2);
	double x2 = pos2(0), y2 = pos2(1), z2 = pos2(2);
	double x3 = pos3(0), y3 = pos3(1), z3 = pos3(2);
	double a = y1*z2 - y2*z1 - y1*z3 + y3*z1 + y2*z3 - y3*z2;
	double b = -(x1*z2 - x2*z1 - x1*z3 + x3*z1 + x2*z3 - x3*z2);
	double c = x1*y2 - x2*y1 - x1*y3 + x3*y1 + x2*y3 - x3*y2;
	double d = -(x1*y2*z3 - x1*y3*z2 - x2*y1*z3 + x2*y3*z1 + x3*y1*z2 - x3*y2*z1);

	return Vector4d(a, b, c, d);
}

Vector4d CirclePlanner::RadiusEqual(Vector3d pos1, Vector3d pos2)
{
	Vector4d res;
	res(0) = 2 * (pos2(0) - pos1(0));
	res(1) = 2 * (pos2(1) - pos1(1));
	res(2) = 2 * (pos2(2) - pos1(2));
	res(3) = pow(pos1(0), 2) + pow(pos1(1), 2) + pow(pos1(2), 2)
		- pow(pos2(0), 2) - pow(pos2(1), 2) - pow(pos2(2), 2);

	return res;
}


RobotTools::CLineAVP CirclePlanner::GeneratePosMotion(double t)
{
	RobotTools::CLineAVP line_avp;
	RobotTools::JAVP uavp;
	if (t>_tf)
	{
		uavp.pos = _pos_len;
		uavp.vel = 0;
		uavp.acc = 0;
	}
	else
	{
		uavp = _pos_uplanner.GenerateMotion(t);
	}
	double r = _radius;
	double th = uavp.pos / r;
	Vector3d parc = Vector3d::Zero();
	parc(0) = r*cos(th);
	parc(1) = r*sin(th);
	line_avp.pos = _center + _rot*parc;
	Vector3d vc = Vector3d::Zero();
	Vector3d ac = Vector3d::Zero();
	vc(0) = -uavp.vel*sin(th);
	vc(1) = uavp.vel*cos(th);
	line_avp.vel = _rot*vc;
	ac(0) = -pow(uavp.vel, 2)*cos(th) / r - uavp.acc*sin(th);
	ac(1) = -pow(uavp.vel, 2)*sin(th) / r + uavp.acc*cos(th);
	line_avp.acc = _rot*ac;

	return line_avp;
}

RobotTools::CLineAVP CirclePlanner::GenerateRotMotion(double t)
{
	RobotTools::CLineAVP rpy_avp;
	RobotTools::JAVP uavp;
	if (_variable_attitude == true)
	{
		if (t > _tf)
		{
			uavp.pos = _pos_len;
			uavp.vel = 0;
			uavp.acc = 0;
		}
		else
		{
			uavp = _pos_uplanner.GenerateMotion(t);
		}
		double th = uavp.pos / _radius;
		Matrix3d temp_r = RobotTools::RotZ(th);
		Matrix3d r = RobotTools::Rodrigues(temp_r.col(1), _angle)*_rot;
		rpy_avp.pos = RobotTools::Tr2FixedZYX(r);
	}
	else
	{
		rpy_avp.pos = _rpy_initial;
	}
	rpy_avp.vel.setZero();
	rpy_avp.acc.setZero();
	return rpy_avp;
}
