/**
* @file		EllipsePlanner.h
* @brief	Cartesian Ellipse trajectory plan
* @version	1.1.0
* @author	wyc
* @date		2021/1/6
**/

#include "EllipsePlanner.h"

EllipsePlanner::EllipsePlanner(Vector6d* pos_rpy1, Vector3d* pos2, Vector6d* pos_rpy3,
	double* vmax, double* amax, double* jmax, double cycle_time, double angle_start, double angle_end)
{
	Vector6d pos1 = *pos_rpy1;
	Vector6d pos3 = *pos_rpy3;
	_rpy_initial = pos1.tail(3);
	_pos_start = pos1.head(3);
	_pos_end = pos1.head(3);
	_pos_initial = pos1.head(3);
	_angle_start = angle_start;
	_angle_end = angle_end;
	_cycle_time = cycle_time;
	CalcTrajOption(pos1, pos3);
	double vel_cons[] = { 0,0,0,0 };
	if (_option == RobotTools::eBoth)
	{
		InitPosPlanner(pos1.head(3), *pos2, pos3.head(3),
			vmax[0], amax[0],jmax[0], -1, vel_cons);
		InitRotPlanner(pos1.tail(3), pos3.tail(3), vmax[1],
			amax[1],jmax[1], -1, &(vel_cons[2]));
		_tf_pos = _pos_uplanner.GetFinalTime();
		_tf_rot = _rot_uplanner.GetFinalTime();
		Vector2d tf(_tf_pos, _tf_rot);
		_tf = tf.maxCoeff();
		if (_tf_pos>_tf_rot)
			InitRotPlanner(pos1.tail(3), pos3.tail(3), vmax[1],
				amax[1],jmax[1], _tf, &(vel_cons[2]));
		else
			InitPosPlanner(pos1.head(3), *pos2, pos3.head(3),
				vmax[0], amax[0], jmax[0], _tf, vel_cons);
		_pos_start = GeneratePosMotion(0).pos;
		_pos_end = GeneratePosMotion(_tf).pos;
	}
	else if (_option == RobotTools::ePos)
	{
		InitPosPlanner(pos1.head(3), *pos2, pos3.head(3),
			vmax[0], amax[0], jmax[0], -1, vel_cons);
		_tf_pos = _pos_uplanner.GetFinalTime();
		_tf = _tf_pos;
		_tf_rot = 0;
		_pos_start = GeneratePosMotion(0).pos;
		_pos_end = GeneratePosMotion(_tf).pos;
	}
	else if (_option == RobotTools::eRot)
	{
		InitRotPlanner(pos1.tail(3), pos3.tail(3), vmax[1],
			amax[1], jmax[1], -1, &(vel_cons[2]));
		_tf_rot = _rot_uplanner.GetFinalTime();
		_tf = _tf_rot;
		_tf_pos = 0;
	}
}

void EllipsePlanner::Reset(Vector6d pos, bool option)
{
	_t = 0;
}

void EllipsePlanner::GeneratePath(Vector6d& pos)
{
	RobotTools::CAVP cavp;
	RobotTools::CLineAVP line_avp, rpy_avp;
	if (_option == RobotTools::ePos)
	{
		line_avp = GeneratePosMotion(_t);
		rpy_avp.pos = _rpy_initial;
		rpy_avp.vel = Vector3d::Zero();
		rpy_avp.acc = Vector3d::Zero();
	}
	else if (_option == RobotTools::eRot)
	{
		rpy_avp = GenerateRotMotion(_t);
		line_avp.pos = _pos_initial;
		line_avp.vel = Vector3d::Zero();
		line_avp.acc = Vector3d::Zero();
	}
	else if (_option == RobotTools::eBoth)
	{
		line_avp = GeneratePosMotion(_t);
		rpy_avp = GenerateRotMotion(_t);
	}
	else
	{
		line_avp.pos = _pos_initial;
		line_avp.vel = Vector3d::Zero();
		line_avp.acc = Vector3d::Zero();
		rpy_avp.pos = _rpy_initial;
		rpy_avp.vel = Vector3d::Zero();
		rpy_avp.acc = Vector3d::Zero();
	}
	cavp = toSpatial(&line_avp, &rpy_avp);
	pos = cavp.pos;
	_t += _cycle_time;
	MathTools::LimitMax(_tf, _t);
	if (fabs(_t - _tf)<EPS3)
		_plan_completed = true;
}

void EllipsePlanner::GenerateMotion(Vector6d& pos, Vector6d& vel, Vector6d& acc)
{
	RobotTools::CAVP cavp;
	RobotTools::CLineAVP line_avp, rpy_avp;
	if (_option == RobotTools::ePos)
	{
		line_avp = GeneratePosMotion(_t);
		rpy_avp.pos = _rpy_initial;
		rpy_avp.vel = Vector3d::Zero();
		rpy_avp.acc = Vector3d::Zero();
	}
	else if (_option == RobotTools::eRot)
	{
		rpy_avp = GenerateRotMotion(_t);
		line_avp.pos = _pos_initial;
		line_avp.vel = Vector3d::Zero();
		line_avp.acc = Vector3d::Zero();
	}
	else if (_option == RobotTools::eBoth)
	{
		line_avp = GeneratePosMotion(_t);
		rpy_avp = GenerateRotMotion(_t);
	}
	else
	{
		line_avp.pos = _pos_initial;
		line_avp.vel = Vector3d::Zero();
		line_avp.acc = Vector3d::Zero();
		rpy_avp.pos = _rpy_initial;
		rpy_avp.vel = Vector3d::Zero();
		rpy_avp.acc = Vector3d::Zero();
	}
	cavp = toSpatial(&line_avp, &rpy_avp);
	pos = cavp.pos;
	vel = cavp.vel;
	acc = cavp.acc;
	_t += _cycle_time;
	MathTools::LimitMax(_tf, _t);
	if (fabs(_t - _tf)<EPS3)
		_plan_completed = true;
}

void EllipsePlanner::InitPosPlanner(Vector3d pos1, Vector3d pos2, Vector3d pos3, double line_vmax,
	double line_amax, double line_jmax, double tf, double* vel_cons)
{
	InitEllipseInfo(pos1, pos2, pos3);
	if (tf<0)
		_pos_uplanner.InitPlanner(Vector2d(0, _pos_len), line_vmax,
			line_amax, line_jmax, Vector2d(0, -1), Vector2d(vel_cons[0], vel_cons[1]));
	else
		_pos_uplanner.InitPlanner(Vector2d(0, _pos_len), line_vmax,
			line_amax, line_jmax, Vector2d(0, tf), Vector2d(vel_cons[0], vel_cons[1]));
}

void EllipsePlanner::InitEllipseInfo(Vector3d pos1, Vector3d pos2, Vector3d pos3)
{
	_center = (pos1 + pos3)/2;
	Vector3d vec_a = pos2 - _center;
	Vector3d vec_b = pos1 - _center;
	_a = vec_a.norm();
	_b = vec_b.norm();
	_max_ab = max(_a, _b);
	double theta = fabs(_angle_start - _angle_end);
	_pos_len = _max_ab*theta;
	Vector3d zr = vec_b.cross(vec_a);
	zr.normalize();
	vec_b.normalize();
	Vector3d yr = zr.cross(vec_b);
	yr.normalize();
	_rot << vec_b, yr, zr;
}

void EllipsePlanner::InitRotPlanner(Vector3d rpy0, Vector3d rpyn, double ang_vmax, double ang_amax, double ang_jmax, double tf, double* vel_cons)
{
	_r0 = RobotTools::FixedZYX2Tr(rpy0);
	_rn = RobotTools::FixedZYX2Tr(rpyn);
	Matrix3d r = _r0.inverse() * _rn;
	_angle_axis = RobotTools::Tr2AngleAxis(r);
	_rot_len = _angle_axis(3);
	Vector2d vp(0, _rot_len), velcons(vel_cons[0], vel_cons[1]);
	if (tf<0)
		_rot_uplanner.InitPlanner(vp, ang_vmax, ang_amax, ang_jmax, Vector2d(0, -1), velcons);
	else
		_rot_uplanner.InitPlanner(vp, ang_vmax, ang_amax, ang_jmax, Vector2d(0, tf), velcons);
}

void EllipsePlanner::CalcTrajOption(Vector6d pos_rpy1, Vector6d pos_rpy2)
{
	Vector4d angleaxis = RobotTools::CalcAngleAxis(pos_rpy1.tail(3), pos_rpy2.tail(3));
	double rpy_len = angleaxis(3);
	double pos_len = fabs(_angle_start - _angle_end);
	if ((pos_len>EPS) && (rpy_len>EPS))
		_option = RobotTools::eBoth;
	else if ((pos_len>EPS) && (rpy_len <= EPS))
		_option = RobotTools::ePos;
	else if ((pos_len <= EPS) && (rpy_len>EPS))
		_option = RobotTools::eRot;
	else
		_option = RobotTools::eNone;
}

RobotTools::CLineAVP EllipsePlanner::GeneratePosMotion(double t)
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
	double th = uavp.pos / _max_ab + _angle_start;
	Vector3d parc = Vector3d::Zero();
	parc(0) = _b*cos(th);
	parc(1) = _a*sin(th);
	line_avp.pos = _center + _rot*parc;
	Vector3d vc = Vector3d::Zero();
	Vector3d ac = Vector3d::Zero();
	vc(0) = -uavp.vel*sin(th);
	vc(1) = uavp.vel*cos(th);
	line_avp.vel = _rot*vc;
	ac(0) = -pow(uavp.vel, 2)*cos(th) / _max_ab - uavp.acc*sin(th);
	ac(1) = -pow(uavp.vel, 2)*sin(th) / _max_ab + uavp.acc*cos(th);
	line_avp.acc = _rot*ac;

	return line_avp;
}

RobotTools::CLineAVP EllipsePlanner::GenerateRotMotion(double t)
{
	RobotTools::CLineAVP rpy_avp;
	RobotTools::JAVP uavp;
	if (t > _tf)
	{
		uavp.pos = _rot_len;
		uavp.vel = 0;
		uavp.acc = 0;
	}
	else
	{
		uavp = _rot_uplanner.GenerateMotion(t);
	}
	Vector4d temp_e;
	temp_e << _angle_axis(0), _angle_axis(1), _angle_axis(2), uavp.pos;
	Matrix3d temp_r = RobotTools::AngleAxis2Tr(temp_e);
	Matrix3d r = _r0* temp_r;
	rpy_avp.pos = RobotTools::Tr2FixedZYX(r);
	Vector3d axis;
	axis << _angle_axis(0), _angle_axis(1), _angle_axis(2);
	rpy_avp.vel = _r0*(axis*uavp.vel);
	rpy_avp.acc = _r0*(axis*uavp.acc);
	return rpy_avp;
}
