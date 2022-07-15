/**
* @file		BaseCartesianPlanner.h
* @brief	Cartesian trajectory plan interface
* @version	1.0.0
* @author	zxliao
* @email	zhixiangleo@163.com
* @date		2021/6/1
**/
#pragma once

#include "LspbPlanner.h"
#include "SVelProfilePlanner.h"

using namespace std;
using namespace Eigen;

class BaseCartesianPlanner
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
protected:
	Vector3d _pos_initial, _rpy_initial;
	Vector3d _pos_dir, _rot_dir;
	Vector4d _angle_axis;
	double _pos_len, _rot_len;
	double _tf_pos, _tf_rot, _tf;
	SVelProfilePlanner _pos_uplanner, _rot_uplanner;
	LspbPlanner _pos_uplanner1;
	RobotTools::CTrajRange _option;
	Matrix3d _r0, _rn;
	bool _continuity;
public:
	BaseCartesianPlanner() {}

	/**
	* @brief	constructor
	* @author	zxliao
	* @date		2021/6/1
	* @param	pos0		initial position and rpy
	* @param	posn		final position and rpy
	* @param	vmax		spatial velocity constraint
	* @param	amax		spatial acceleration constraint
	* @param	vel_cons	initial and final spatial velocity
	**/
	BaseCartesianPlanner(Vector6d pos0, Vector6d posn, double* vmax, double* amax, double* vel_cons, bool conti_type)
	{
		CalcTrajOption(pos0, posn);
		_pos_initial = pos0.head(3);
		_rpy_initial = pos0.tail(3);
		_tf_pos = _tf_rot = _tf = 0;
		_pos_len = _rot_len = 0;
		_continuity = conti_type;
	}

	/**
	* @brief	constructor
	* @author	zxliao
	* @date		2021/6/1
	* @param	pos0		initial position and rpy
	* @param	posn		final position and rpy
	* @param	vmax		line or angular velocity constraint
	* @param	amax		line or angular acceleration constraint
	* @param	vel_cons	initial and final line or angular velocity
	**/
	BaseCartesianPlanner(Vector6d pos0, Vector6d posn, double vmax, double amax, double* vel_cons, bool conti_type)
	{
		CalcTrajOption(pos0, posn);
		_pos_initial = pos0.head(3);
		_rpy_initial = pos0.tail(3);
		_tf_pos = _tf_rot = _tf = 0;
		_pos_len = _rot_len = 0;
		_continuity = conti_type;
	}

	/**
	* @brief	generate Cartesian motion data
	* @author	zxliao
	* @date		2021/6/3
	* @param	t		time
	* @return	pos_rpy	position, rpy, spatial velocity and acceleration
	**/
	RobotTools::CAVP GenerateMotion(double t)
	{
		RobotTools::CLineAVP line_avp, rpy_avp;
		if (_option==RobotTools::ePos)
		{
			line_avp = GeneratePosMotion(t);
			rpy_avp.pos = _rpy_initial;
			rpy_avp.vel = Vector3d::Zero();
			rpy_avp.acc = Vector3d::Zero();
		}
		else if (_option==RobotTools::eRot)
		{
			rpy_avp = GenerateRotMotion(t);
			line_avp.pos = _pos_initial;
			line_avp.vel = Vector3d::Zero();
			line_avp.acc = Vector3d::Zero();
		}
		else if (_option==RobotTools::eBoth)
		{
			line_avp = GeneratePosMotion(t);
			rpy_avp = GenerateRotMotion(t);
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

		return toSpatial(&line_avp, &rpy_avp);
	}

	/**
	* @brief	generate position
	* @author	zxliao
	* @date		2021/6/1
	* @param	t		time
	* @return	pos_rpy	position and rpy
	**/
	Vector6d GeneratePoint(double t)
	{
		Vector6d pos_rpy;
		if (_option==RobotTools::ePos)
		{
			pos_rpy.head(3) = GeneratePos(t);
			pos_rpy.tail(3) = _rpy_initial;
		}
		else if (_option==RobotTools::eRot)
		{
			pos_rpy.head(3) = _pos_initial;
			pos_rpy.tail(3) = GenerateRot(t);
		}
		else if (_option==RobotTools::eBoth)
		{
			pos_rpy.head(3) = GeneratePos(t);
			pos_rpy.tail(3) = GenerateRot(t);
		}
		else
		{
			pos_rpy.head(3) = _pos_initial;
			pos_rpy.tail(3) = _rpy_initial;
		}

		return pos_rpy;
	}

	/**
	* @brief	get final time
	* @author	zxliao
	* @date		2021/6/1
	* @return	tf		final time of planner
	**/
	double GetFinalTime()
	{
		return _tf;
	}

    virtual ~BaseCartesianPlanner() {}

protected:
    virtual void InitPosPlanner(Vector3d pos0, Vector3d posn, double line_vmax, double line_amax, double line_jmax, double tf, double* vel_cons) = 0;

    virtual void InitPosPlanner(Vector3d pos1, Vector3d pos2, Vector3d pos3, double vmax, double amax, double jmax, double tf, double* vel_cons) = 0;

	virtual RobotTools::CLineAVP GeneratePosMotion(double t) = 0;

	/**
	* @brief	initialize rotation planner
	* @author	zxliao
	* @date		2021/6/1
	* @param	pos0		initial rpy
	* @param	posn		final rpy
	* @param	line_vmax	angular velocity constraint
	* @param	line_amax	angular acceleration constraint
	* @param	tf			final time
	* @param	vel_cons	initial and final angular velocity
	**/
    void InitRotPlanner(Vector3d rpy0, Vector3d rpyn, double ang_vmax, double ang_amax, double ang_jmax, double tf, double* vel_cons)
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

	/**
	* @brief	calculate trajectory option between 2 positions and rotations
	* @author	zxliao
	* @date		2021/6/28
	* @param	pos0		1st position and rotation
	* @param	posn		2nd position and rotation
	**/
	void CalcTrajOption(Vector6d pos_rpy1, Vector6d pos_rpy2)
	{
		double pos_len = MathTools::Norm(pos_rpy1.head(3)-pos_rpy2.head(3));
		Vector4d angleaxis = RobotTools::CalcAngleAxis(pos_rpy1.tail(3), pos_rpy2.tail(3));
		double rpy_len = angleaxis(3);
		if ((pos_len>EPS)&&(rpy_len>EPS))
			_option = RobotTools::eBoth;
		else if ((pos_len>EPS)&&(rpy_len<=EPS))
			_option = RobotTools::ePos;
		else if ((pos_len<=EPS)&&(rpy_len>EPS))
			_option = RobotTools::eRot;
		else
			_option = RobotTools::eNone;
	}

	/**
	* @brief	generate positin
	* @author	zxliao
	* @date		2021/6/1
	* @param	t	time
	* @return	pos	Cartesian position
	**/
	Vector3d GeneratePos(double t)
	{
		Vector3d pos;
		double up;
		if (t>_tf)
			up = _pos_len;
		else
		{
			RobotTools::JAVP uavp;
			if(_continuity)
				uavp = _pos_uplanner1.GenerateMotion(t);
			else
				uavp = _pos_uplanner.GenerateMotion(t);
			up = uavp.pos;
		}
		pos = _pos_initial+up*_pos_dir;

		return pos;
	}

	/**
	* @brief	generate rotation
	* @author	zxliao
	* @date		2021/6/1
	* @param	t	time
	* @return	rpy	roll, pitch, yaw angle
	**/
	Vector3d GenerateRot(double t)
	{
		Vector3d rpy;
		double up;
		if (t>_tf)
			up = _rot_len;
		else
		{
			RobotTools::JAVP uavp = _rot_uplanner.GenerateMotion(t);
			up = uavp.pos;
		}
		Vector4d temp_e;
		temp_e << _angle_axis(0), _angle_axis(1), _angle_axis(2), up;
		Matrix3d temp_r = RobotTools::AngleAxis2Tr(temp_e);
		Matrix3d r = _r0* temp_r;
		rpy = RobotTools::Tr2FixedZYX(r);

		return rpy;
	}

	/**
	* @brief	generate rpy motion data
	* @author	zxliao
	* @date		2021/6/3
	* @param	t		time
	* @return	rpy_avp	rpy, angular velocity and acceleration
	**/
	RobotTools::CLineAVP GenerateRotMotion(double t)
	{
		RobotTools::CLineAVP rpy_avp;
		RobotTools::JAVP uavp;
		if (t>_tf)
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
		//rpy_avp.vel.setZero();
		//rpy_avp.acc.setZero();
		Vector3d axis; 
		axis << _angle_axis(0), _angle_axis(1), _angle_axis(2);
		rpy_avp.vel = _r0*(axis*uavp.vel);
		rpy_avp.acc = _r0*(axis*uavp.acc);
		return rpy_avp;
	}



};

