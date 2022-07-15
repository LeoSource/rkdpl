/**
* @file		LinePlanner.h
* @brief	Cartesian line trajectory plan
* @version	1.0.0
* @author	zxliao
* @email	zhixiangleo@163.com
* @date		2021/6/1
**/
#pragma once

#include "BaseCartesianPlanner.hpp"

class LinePlanner : public BaseCartesianPlanner
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
public:
	LinePlanner() {}

	/**
	* @brief	constructor
	* @author	zxliao
	* @date		2021/6/1
	* @param	pos0		initial position and rpy
	* @param	posn		final position and rpy
	* @param	vmax		spatial velocity constraint
	* @param	amax		spatial acceleration constraint
	* @param	duration	initial and final time
	* @param	vel_cons	initial and final spatial velocity
	**/
	LinePlanner(Vector6d pos0, Vector6d posn, double* vmax,
				double* amax, double* jmax, double* vel_cons, bool conti_type)
		:BaseCartesianPlanner(pos0, posn, vmax, amax, vel_cons, conti_type)
	{
		if (_option==RobotTools::eBoth)
		{
            InitPosPlanner(pos0.head(3), posn.head(3), vmax[0],
							amax[0],jmax[0], -1, vel_cons);
            InitRotPlanner(pos0.tail(3), posn.tail(3), vmax[1],
							amax[1], jmax[1], -1, &(vel_cons[2]));
			if (_continuity)
				_tf_pos = _pos_uplanner1.GetFinalTime();//lspb
			else
				_tf_pos = _pos_uplanner.GetFinalTime();
			_tf_rot = _rot_uplanner.GetFinalTime();
			Vector2d tf(_tf_pos, _tf_rot);
			_tf = tf.maxCoeff();
			if (_tf_pos>_tf_rot)
                InitRotPlanner(pos0.tail(3), posn.tail(3), vmax[1],
								amax[1], jmax[1], _tf, &(vel_cons[2]));
			else
                InitPosPlanner(pos0.head(3), posn.head(3), vmax[0],
								amax[0], jmax[0], _tf, vel_cons);
		}
		else if (_option==RobotTools::ePos)
		{
            InitPosPlanner(pos0.head(3), posn.head(3), vmax[0],
							amax[0], jmax[0], -1, vel_cons);
			if (_continuity)
				_tf_pos = _pos_uplanner1.GetFinalTime();//lspb
			else
				_tf_pos = _pos_uplanner.GetFinalTime();
			_tf = _tf_pos;
			_tf_rot = 0;
		}
		else if (_option==RobotTools::eRot)
		{
            InitRotPlanner(pos0.tail(3), posn.tail(3), vmax[1],
							amax[1],jmax[1], -1, &(vel_cons[2]));
			_tf_rot = _rot_uplanner.GetFinalTime();
			_tf = _tf_rot;
			_tf_pos = 0;
		}
		else if (_option==RobotTools::eNone)
		{
			_tf_pos = _tf_rot = _tf = 0;
		}
	}

	/**
	* @brief	constructor
	* @author	zxliao
	* @date		2021/6/1
	* @param	pos0		initial position and rpy
	* @param	posn		final position and rpy
	* @param	vmax		line or angular velocity constraint
	* @param	amax		line or angular acceleration constraint
	* @param	duration	initial and final time
	* @param	vel_cons	initial and final line or angular velocity
	**/
	LinePlanner(Vector6d pos0, Vector6d posn, double vmax,
				double amax, double jmax, double* vel_cons, bool conti_type)
		:BaseCartesianPlanner(pos0, posn, vmax, amax, vel_cons, conti_type)
	{
		if (_option==RobotTools::ePos)
		{
			InitPosPlanner(pos0.head(3), posn.head(3), vmax, amax,jmax, -1, vel_cons);
			if (_continuity)
				_tf_pos = _pos_uplanner1.GetFinalTime();//lspb
			else
				_tf_pos = _pos_uplanner.GetFinalTime();
			_tf = _tf_pos;
			_tf_rot = 0;
		}
		else if (_option==RobotTools::eRot)
		{
			InitRotPlanner(pos0.tail(3), posn.tail(3), vmax, amax,jmax, -1, vel_cons);
			_tf_rot = _rot_uplanner.GetFinalTime();
			_tf = _tf_rot;
			_tf_pos = 0;
		}
	}


	~LinePlanner() {}

private:
	/**
	* @brief	initialize position planner
	* @author	zxliao
	* @date		2021/6/1
	* @param	pos0		initial position
	* @param	posn		final position
	* @param	line_vmax	line velocity constraint
	* @param	line_amax	line acceleration constraint
	* @param	tf			final time
	* @param	vel_cons	initial and final line velocity
	**/
    void InitPosPlanner(Vector3d pos0, Vector3d posn, double line_vmax,
						double line_amax, double line_jmax, double tf, double* vel_cons) override
	{
		_pos_len = MathTools::Norm(posn-pos0);
		_pos_dir = (posn-pos0)/_pos_len;
		if (tf < 0)
		{
			if (_continuity)
				_pos_uplanner1.InitPlanner(Vector2d(0, _pos_len), line_vmax,
					line_amax, Vector2d(0, -1), Vector2d(vel_cons[0], vel_cons[1]));//lspb
			else
				_pos_uplanner.InitPlanner(Vector2d(0, _pos_len), line_vmax,
					line_amax, line_jmax, Vector2d(0, -1), Vector2d(vel_cons[0], vel_cons[1]));
		}
		else
		{
			if (_continuity)
				_pos_uplanner1.InitPlanner(Vector2d(0, _pos_len), line_vmax,
					line_amax, Vector2d(0, tf), Vector2d(vel_cons[0], vel_cons[1]));//lspb
			else
				_pos_uplanner.InitPlanner(Vector2d(0, _pos_len), line_vmax,
					line_amax, line_jmax, Vector2d(0, tf), Vector2d(vel_cons[0], vel_cons[1]));
		}
	}

    void InitPosPlanner(Vector3d pos1, Vector3d pos2, Vector3d pos3,
				double vmax, double amax, double jmax, double tf, double* vel_cons) override
	{	}

	/**
	* @brief	generate position motion data
	* @author	zxliao
	* @date		2021/6/3
	* @param	t		time
	* @return	line_avp	position, line velocity and acceleration
	**/
	RobotTools::CLineAVP GeneratePosMotion(double t) override
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
			if(_continuity)
				uavp = _pos_uplanner1.GenerateMotion(t);//lspb
			else
				uavp = _pos_uplanner.GenerateMotion(t);
		}
		line_avp.pos = _pos_initial+uavp.pos*_pos_dir;
		line_avp.vel = uavp.vel*_pos_dir;
		line_avp.acc = uavp.acc*_pos_dir;

		return line_avp;
	}


};

