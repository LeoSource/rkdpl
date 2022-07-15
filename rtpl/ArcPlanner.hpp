/**
* @file		ArcPlanner.h
* @brief	Cartesian arc trajectory plan
* @version	1.1.0
* @author	zxliao
* @email	zhixiangleo@163.com
* @date		2021/6/28
**/
#pragma once

#include "BaseCartesianPlanner.hpp"

class ArcPlanner : public BaseCartesianPlanner
{
private:
	Vector3d _center;
	double _radius;
	double _theta;
	Matrix3d _rot;
	string _arc_style;

public:
	ArcPlanner() {}

	/**
	* @brief	constructor
	* @author	zxliao
	* @date		2021/6/3
	* @param	pos0		initial position and rpy
	* @param	posn		final position and rpy
	* @param	vmax		spatial velocity constraint
	* @param	amax		spatial acceleration constraint
	* @param	duration	initial and final time
	* @param	vel_cons	initial and final spatial velocity
	* @param	arc_style	arc style: arc or arctrans
	**/
	ArcPlanner(Vector6d pos1, Vector3d pos2, Vector6d pos3,
				double* vmax, double* amax, double* jmax, double* vel_cons, string arc_style, bool conti_type)
		:BaseCartesianPlanner(pos1, pos3, vmax, amax, vel_cons, conti_type)
	{
		_arc_style = arc_style;
		if (_option==RobotTools::eBoth)
		{
            InitPosPlanner(pos1.head(3), pos2,pos3.head(3),
							vmax[0], amax[0],jmax[0], -1, vel_cons);
            InitRotPlanner(pos1.tail(3), pos3.tail(3), vmax[1],
							amax[1], jmax[1], -1, &(vel_cons[2]));
			if(_continuity)
				_tf_pos = _pos_uplanner1.GetFinalTime();
			else
				_tf_pos = _pos_uplanner.GetFinalTime();
			_tf_rot = _rot_uplanner.GetFinalTime();
			Vector2d tf(_tf_pos, _tf_rot);
			_tf = tf.maxCoeff();
			if (_tf_pos>_tf_rot)
                InitRotPlanner(pos1.tail(3), pos3.tail(3), vmax[1],
								amax[1], jmax[1], _tf, &(vel_cons[2]));
			else
                InitPosPlanner(pos1.head(3), pos2,pos3.head(3),
								vmax[0], amax[0], jmax[0], _tf, vel_cons);
		}
		else if (_option==RobotTools::ePos)
		{
            InitPosPlanner(pos1.head(3), pos2,pos3.head(3),
							vmax[0], amax[0],jmax[0], -1, vel_cons);
			if(_continuity)
				_tf_pos = _pos_uplanner1.GetFinalTime();
			else
				_tf_pos = _pos_uplanner.GetFinalTime();
			_tf = _tf_pos;
			_tf_rot = 0;
		}
		else if (_option==RobotTools::eRot)
		{
            InitRotPlanner(pos1.tail(3), pos3.tail(3), vmax[1],
							amax[1],jmax[1], -1, &(vel_cons[2]));
			_tf_rot = _rot_uplanner.GetFinalTime();
			_tf = _tf_rot;
			_tf_pos = 0;
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
	* @param	arc_style	arc style: arc or arctrans
	**/
	ArcPlanner(Vector6d pos1,Vector3d pos2, Vector6d pos3, double vmax,
				double amax, double jmax, double* vel_cons, bool conti_type)
		:BaseCartesianPlanner(pos1, pos3, vmax, amax, vel_cons, conti_type)
	{
		_arc_style = _arc_style;
		if (_option==RobotTools::ePos)
		{
            InitPosPlanner(pos1.head(3), pos2,pos3.head(3), vmax,
							amax, jmax, -1, vel_cons);
			if(_continuity)
				_tf_pos = _pos_uplanner1.GetFinalTime();
			else
				_tf_pos = _pos_uplanner.GetFinalTime();
			_tf = _tf_pos;
			_tf_rot = 0;
		}
		else if (_option==RobotTools::eRot)
		{
            InitRotPlanner(pos1.tail(3), pos3.tail(3), vmax,
							amax, jmax, -1, vel_cons);
			_tf_rot = _rot_uplanner.GetFinalTime();
			_tf = _tf_rot;
			_tf_pos = 0;
		}
	}

	~ArcPlanner() {}

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
    void InitPosPlanner(Vector3d pos1, Vector3d pos2, Vector3d pos3, double line_vmax,
					double line_amax, double line_jmax, double tf, double* vel_cons) override
	{
		InitArcInfo(pos1, pos2, pos3);
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

	void InitArcInfo(Vector3d pos1, Vector3d pos2, Vector3d pos3)
	{
		if (_arc_style == "arctrans")
		{
			Vector3d p2p1 = pos1-pos2;
			Vector3d p2p3 = pos3-pos2;
			double inc_angle = acos(p2p1.dot(p2p3)/p2p1.norm()/p2p3.norm());
			MathTools::LimitNum(-1, inc_angle, 1);
			_theta = acos(1 - 2 * (1 - inc_angle) * (1 + inc_angle));
			if (inc_angle > 0)
				_theta = 2 * pi - _theta;
			_radius = p2p1.norm()*tan(0.5*inc_angle);
			Vector3d pc = 0.5*(pos1+pos3);
			Vector3d p2pc = pc-pos2;
			double scale = _radius/sin(0.5*inc_angle)/p2pc.norm();
			Vector3d p2center = scale*p2pc;
			_center = pos2+p2center;
			Vector3d n = (pos1-_center)/MathTools::Norm(pos1-_center);
			Vector4d tmp_a = PointsCoplane(pos1, pos2, pos3);
			Vector3d a = tmp_a.head(3);
			a.normalize();
			Vector3d o = a.cross(n);
			_rot<<n, o, a;
			_pos_len = _radius*_theta;
		}
		else if (_arc_style == "arc")
		{
			Vector4d params1 = PointsCoplane(pos1, pos2, pos3);
			Vector4d params2 = RadiusEqual(pos1, pos2);
			Vector4d params3 = RadiusEqual(pos1, pos3);
			double a1 = params1(0), b1 = params1(1), c1 = params1(2), d1 = params1(3);
			double a2 = params2(0), b2 = params2(1), c2 = params2(2), d2 = params2(3);
			double a3 = params3(0), b3 = params3(1), c3 = params3(2), d3 = params3(3);
			_center(0) = -(b1*c2*d3-b1*c3*d2-b2*c1*d3+b2*c3*d1+b3*c1*d2-b3*c2*d1)/
				(a1*b2*c3-a1*b3*c2-a2*b1*c3+a2*b3*c1+a3*b1*c2-a3*b2*c1);
			_center(1) = (a1*c2*d3-a1*c3*d2-a2*c1*d3+a2*c3*d1+a3*c1*d2-a3*c2*d1)/
				(a1*b2*c3-a1*b3*c2-a2*b1*c3+a2*b3*c1+a3*b1*c2-a3*b2*c1);
			_center(2) = -(a1*b2*d3-a1*b3*d2-a2*b1*d3+a2*b3*d1+a3*b1*d2-a3*b2*d1)/
				(a1*b2*c3-a1*b3*c2-a2*b1*c3+a2*b3*c1+a3*b1*c2-a3*b2*c1);
			Vector3d line_vec = pos3-pos1;
			double line_length = line_vec.norm();
			Vector3d xr = pos1-_center;
			_radius = xr.norm();
			//double tmp_cos = (pow(_radius, 2)*2-pow(line_length, 2))/(2*_radius*_radius);
			//MathTools::LimitNum(-1, tmp_cos, 1);
			//_theta = acos(tmp_cos);
			/*central angle > 180 deg*/
			Vector3d line_vec1 = pos1 - pos2;
			Vector3d line_vec2 = pos3 - pos2;
			double cos_a = line_vec1.dot(line_vec2) / (line_vec1.norm()*line_vec2.norm());
			MathTools::LimitNum(-1, cos_a, 1);
			_theta = acos(1 - 2 * (1 - cos_a) * (1 + cos_a));
			if (cos_a > 0)
				_theta = 2 * pi - _theta;
			Vector3d zr(a1, b1, c1);
			Vector3d yr = zr.cross(xr);
			xr.normalize(); yr.normalize(); zr.normalize();
			_rot<<xr, yr, zr;
			_pos_len = _radius*_theta;
		}
	}

	/**
	* @brief	calculate plane function according to 3 points in the plane
	* @author	zxliao
	* @date		2021/6/28
	* @param	pos1		1st position
	* @param	pos2		2nd position
	* @param	pos3		3rd position
	* @return	plane_parmas	plane function parameters
	**/
	Vector4d PointsCoplane(Vector3d pos1, Vector3d pos2, Vector3d pos3)
	{
		double x1 = pos1(0), y1 = pos1(1), z1 = pos1(2);
		double x2 = pos2(0), y2 = pos2(1), z2 = pos2(2);
		double x3 = pos3(0), y3 = pos3(1), z3 = pos3(2);
		double a = y1*z2-y2*z1-y1*z3+y3*z1+y2*z3-y3*z2;
		double b = -(x1*z2-x2*z1-x1*z3+x3*z1+x2*z3-x3*z2);
		double c = x1*y2-x2*y1-x1*y3+x3*y1+x2*y3-x3*y2;
		double d = -(x1*y2*z3-x1*y3*z2-x2*y1*z3+x2*y3*z1+x3*y1*z2-x3*y2*z1);
		
		return Vector4d(a, b, c, d);
	}

	/**
	* @brief	calculate plane function according to that 2 points of arc are
				equidistant from the centre
	* @author	zxliao
	* @date		2021/6/28
	* @param	pos1		1st position
	* @param	pos2		2nd position
	* @param	pos3		3rd position
	* @return	plane_parmas	plane function parameters
	**/
	Vector4d RadiusEqual(Vector3d pos1, Vector3d pos2)
	{
		Vector4d res;
		res(0) = 2*(pos2(0)-pos1(0));
		res(1) = 2*(pos2(1)-pos1(1));
		res(2) = 2*(pos2(2)-pos1(2));
		res(3) = pow(pos1(0), 2)+pow(pos1(1), 2)+pow(pos1(2), 2)
			-pow(pos2(0), 2)-pow(pos2(1), 2)-pow(pos2(2), 2);

		return res;
	}

    void InitPosPlanner(Vector3d pos0, Vector3d posn, double line_vmax,
						double line_amax, double line_jmax, double tf, double* vel_cons) override
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
			if (_continuity)
				uavp = _pos_uplanner1.GenerateMotion(t);//lspb
			else
				uavp = _pos_uplanner.GenerateMotion(t);
		}
		double r = _radius;
		double th = uavp.pos/r;
		Vector3d parc = Vector3d::Zero();
		parc(0) = r*cos(th);
		parc(1) = r*sin(th);
		line_avp.pos = _center+_rot*parc;
		Vector3d vc = Vector3d::Zero();
		Vector3d ac = Vector3d::Zero();
		vc(0) = -uavp.vel*sin(th);
		vc(1) = uavp.vel*cos(th);
		line_avp.vel = _rot*vc;
		ac(0) = -pow(uavp.vel, 2)*cos(th)/r-uavp.acc*sin(th);
		ac(1) = -pow(uavp.vel, 2)*sin(th)/r+uavp.acc*cos(th);
		line_avp.acc = _rot*ac;

		return line_avp;
	}


};


