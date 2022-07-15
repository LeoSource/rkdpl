#pragma once

#include "RobotMath.h"

using namespace std;
using namespace Eigen;

enum LinkType
{
	e_rotation = 0,
	e_prismatic = 1
};

class MDHLink
{
public:
	int _type;

private:
	double _theta;
	double _d;
	double _a;
	double _alpha;
	double _offset;

public:
	MDHLink() {}

	MDHLink(double theta, double d, double a, double alpha, int type, double offset):
		_theta(theta), _d(d), _a(a), _alpha(alpha), _type(type), _offset(offset)
	{}

	RobotTools::Pose Transform(double q)
	{
		RobotTools::Pose pose_link;
		if (_type==e_rotation)
			_theta = q+_offset;
		else
			_d = q+_offset;

		double st = sin(_theta); double sa = sin(_alpha);
		double ct = cos(_theta); double ca = cos(_alpha);
		pose_link.rot(0, 0) = ct;
		pose_link.rot(0, 1) = -st;
		pose_link.rot(0, 2) = 0;

		pose_link.rot(1, 0) = st*ca;
		pose_link.rot(1, 1) = ct*ca;
		pose_link.rot(1, 2) = -sa;

		pose_link.rot(2, 0) = st*sa;
		pose_link.rot(2, 1) = ct*sa;
		pose_link.rot(2, 2) = ca;

		pose_link.pos<<_a, -sa*_d, ca*_d;

		return pose_link;
	}

	~MDHLink() {}
};

