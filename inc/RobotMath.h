#pragma once
#include "ErrorID.h"
#include <algorithm>
#include <math.h>
#include <Eigen/Dense>
#include <string>

#define	EPS	1e-5
#define	EPS3 1e-3
#define EPS4 1e-4
#define EPS5 1e-5
#define EPS6 1e-6
#define EPS7 1e-7
#define EPS10 1e-10
#define pi	3.141592657

const double D2R = pi/180.0;
const double R2D = 180.0/pi;

using namespace std;
using namespace Eigen;

namespace MathTools
{
	int Discretize(double* arr, int len, double value);

	int Discretize(VectorXd vec, int len, double value);

	double CalcTransEqua(double a, double b, double c, double pre_value);

	int CalcSinCosEqua(double *x1, double *x2, double a, double b, double c);

	int Sign(double x);

    int LimitNum(double min_value, double& value, double max_value);

	void LimitMin(double min_value, double& value);

	void LimitMax(double max_value, double& value);

    int LimitVector(VectorXd min_vec, VectorXd* value, VectorXd max_vec);

	bool Any(VectorXd vec);

	double Norm(VectorXd vec);

	Vector3d Cross(Vector3d v1, Vector3d v2);

	int Factorial(int n);

	template<typename T>
	T LimitMaxValue(T max_value, T value)
	{
		T res;
		if (value>max_value)
			res = max_value;
		else
			res = value;

		return res;
	}

	Vector3d CalcXZPlaneIntersection(Vector3d norm_vec, Vector3d via_point);

}

using Vector5d = Matrix<double, 5, 1>;
using Vector6d = Matrix<double, 6, 1>;
using Matrix6d = Matrix<double, 6, 6>;

namespace RobotTools
{
	struct JAVP
	{
		double pos;
		double vel;
		double acc;
	};

	struct CLineAVP
	{
		Vector3d pos;
		Vector3d vel;
		Vector3d acc;
	};

	struct CAVP
	{
		Vector6d pos;
		Vector6d vel;
		Vector6d acc;
	};

	struct Pose
	{
		Vector3d pos;
		Matrix3d rot;
	};

	enum TrajType
	{
		eJointSpace,
		eCartesianSpace,
		eCartesianArc,
		eBSpline,
		eCircle,
		eEllipse,
		eDynIden
	};

	enum CTrajRange
	{
		ePos,
		eRot,
		eBoth,
		eNone
	};

	Pose PoseProduct(Pose p1, Pose p2);

	Pose PoseInverse(Pose p1);

	Matrix3d RotX(double angle);

	Matrix3d RotY(double angle);

	Matrix3d RotZ(double angle);

	CAVP LinetoSpatial(CLineAVP* line_avp);

	CAVP AngtoSpatial(CLineAVP* ang_avp);

	CAVP toSpatial(CLineAVP* line_avp, CLineAVP* ang_avp);

	Matrix3d RPY2Jaco(Vector3d rpy);

	Vector4d Tr2Quat1(Matrix3d r);

	Vector4d Tr2Quat2(Matrix3d r);

	Vector4d Tr2AngleAxis(Matrix3d r);

	Matrix3d AngleAxis2Tr(Vector4d E);

	Vector3d Tr2FixedZYX(Matrix3d r);

	Matrix3d FixedZYX2Tr(Vector3d rpy);

	Vector4d CalcAngleAxis(Vector3d rpy0, Vector3d rpyn);

	Matrix6d InvAdT(Pose pose);

	Matrix3d Skew(Vector3d w);

	Vector6d LogT(Pose pose);

	MatrixXd CalcSplineTransPos(Vector3d pos1, Vector3d pos2, Vector3d pos3,
								double r, string opt);

	Vector4d RadiusEqual(Vector3d pos1, Vector3d pos2);

	Vector4d PointsCoplane(Vector3d pos1, Vector3d pos2, Vector3d pos3);

	double CalcArcRadius(Vector3d pos1, Vector3d pos2, Vector3d pos3);

	Matrix3d Rodrigues(Vector3d r, double angle);
}


