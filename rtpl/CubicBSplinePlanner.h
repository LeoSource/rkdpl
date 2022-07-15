/**
* @file		CubicBSplinePlanner.h
* @brief	B-spline trajectory plan which includes position and rotation plan,
			rotation is defined by Roll-Pitch-Yaw.
			Rotation plan and position plan have the same method because they
			both are 3x1 vector.
* @version	2.0.1
* @author	zxliao
* @email	zhixiangleo@163.com
* @date		2021/10/25
**/
#pragma once

#include "BaseTrajPlanner.h"
#include "LspbPlanner.h"

class CubicBSplinePlanner : public BaseTrajPlanner
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
private:
	int _nump;
	int _num_ctrlp;
	int _pdegree;
	VectorXd _knot_vec;
	VectorXd _uknot_vec;
	MatrixXd _ctrl_pos;
	MatrixXd _ctrl_rot;
	bool _interp;
	LspbPlanner _uplanner;
	bool _rot_planned;

public:
	CubicBSplinePlanner() {}

	CubicBSplinePlanner(MatrixXd* via_pos, bool interp, double cycle_time);

	CubicBSplinePlanner(MatrixXd* via_pos, bool interp, double uk, double cycle_time);

	CubicBSplinePlanner(MatrixXd* via_pos, bool interp, VectorXd uk, double cycle_time);

	void GenerateMotion(Vector6d& cpos, Vector6d& cvel, Vector6d& cacc) override;
	
	void GeneratePath(Vector6d& cpos) override;

	void Reset(Vector6d pos_rpy, bool interp) override;

	void AddViaPos(MatrixXd* via_pos, double* vmax, double* amax) override {}

	~CubicBSplinePlanner() {}

private:
	RobotTools::CAVP GenerateMotion(double u, double du, double ddu);

	Vector6d GeneratePos(double u);

	Vector6d GenerateVel(double u, double du);

	Vector6d GenerateAcc(double u, double du, double ddu);

	void CalcBSplineParams(MatrixXd* via_pos);

	MatrixXd CalcCtrlPos(MatrixXd* q);

	MatrixXd CalcApproCtrlPos(MatrixXd* q);

	VectorXd CalcKnotVec();

	VectorXd CalcUKnot(MatrixXd* q);

	VectorXd CalcApproKnotVec();

	double CalcBSplineCoeff(int p, int idx, double u);

	double DiffBSplineCoeff(int p, int jidx, double u, int k);

	double CalcDiffCoeff(int k, int idx, int jidx);

	double Divide(double num, double den);
};

