#pragma once
#include "MDHLink.hpp"
#include <vector>

using namespace Eigen;
using namespace RobotTools;

//#define arm_singularity 1
//#define wrist_singularity 2
//#define outside_reach 3

class UrRobot
{
public:
	Matrix<double, 6, 4, RowMajor> _mdh_table;
	Vector6d _offset;
	Matrix<int, 6, 1> _type;
	int _nlinks;
	MatrixXd _qlimit;

private:
	vector<MDHLink> _links;
	Pose _tool;
	VectorXd _hold_jpos;

public:
	UrRobot() {}

	UrRobot(MatrixXd mdh_table, Matrix<int, Dynamic, 1> type, VectorXd offset, Pose tool);

	void SetJntLimit(MatrixXd qlimit);

	Pose FKSolve(VectorXd q);

	Pose FKSolveFast(VectorXd q);

	Pose FKSolveTool2(VectorXd q, MatrixXd DH, Pose tool);

	Pose FKSolveTool(VectorXd q);

	int IKSolve(Vector6d *q_out, Pose *pose, Vector6d *q_old);

	int IKSolve(Vector6d * q_out, Pose * pose, Vector6d * q_old, Vector3i config);

	int IKSolveRecursive(Vector6d *q_out, Pose *pose, Vector6d *q_old);

	MatrixXd CalcJaco(VectorXd q);

	MatrixXd CalcJacoFast(VectorXd q);

	MatrixXd CalcBodyJaco(VectorXd q);

	MatrixXd CalcJv(VectorXd q);

	MatrixXd CalcJw(VectorXd q);

	MatrixXd CalcThToolJaco(Vector6d q, MatrixXd DH, Pose tool);

	void UpdateJntHoldPos(VectorXd q);

	VectorXd HoldJntPos();

    void UpdateTool(Vector3d tool_pos, Vector3d tool_rpy);

	double GetToolLength();

	Vector6d CalcJacodot(Vector6d q, Vector6d qd);

	Vector3i CalcConfig(VectorXd q);

	~UrRobot() {}

private:

	Pose Transform(VectorXd q, int s_idx, int e_idx);
};