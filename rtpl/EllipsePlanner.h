#pragma once
#include "BaseTrajPlanner.h"
#include "LspbPlanner.h"
#include "SVelProfilePlanner.h"

class EllipsePlanner : public BaseTrajPlanner
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
private:
	double _angle_start, _angle_end;
	Vector3d _center;
	double _a;
	double _b;
	double _max_ab;
	Matrix3d _rot;

	Vector3d _rpy_initial, _pos_initial;
	Vector3d _pos_dir, _rot_dir;
	Vector4d _angle_axis;
	double _pos_len, _rot_len;
	double  _tf_pos, _tf_rot, _tf;
	SVelProfilePlanner _pos_uplanner, _rot_uplanner;
	RobotTools::CTrajRange _option;
	Matrix3d _r0, _rn;

public:
	EllipsePlanner() {}

	/** @param		
	start pos_rpy axis X: from center to pos1; xis Z: Normal vector of pos1->pos2->pos3 plane,
	angle_start	  angle_end  elliptical arc length
	@example	task_planner.AddTraj(&via_pos, eEllipse, false ,1,1,pi/3,pi);
	**/
	EllipsePlanner(Vector6d* pos_rpy1, Vector3d* pos2, Vector6d* pos_rpy3,
		double* vmax, double* amax, double* jmax, double cycle_time, double angle_start, double angle_end);

	void AddViaPos(MatrixXd* via_pos, double* vmax, double* amax) override {};

	void Reset(Vector6d pos, bool option) override;

	void GeneratePath(Vector6d& pos) override;

	void GenerateMotion(Vector6d& pos, Vector6d& vel, Vector6d& acc) override;

	~EllipsePlanner() {}

private:
	void InitPosPlanner(Vector3d pos1, Vector3d pos2, Vector3d pos3, double line_vmax,
		double line_amax, double line_jmax, double tf, double* vel_cons);

	void InitEllipseInfo(Vector3d pos1, Vector3d pos2, Vector3d pos3);

	void InitRotPlanner(Vector3d rpy0, Vector3d rpyn, double ang_vmax, 
		double ang_amax, double ang_jmax, double tf, double* vel_cons);

	void CalcTrajOption(Vector6d pos_rpy1, Vector6d pos_rpy2);

	RobotTools::CLineAVP GeneratePosMotion(double t);

	RobotTools::CLineAVP GenerateRotMotion(double t);

};