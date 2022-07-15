#pragma once
#include "BaseTrajPlanner.h"
#include "LspbPlanner.h"
#include "SVelProfilePlanner.h"

class CirclePlanner : public BaseTrajPlanner
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
private:
	Vector3d _center;
	double _radius;
	double _theta;
	Matrix3d _rot;
	double _angle;
	bool _variable_attitude;

	Vector3d _pos_initial, _rpy_initial;
	double _pos_len;
	double  _tf;
	SVelProfilePlanner _pos_uplanner;

public:
	CirclePlanner() {}

	/** @param		traj_opt	false: use pos_rpy1 attitude all the path
								true: variable attitude, 
								start pos_rpy axis X: from center to pos1; xis Z: Normal vector of pos1->pos2->pos3 plane, 
								then rotate angle around the axis Y.
					angle	    angle between the normal vector and circular plane: pi/2 perpendicular
		@example	task_planner.AddTraj(&via_pos, eCircle, true ,1,1,pi/3);
					task_planner.AddTraj(&via_pos, eCircle, false);
	**/
	CirclePlanner(Vector6d* pos_rpy1, Vector3d* pos2, Vector3d* pos3,
		bool traj_opt, double* vmax, double* amax, double* jmax, double cycle_time, double angle);

	void AddViaPos(MatrixXd* via_pos, double* vmax, double* amax) override {};

	void Reset(Vector6d pos, bool option) override;

	void GeneratePath(Vector6d& pos) override;

	void GenerateMotion(Vector6d& pos, Vector6d& vel, Vector6d& acc) override;

	~CirclePlanner() {}

private:
	void InitPosPlanner(Vector3d pos1, Vector3d pos2, Vector3d pos3, double line_vmax,
		double line_amax, double line_jmax, double tf, double* vel_cons);

	void InitCircleInfo(Vector3d pos1, Vector3d pos2, Vector3d pos3);

	Vector4d PointsCoplane(Vector3d pos1, Vector3d pos2, Vector3d pos3);

	Vector4d RadiusEqual(Vector3d pos1, Vector3d pos2);

	RobotTools::CLineAVP GeneratePosMotion(double t);

	RobotTools::CLineAVP GenerateRotMotion(double t);

};