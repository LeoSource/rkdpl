/**
* @file		RobotDynamics.h
* @brief	dynamics algorithm for 6-dof robot
* @version	1.0.1
* @author	zxliao
* @email	zhixiangleo@163.com
* @date		2021/11/12
**/

#pragma once

#include "RobotMath.h"
#include "Biquad.h"
#include "UrRobot.h"

struct FeedForwardParams
{
	VectorXd grav_scale;
	VectorXd fric_scale;
};

struct CollisionParams
{
	bool check_state;
	int collision_count[6];
	VectorXd tau_external_base;
	VectorXd tau_external_limit;
};

struct AdmittanceParams
{
	VectorXd invk;
	VectorXd wn;
	VectorXd epsilon;
	VectorXd vmax;
};

struct JointAdmittanceData
{
	VectorXd pos_err;
	VectorXd vel_err;
	VectorXd acc_err;
};

struct CartesianAdmittanceData
{
	VectorXd pos_err;
	VectorXd vel_err;
	VectorXd acc_err;
	bool initialized;
	RobotTools::Pose pose0;
	VectorXd jpos_err;
};

class RobotDynamics
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:
	UrRobot* _rbtdef;
	VectorXd _barycenter_params;
	VectorXd _fric_params;
	VectorXd _tau_external_base;
	int _n_barycenter, _n_fric;
	int _njoint;
	Biquad _tau_filter[6];
	double _g;
	double _cycle_time;

	AdmittanceParams _joint_admittance_params, _cart_admittance_params;
	JointAdmittanceData _joint_admittance_data;
	CartesianAdmittanceData _cart_admittance_data;

public:
	RobotDynamics() = default;
	RobotDynamics(const RobotDynamics& rbtdyn) = delete;
	RobotDynamics& operator=(const RobotDynamics& rbtdyn) = delete;

	~RobotDynamics() = default;

	void InitDynamics(UrRobot* rbt, VectorXd barycenter_params,
					VectorXd fric_params, VectorXd tau_external_base);

	void InitAdmittanceControl();

	void GenerateGravityTorque(VectorXd& tau_grav, const VectorXd& jpos);

	void GenerateFrictionTorque(VectorXd& tau_fric, const VectorXd& jvel);

	void GenerateIdenTorque(VectorXd& tau_iden, const VectorXd& jpos, const VectorXd& jvel);

	void GenerateExternalTorque(VectorXd& tau_external,
			const VectorXd& tau_iden, const VectorXd& tau_fdb);

	void JointAdmittanceControl(VectorXd& pos_compliance,const VectorXd& tau_external);

	//TO DO: OPTIMIZE ORIENTATION ADMITTANCE CONTROL LOOP
	void CartesianAdmittanceControl(VectorXd& pos_compliance, const VectorXd& tau_external,
			const VectorXd& jpos);

	void ResetAdmittanceControl();

private:
	MatrixXd CalcRegGravMat(const VectorXd& jpos);

	RowVectorXd CalcRegressorJoint5(const VectorXd& jpos);

	RowVectorXd CalcRegressorJoint4(const VectorXd& jpos);

	RowVectorXd CalcRegressorJoint3(const VectorXd& jpos);

	RowVectorXd CalcRegressorJoint2(const VectorXd& jpos);

	int SignVel(double vel);

};