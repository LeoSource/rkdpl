/**
* @file		ForceControl.h
* @brief	wrapper of robot dynamics
* @version	1.0.0
* @author	zxliao
* @email	zhixiangleo@163.com
* @date		2022/01/06
**/
#pragma once

#include "RobotMath.h"

class RobotDynamics;
class UrRobot;
struct FeedForwardParams;
struct CollisionParams;

struct AllDynamicsParams
{
	VectorXd barycenter_params;
	VectorXd fric_params;
	VectorXd grav_scale;
	VectorXd fric_scale;
	VectorXd tau_external_base;
	VectorXd tau_external_limit;
};


class ForceControl
{
private:
	RobotDynamics* _rbtdyn;
	FeedForwardParams* _ffd_params;
	CollisionParams* _col_params;
	int _njoint;

	VectorXd _tau_dynamics;
	VectorXd _tau_grav;
	VectorXd _tau_fric;
	VectorXd _tau_ffd;
	VectorXd _tau_external;

	VectorXd _pos_compliance;

public:
	ForceControl();
	ForceControl(const ForceControl& fc) = delete;
	ForceControl& operator=(const ForceControl& fc) = delete;

	void CreateForceControl(UrRobot* rbt, const AllDynamicsParams& all_params);

	void RobotDynamicsCalculation(const VectorXd& jpos,
			const VectorXd& jvel,const VectorXd& jtau);

	void SwitchCollisionCheckState(bool check_state);

	bool CheckCollision();

	VectorXd GetGravityTorque();

	VectorXd GetFrictionTorque();

	VectorXd GetDynamicsTorque();

	VectorXd GetFeedforwardTorque();

	VectorXd GetExternalTorque();

	VectorXd GetCompliancePosition();

	~ForceControl();

};

