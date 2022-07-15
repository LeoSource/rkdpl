#include "ForceControl.h"
#include "RobotDynamics.h"


ForceControl::ForceControl()
{
	_rbtdyn = nullptr;
	_ffd_params = nullptr;
	_col_params = nullptr;
	_njoint = 6;
}

void ForceControl::CreateForceControl(UrRobot* rbt,const AllDynamicsParams& all_params)
{
	_rbtdyn = new RobotDynamics;
	_rbtdyn->InitDynamics(rbt, all_params.barycenter_params, 
			all_params.fric_params,all_params.tau_external_base);
	_ffd_params = new FeedForwardParams;
	_ffd_params->grav_scale = all_params.grav_scale;
	_ffd_params->fric_scale = all_params.fric_scale;
	_col_params = new CollisionParams;
	_col_params->check_state = false;
	_col_params->tau_external_base = all_params.tau_external_base;
	_col_params->tau_external_limit = all_params.tau_external_limit;
	memset(_col_params->collision_count, 0,
		sizeof(_col_params->collision_count)/sizeof(_col_params->collision_count[0]));
}

void ForceControl::RobotDynamicsCalculation(const VectorXd& jpos,
					const VectorXd& jvel, const VectorXd& jtau)
{
	_rbtdyn->GenerateGravityTorque(_tau_grav, jpos);
	_rbtdyn->GenerateFrictionTorque(_tau_fric, jvel);
	_tau_dynamics = _tau_grav+_tau_fric;
	_tau_ffd = _ffd_params->grav_scale.cwiseProduct(_tau_grav)
		+_ffd_params->fric_scale.cwiseProduct(_tau_fric);
	_rbtdyn->GenerateExternalTorque(_tau_external, _tau_dynamics, jtau);
	_rbtdyn->CartesianAdmittanceControl(_pos_compliance, _tau_external, jpos);
	//_rbtdyn->JointAdmittanceControl(_pos_compliance, _tau_external);
}

VectorXd ForceControl::GetFeedforwardTorque()
{
	return _tau_ffd;
}

VectorXd ForceControl::GetGravityTorque()
{
	return _tau_grav;
}

VectorXd ForceControl::GetFrictionTorque()
{
	return _tau_fric;
}

VectorXd ForceControl::GetDynamicsTorque()
{
	return _tau_dynamics;
}

VectorXd ForceControl::GetExternalTorque()
{
	return _tau_external;
}

VectorXd ForceControl::GetCompliancePosition()
{
	return _pos_compliance;
}

void ForceControl::SwitchCollisionCheckState(bool check_state)
{
	_col_params->check_state = check_state;
	if (!check_state)
	{
		memset(_col_params->collision_count, 0,
			sizeof(_col_params->collision_count)/sizeof(_col_params->collision_count[0]));
	}
}

bool ForceControl::CheckCollision()
{
	if (_col_params->check_state)
	{
		for (int idx = 0; idx<_njoint; idx++)
		{
			if (fabs(_tau_external(idx))>_col_params->tau_external_limit(idx))
				_col_params->collision_count[idx]++;
			else
				_col_params->collision_count[idx] = 0;

			if (_col_params->collision_count[idx]>4)
			{
				memset(_col_params->collision_count, 0,
				sizeof(_col_params->collision_count)/sizeof(_col_params->collision_count[0]));
				return true;
			}
		}
	}
	return false;
}

ForceControl::~ForceControl()
{
	delete _rbtdyn;
	delete _ffd_params;
	delete _col_params;
	_rbtdyn = nullptr;
	_ffd_params = nullptr;
	_col_params = nullptr;
}
