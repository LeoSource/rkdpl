#ifndef _USE_MATH_DEFINES
#define _USE_MATH_DEFINES
#endif // _USE_MATH_DEFINES

#include <math.h>
#include "RobotDynamics.h"

void RobotDynamics::InitDynamics(UrRobot* rbt, VectorXd barycenter_params,
					VectorXd fric_params,VectorXd tau_external_base)
{
	_rbtdef = rbt;
	_barycenter_params = barycenter_params;
	_fric_params = fric_params;
	_tau_external_base = tau_external_base;
	_njoint = 6;
	_n_barycenter = (int)(barycenter_params.size());
	_n_fric = (int)(fric_params.size());
	for (int idx = 0; idx<6; idx++)
	{
		_tau_filter[idx].SetBiquad(eLowpass, 10.0/200.0, sqrt(2)/2.0, 0);
	}
	_g = 9.81;
	_cycle_time = 0.005;
	InitAdmittanceControl();
}

void RobotDynamics::InitAdmittanceControl()
{
	//initialize joint admittance controller
	_joint_admittance_data.pos_err.setZero(_njoint);
	_joint_admittance_data.vel_err.setZero(_njoint);
	_joint_admittance_data.acc_err.setZero(_njoint);
	_joint_admittance_params.epsilon.setZero(_njoint);
	_joint_admittance_params.invk.setZero(_njoint);
	_joint_admittance_params.vmax.setZero(_njoint);
	_joint_admittance_params.wn.setZero(_njoint);
	_joint_admittance_params.invk<<1/40.0, 1/70.0, 1/50.0, 1/15.0, 1/15.0, 1/5.0;
	_joint_admittance_params.epsilon<<1.0, 1.0, 1.0, 1.0, 1.0, 1.0;
	_joint_admittance_params.vmax<<M_PI/4.0, M_PI/4.0, M_PI/4.0, M_PI/4.0, M_PI/4.0, M_PI/4.0;
	_joint_admittance_params.wn<<2*M_PI*3, 2*M_PI*3, 2*M_PI*3, 2*M_PI*3, 2*M_PI*3, 2*M_PI*3;
	//initialize cartesian admittance controller
	_cart_admittance_data.pos_err.setZero(6);
	_cart_admittance_data.vel_err.setZero(6);
	_cart_admittance_data.acc_err.setZero(6);
	_cart_admittance_data.initialized = false;
	_cart_admittance_data.jpos_err.setZero(_njoint);
	_cart_admittance_params.epsilon.setZero(6);
	_cart_admittance_params.invk.setZero(6);
	_cart_admittance_params.vmax.setZero(6);
	_cart_admittance_params.wn.setZero(6);
	_cart_admittance_params.invk<<1/200.0, 1/200.0, 1/200.0, 1/50.0, 1/50.0, 1/50.0;
	_cart_admittance_params.epsilon<<1.0, 1.0, 1.0, 1.0, 1.0, 1.0;
	_cart_admittance_params.vmax<<0.3, 0.3, 0.3, 0.6, 0.6, 0.6;
	_cart_admittance_params.wn<<2*M_PI*3, 2*M_PI*3, 2*M_PI*3, 2*M_PI*3, 2*M_PI*3, 2*M_PI*3;
}

void RobotDynamics::GenerateGravityTorque(VectorXd& tau_grav, const VectorXd& jpos)
{
	tau_grav.setZero(_njoint);
	MatrixXd reg_mat = CalcRegGravMat(jpos);
	tau_grav.segment(1, 4) = reg_mat*_barycenter_params;
}

void RobotDynamics::GenerateFrictionTorque(VectorXd& tau_fric, const VectorXd& jvel)
{
	tau_fric.setZero(_njoint);
	for (int idx = 0; idx<_njoint; idx++)
	{
		tau_fric(idx) = _fric_params(2*idx)*SignVel(jvel(idx))
			+_fric_params(2*idx+1)*jvel(idx);
	}
}

void RobotDynamics::GenerateIdenTorque(VectorXd& tau_iden,
	const VectorXd& jpos, const VectorXd& jvel)
{
	VectorXd tau_grav, tau_fric;
	GenerateGravityTorque(tau_grav, jpos);
	GenerateFrictionTorque(tau_fric, jvel);
	tau_iden = tau_grav+tau_fric;
}

void RobotDynamics::GenerateExternalTorque(VectorXd& tau_external,
					const VectorXd& tau_iden, const VectorXd& tau_fdb)
{
	// VectorXd jtau_filter;
	// jtau_filter.resize(6);
	// for (int idx = 0; idx<_njoint; idx++)
	// {
	// 	jtau_filter(idx) = _tau_filter[idx].Process(tau_fdb(idx));
	// }
	tau_external = tau_iden-tau_fdb;
	for (int idx = 0; idx<_njoint; idx++)
	{
		if (tau_external(idx)>_tau_external_base(idx))
			tau_external(idx) = tau_external(idx)-_tau_external_base(idx);
		else if (tau_external(idx)<-_tau_external_base(idx))
			tau_external(idx) = tau_external(idx)+_tau_external_base(idx);
		else
			tau_external(idx) = 0;

		tau_external(idx) = _tau_filter[idx].Process(tau_external(idx));
	}
}


void RobotDynamics::JointAdmittanceControl(VectorXd& pos_compliance,
										const VectorXd& tau_external)
{
	for (int jidx = 0; jidx<_njoint; jidx++)
	{
		double wn = _joint_admittance_params.wn(jidx);
		double invk = _joint_admittance_params.invk(jidx);
		double epsilon = _joint_admittance_params.epsilon(jidx);
		double vmax = _joint_admittance_params.vmax(jidx);

		_joint_admittance_data.acc_err(jidx) = invk*tau_external(jidx)*wn*wn
							-wn*wn*_joint_admittance_data.pos_err(jidx)
							-2*epsilon*wn*_joint_admittance_data.vel_err(jidx);
		_joint_admittance_data.vel_err(jidx) += _joint_admittance_data.acc_err(jidx)*_cycle_time;
		MathTools::LimitNum(-vmax, _joint_admittance_data.vel_err(jidx), vmax);
		_joint_admittance_data.pos_err(jidx) += _joint_admittance_data.vel_err(jidx)*_cycle_time;
	}

	pos_compliance = _joint_admittance_data.pos_err;
}

void RobotDynamics::CartesianAdmittanceControl(VectorXd& pos_compliance,
	const VectorXd& tau_external, const VectorXd& jpos)
{
	MatrixXd jaco = _rbtdef->CalcJaco(jpos);
	VectorXd force_external = jaco.transpose().inverse()*tau_external;
	if (!_cart_admittance_data.initialized)
	{
		_cart_admittance_data.initialized = true;
		_cart_admittance_data.pose0 = _rbtdef->FKSolveTool(jpos);
	}
	for (int jidx = 0; jidx<6; jidx++)
	{
		double wn = _cart_admittance_params.wn(jidx);
		double invk = _cart_admittance_params.invk(jidx);
		double epsilon = _cart_admittance_params.epsilon(jidx);
		double vmax = _cart_admittance_params.vmax(jidx);

		_cart_admittance_data.acc_err(jidx) = invk*force_external(jidx)*wn*wn
						-wn*wn*_cart_admittance_data.pos_err(jidx)
						-2*epsilon*wn*_cart_admittance_data.vel_err(jidx);
		_cart_admittance_data.vel_err(jidx) += _cart_admittance_data.acc_err(jidx)*_cycle_time;
		MathTools::LimitNum(-vmax, _cart_admittance_data.vel_err(jidx), vmax);
		_cart_admittance_data.pos_err(jidx) += _cart_admittance_data.vel_err(jidx)*_cycle_time;
	}
	VectorXd jvel_err = jaco.inverse()*_cart_admittance_data.vel_err;
	_cart_admittance_data.jpos_err += jvel_err*_cycle_time;
	//RobotTools::Pose pose_fdb = _rbtdef->FKSolveTool(jpos);
	//_cart_admittance_data.pos_err.head(3) = pose_fdb.pos-_cart_admittance_data.pose0.pos;
	//Eigen::AngleAxis<double> angle_axis;
	//angle_axis.fromRotationMatrix(pose_fdb.rot*_cart_admittance_data.pose0.rot.transpose());
	//_cart_admittance_data.pos_err.tail(3) = angle_axis.angle()*angle_axis.axis();

	pos_compliance = _cart_admittance_data.jpos_err;
}

void RobotDynamics::ResetAdmittanceControl()
{
	_joint_admittance_data.pos_err.setZero(_njoint);
	_joint_admittance_data.vel_err.setZero(_njoint);
	_joint_admittance_data.acc_err.setZero(_njoint);

	_cart_admittance_data.pos_err.setZero(6);
	_cart_admittance_data.vel_err.setZero(6);
	_cart_admittance_data.acc_err.setZero(6);
	_cart_admittance_data.initialized = false;
	_cart_admittance_data.jpos_err.setZero(_njoint);
}

MatrixXd RobotDynamics::CalcRegGravMat(const VectorXd& jpos)
{
	MatrixXd reg_mat;
	reg_mat.setZero(4, _n_barycenter);
	reg_mat.row(0) = CalcRegressorJoint2(jpos);
	reg_mat.row(1) = CalcRegressorJoint3(jpos);
	reg_mat.row(2) = CalcRegressorJoint4(jpos);
	reg_mat.row(3) = CalcRegressorJoint5(jpos);

	return reg_mat;
}

RowVectorXd RobotDynamics::CalcRegressorJoint2(const VectorXd& jpos)
{
	RowVectorXd reg_vec;
	reg_vec.setZero(_n_barycenter);
	double q2 = jpos(1);
	double q3 = jpos(2);
	double q4 = jpos(3);
	double q5 = jpos(4);
	double q6 = jpos(5);
	reg_vec(0) = -_g*sin(q2);
	reg_vec(1) = -_g*cos(q2);
	reg_vec(2) = -_g*cos(q2+q3);
	reg_vec(3) = _g*sin(q2+q3);
	reg_vec(4) = _g*_rbtdef->_mdh_table(2,2)*sin(q3)*cos(q2+q3)
				-0.01*_g*_rbtdef->_mdh_table(2,2)*sin(q2+q3)*cos(q3);
	reg_vec(5) = -_g*sin(q2+q3+q4);
	reg_vec(6) = -_g*cos(q2+q3+q4);
	reg_vec(7) = _g*_rbtdef->_mdh_table(2,2)*sin(q3)*cos(q2+q3)
				-_g*_rbtdef->_mdh_table(3,2)*sin(q4)*sin(q2+q3+q4)
				-0.01*_g*_rbtdef->_mdh_table(2,2)*sin(q2+q3)*cos(q3)
				-_g*_rbtdef->_mdh_table(3,2)*cos(q4)*cos(q2+q3+q4);
	reg_vec(8) = -_g*sin(q2+q3+q4)*cos(q5);
	reg_vec(9) = _g*sin(q5)*sin(q2+q3+q4);
	reg_vec(10) = -_g*pow(sin(q5),2)*cos(q2+q3+q4)-_g*pow(cos(q5),2)*cos(q2+q3+q4);
	reg_vec(11) = _g*_rbtdef->_mdh_table(2,2)*sin(q3)*cos(q2+q3)
				-_g*_rbtdef->_mdh_table(3,2)*sin(q4)*sin(q2+q3+q4)
				-_g*_rbtdef->_mdh_table(4,1)*pow(sin(q5),2)*cos(q2+q3+q4)
				-0.01*_g*_rbtdef->_mdh_table(2,2)*sin(q2+q3)*cos(q3)
				-_g*_rbtdef->_mdh_table(3,2)*cos(q4)*cos(q2+q3+q4)
				-_g*_rbtdef->_mdh_table(4,1)*pow(cos(q5),2)*cos(q2+q3+q4);
	reg_vec(12) = -_g*pow(sin(q5),2)*sin(q6)*cos(q2+q3+q4)
				-_g*sin(q6)*pow(cos(q5),2)*cos(q2+q3+q4)
				-_g*sin(q2+q3+q4)*cos(q5)*cos(q6);
	reg_vec(13) = -_g*pow(sin(q5),2)*cos(q6)*cos(q2+q3+q4)
				+_g*sin(q6)*sin(q2+q3+q4)*cos(q5)
				-_g*pow(cos(q5),2)*cos(q6)*cos(q2+q3+q4);
	reg_vec(14) = -_g*sin(q5)*sin(q2+q3+q4);
	reg_vec(15) = _g*_rbtdef->_mdh_table(2,2)*sin(q3)*cos(q2+q3)
				-_g*_rbtdef->_mdh_table(3,2)*sin(q4)*sin(q2+q3+q4)
				-_g*_rbtdef->_mdh_table(4,1)*pow(sin(q5),2)*cos(q2+q3+q4)
				-_g*_rbtdef->_mdh_table(5,1)*sin(q5)*sin(q2+q3+q4)
				-0.01*_g*_rbtdef->_mdh_table(2,2)*sin(q2+q3)*cos(q3)
				-_g*_rbtdef->_mdh_table(3,2)*cos(q4)*cos(q2+q3+q4)
				-_g*_rbtdef->_mdh_table(4,1)*pow(cos(q5),2)*cos(q2+q3+q4);

	return reg_vec;
}

RowVectorXd RobotDynamics::CalcRegressorJoint3(const VectorXd& jpos)
{
	RowVectorXd reg_vec;
	reg_vec.setZero(_n_barycenter);
	double q2 = jpos(1);
	double q3 = jpos(2);
	double q4 = jpos(3);
	double q5 = jpos(4);
	double q6 = jpos(5);
	reg_vec(2) = -_g*cos(q2+q3);
	reg_vec(3) = _g*sin(q2+q3);
	reg_vec(5) = -_g*sin(q2+q3+q4);
	reg_vec(6) = -_g*cos(q2+q3+q4);
	reg_vec(7) = -_g*_rbtdef->_mdh_table(3,2)*cos(q2+q3);
	reg_vec(8) = -_g*sin(q2+q3+q4)*cos(q5);
	reg_vec(9) = _g*sin(q5)*sin(q2+q3+q4);
	reg_vec(10) = -_g*cos(q2+q3+q4);
	reg_vec(11) = -_g*_rbtdef->_mdh_table(3,2)*cos(q2+q3)
				-_g*_rbtdef->_mdh_table(4,1)*cos(q2+q3+q4);
	reg_vec(12) = -_g*sin(q6)*cos(q2+q3+q4)-_g*sin(q2+q3+q4)*cos(q5)*cos(q6);
	reg_vec(13) = _g*sin(q6)*sin(q2+q3+q4)*cos(q5)-_g*cos(q6)*cos(q2+q3+q4);
	reg_vec(14) = -_g*sin(q5)*sin(q2+q3+q4);
	reg_vec(15) = -_g*_rbtdef->_mdh_table(5,1)*sin(q5)*sin(q2+q3+q4)
				-_g*_rbtdef->_mdh_table(3,2)*cos(q2+q3)
				-_g*_rbtdef->_mdh_table(4,1)*cos(q2+q3+q4);

	return reg_vec;
}

RowVectorXd RobotDynamics::CalcRegressorJoint4(const VectorXd& jpos)
{
	RowVectorXd reg_vec;
	reg_vec.setZero(_n_barycenter);
	double q2 = jpos(1);
	double q3 = jpos(2);
	double q4 = jpos(3);
	double q5 = jpos(4);
	double q6 = jpos(5);
	reg_vec(5) = -_g*sin(q2+q3+q4);
	reg_vec(6) = -_g*cos(q2+q3+q4);
	reg_vec(8) = -_g*sin(q2+q3+q4)*cos(q5);
	reg_vec(9) = _g*sin(q5)*sin(q2+q3+q4);
	reg_vec(10) = -_g*cos(q2+q3+q4);
	reg_vec(11) = -_g*_rbtdef->_mdh_table(4,1)*cos(q2+q3+q4);
	reg_vec(12) = -_g*sin(q6)*cos(q2+q3+q4)-_g*sin(q2+q3+q4)*cos(q5)*cos(q6);
	reg_vec(13) = _g*sin(q6)*sin(q2+q3+q4)*cos(q5)-_g*cos(q6)*cos(q2+q3+q4);
	reg_vec(14) = -_g*sin(q5)*sin(q2+q3+q4);
	reg_vec(15) = -_g*_rbtdef->_mdh_table(5,1)*sin(q5)*sin(q2+q3+q4)
				-_g*_rbtdef->_mdh_table(4,1)*cos(q2+q3+q4);

	return reg_vec;
}


RowVectorXd RobotDynamics::CalcRegressorJoint5(const VectorXd& jpos)
{
	RowVectorXd reg_vec;
	reg_vec.setZero(_n_barycenter);
	double q2 = jpos(1);
	double q3 = jpos(2);
	double q4 = jpos(3);
	double q5 = jpos(4);
	double q6 = jpos(5);
	reg_vec(8) = -_g*sin(q5)*cos(q2+q3+q4);
	reg_vec(9) = -_g*cos(q5)*cos(q2+q3+q4);
	reg_vec(12) = -_g*sin(q5)*cos(q6)*cos(q2+q3+q4);
	reg_vec(13) = _g*sin(q5)*sin(q6)*cos(q2+q3+q4);
	reg_vec(14) = _g*cos(q5)*cos(q2+q3+q4);
	reg_vec(15) = _g*_rbtdef->_mdh_table(5,1)*cos(q5)*cos(q2+q3+q4);

	return reg_vec;
}

int RobotDynamics::SignVel(double vel)
{
	double thre_val = 0.02;
	int dir;
	if (fabs(vel)<=thre_val)
		dir = 0;
	else
		dir = MathTools::Sign(vel);

	return dir;
}
