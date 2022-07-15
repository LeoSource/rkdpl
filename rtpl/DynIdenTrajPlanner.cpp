#include "DynIdenTrajPlanner.h"

void DynIdenTrajPlanner::Init(const vector<vector<double>>& params_half1hz,
						const vector<vector<double>>& params_1hz, double cycle_time)
{
	DeleteAllObject();
	_cycle_time = cycle_time;
	InitTrajParams(_traj_params_half1hz, params_half1hz, 5);
	InitTrajParams(_traj_params_1hz, params_1hz, 10);
	_w0 = 2*pi*0.1;
	_t = 0;
	_traj_idx = 0;
	_plan_completed = false;
}

void DynIdenTrajPlanner::InitTrajParams(vector<MatrixXd*>& traj_params,
							const vector<vector<double>>& params_hz,int order)
{
	int nparams = 2*order+1;
	for (int traj_idx = 0; traj_idx<(int)(params_hz.size()); traj_idx++)
	{
		MatrixXd* tmp_traj_order_params = new MatrixXd;
		tmp_traj_order_params->setZero(6, nparams);
		for (int jidx = 0; jidx<6; jidx++)
		{
			for (int pidx = 0; pidx<nparams; pidx++)
			{
				(*tmp_traj_order_params)(jidx, pidx) =
					params_hz[traj_idx][jidx*nparams+pidx];
			}
		}
		traj_params.push_back(tmp_traj_order_params);
		//Vector6d q;
		//GenerateMotion(q, 0, traj_idx);
		//_q0.push_back(q);
	}
	int order_idx = (order==5)?0:1;
	_q_b[order_idx].setOnes(2*order+1);
	_qd_b[order_idx].setZero(2*order+1);
	_qdd_b[order_idx].setZero(2*order+1);
}

void DynIdenTrajPlanner::AddTraj(const int order, const int traj_id)
{
	_traj_num = 1;
	_order = order;
	_traj_id.push_back(traj_id);
	int order_idx = (order==5) ? 0 : 1;
	_q_base = _q_b[order_idx];
	_qd_base = _qd_b[order_idx];
	_qdd_base = _qdd_b[order_idx];
	_traj_order_params = (order==5) ? _traj_params_half1hz : _traj_params_1hz;
	Vector6d q;
	GenerateMotion(q, 0, traj_id);
	_q0.push_back(q);
}

void DynIdenTrajPlanner::AddTraj(const int order,vector<int> traj_id)
{
	_traj_num = (int)(traj_id.size());
	_order = order;
	_traj_id = traj_id;
	int order_idx = (order==5) ? 0 : 1;
	_q_base = _q_b[order_idx];
	_qd_base = _qd_b[order_idx];
	_qdd_base = _qdd_b[order_idx];
	_traj_order_params = (order==5) ? _traj_params_half1hz : _traj_params_1hz;
	Vector6d q;
	for (int idx : _traj_id)
	{
		GenerateMotion(q, 0, idx);
		_q0.push_back(q);
	}
}

void DynIdenTrajPlanner::GenerateTraj(Vector6d& q_traj)
{
	GenerateMotion(q_traj, _t, _traj_id.at(_traj_idx));
	_t += _cycle_time;
	MathTools::LimitMax(10, _t);
	if (fabs(_t-10)<EPS)
	{
		if (_traj_idx==_traj_num-1)
			_plan_completed = true;
		else
		{
			_traj_idx += 1;
			_t = 0;
		}
	}
}

void DynIdenTrajPlanner::GenerateMotion(Vector6d& jpos, double t, int traj_idx)
{
	for (int jnt_idx = 0; jnt_idx<6; jnt_idx++)
	{
		RobotTools::JAVP javp = GenerateJntMotion(t, jnt_idx, traj_idx);
		jpos(jnt_idx) = javp.pos;
	}
}

RobotTools::JAVP DynIdenTrajPlanner::GenerateJntMotion(double t, int jnt_idx, int traj_idx)
{
	for (int idx = 0; idx<_order; idx++)
	{
		double tmp = _w0*(idx+1);
		double sa = sin(tmp*t);
		double ca = cos(tmp*t);
		_q_base(idx) = sa/tmp;
		_q_base(_order+idx) = -ca/tmp;
		_qd_base(idx) = ca;
		_qd_base(_order+idx) = sa;
		_qdd_base(idx) = -sa*tmp;
		_qdd_base(_order+idx) = ca*tmp;
	}
	RobotTools::JAVP javp;
	javp.pos = _traj_order_params.at(traj_idx)->row(jnt_idx)*_q_base;
	javp.vel = _traj_order_params.at(traj_idx)->row(jnt_idx)*_qd_base;
	javp.acc = _traj_order_params.at(traj_idx)->row(jnt_idx)*_qdd_base;

	return javp;
}

void DynIdenTrajPlanner::Reset()
{
	//_traj_order_params.clear();
	_t = 0;
	_traj_idx = 0;
	_plan_completed = false;
	_traj_id.clear();
	_q0.clear();
}

void DynIdenTrajPlanner::DeleteAllObject()
{
	DeleteObject(_traj_params_half1hz);
	DeleteObject(_traj_params_1hz);
	_traj_order_params.clear();
}

void DynIdenTrajPlanner::DeleteObject(vector<MatrixXd*>& params)
{
	if (params.size()!=0)
	{
		for (auto it = params.begin(); it!=params.end(); it++)
		{
			if (*it!=nullptr)
			{
				delete *it;
				*it = nullptr;
			}
		}
	}
	params.clear();
}

DynIdenTrajPlanner::~DynIdenTrajPlanner()
{
	DeleteAllObject();
}

