#include "JointPlanner.h"

JointPlanner::JointPlanner(Vector6d q0, bool sync, double cycle_time,double* qlimit,double* jjmax)
	:_sync(sync),_continuity(false),_qlimit(qlimit),_jjmax(jjmax)
{
	_jpos.push_back(q0);
	_t = 0;
	_plan_completed = false;
	_ntraj = 0;
	_traj_idx = 0;
	_cycle_time = cycle_time;
}

void JointPlanner::AddViaPos(MatrixXd* jpos, double* vmax, double* amax)
{
	_vmax = vmax;
	_amax = amax;
	if (_continuity)
		AddContiJpos(jpos);
	else
		AddDiscontiJpos(jpos);
}

void JointPlanner::AddContiJpos(MatrixXd* jpos)
{
	double cofficient = 0.3;
	_ntraj = (int)jpos->cols();
	for (int traj_idx = 0; traj_idx<_ntraj; traj_idx++)
	{
		_jpos.push_back(jpos->col(traj_idx));
		Vector6d q0, qf;
		q0 = _jpos[traj_idx];
		qf = _jpos[traj_idx + 1];
		vector<int> planidx, unplanidx;
		for (int jidx = 0; jidx<6; jidx++)
		{
			if (fabs(q0(jidx) - qf(jidx))>EPS3)
				planidx.push_back(jidx);
			else
				unplanidx.push_back(jidx);
		}
		_jplanned.push_back(planidx);
		_junplanned.push_back(unplanidx);
		vector<double> tf_jnt;
		vector<LspbPlanner*> vec_uplanner;
		vec_uplanner.resize(6);
		for (int jidx : _jplanned[traj_idx])
		{
			if ((qf(jidx) - _qlimit[2 * jidx + 1]>0) || (qf(jidx) - _qlimit[2 * jidx]<0))
				throw eErrJointOutOfRange;

			LspbPlanner* uplanner = new LspbPlanner(Vector2d(q0(jidx), qf(jidx)),
				_vmax[jidx], _amax[jidx]);
			vec_uplanner[jidx] = uplanner;
			tf_jnt.push_back(uplanner->GetFinalTime());
		}
		if (tf_jnt.size() > 0)
		{
			double temp_time = *max_element(tf_jnt.begin(), tf_jnt.end());
			temp_time = ceil(temp_time / _cycle_time)*_cycle_time;
			_tf.push_back(temp_time);
		}
		else
			_tf.push_back(0);
		if (_sync)
		{
			for (int jidx : _jplanned[traj_idx])
			{
				LspbPlanner* uplanner = new LspbPlanner(Vector2d(q0(jidx), qf(jidx)),
					_vmax[jidx], _amax[jidx], _tf[traj_idx]);
				vec_uplanner[jidx] = uplanner;
			}
		}
		_segplanner1.push_back(vec_uplanner);
		_upsegplanner1.push_back(vec_uplanner);
	}
	//check LspbPlanner dir  transition
	if (_jplanned.size() > 0)
	{
		if (_ntraj > 1)
		{
			for (int traj_idx = 0; traj_idx < _ntraj - 1; traj_idx++)
			{
				vector<bool> transition;
				vector<double> transition_time;
				vector<Vector2d> transition_pos;
				vector<RobotTools::JAVP> Javp;
				for (int jidx = 0; jidx < (int)jpos->rows(); jidx++)
				{
					if (_segplanner1[traj_idx][jidx] == NULL || _segplanner1[(traj_idx + 1)][jidx] == NULL)
					{
						transition.push_back(false);
						transition_time.push_back(0);
						Vector2d pos;
						pos << 0, 0;
						transition_pos.push_back(pos);
					}
					else
					{
						if (_segplanner1[traj_idx][jidx]->_dir == _segplanner1[(traj_idx + 1)][jidx]->_dir)
						{
							transition.push_back(true);
							double time = cofficient * min(_segplanner1[traj_idx][jidx]->GetDecTime(), _segplanner1[(traj_idx + 1)][jidx]->GetAccTime());
							time = floor(time / _cycle_time)*_cycle_time;
							transition_time.push_back(time);
							Vector2d pos;
							pos(0) = _segplanner1[traj_idx][jidx]->GenerateMotion(_tf[traj_idx] - time).pos;
							pos(1) = _segplanner1[traj_idx+1][jidx]->GenerateMotion(time).pos;
							transition_pos.push_back(pos);
						}
						else
						{
							transition.push_back(false);
							transition_time.push_back(0);
							Vector2d pos;
							pos << 0, 0;
							transition_pos.push_back(pos);
						}
					}
				}
				_transition.push_back(transition);
				_transition_time.push_back(transition_time);
				_transition_pos.push_back(transition_pos);
			}
		}
	}
	// true true update jplanner
	if (_jplanned.size() > 0)
	{
		if (_ntraj > 2)
		{
			vector<vector<int>> upjplanned;
			for (int traj_idx = 1; traj_idx < _ntraj - 1; traj_idx++)
			{
				vector<int> upplanidx;
				for (int jidx = 0; jidx < (int)jpos->rows(); jidx++)
				{
					if (_transition[traj_idx - 1][jidx] == true && _transition[traj_idx][jidx] == true)
					{
						upplanidx.push_back(jidx);
					}					
				}
				upjplanned.push_back(upplanidx);
			}
			for (int traj_idx = 1; traj_idx < _ntraj - 1; traj_idx++)
			{
				if (upjplanned[traj_idx-1].size()>0)
				{
					for (int jidx : upjplanned[traj_idx-1])
					{
						RobotTools::JAVP javp_traj1 = _segplanner1[traj_idx][jidx]->GenerateMotion(_transition_time[traj_idx - 1][jidx]);
						RobotTools::JAVP javp_traj2 = _segplanner1[traj_idx][jidx]->GenerateMotion(_tf[traj_idx] - _transition_time[traj_idx][jidx]);

						LspbPlanner* uplanner = new LspbPlanner(Vector2d(javp_traj1.pos, javp_traj2.pos),
							_vmax[jidx], _amax[jidx], Vector2d(0, _tf[traj_idx] - _transition_time[traj_idx][jidx]), Vector2d(javp_traj1.vel, javp_traj2.vel));
						_upsegplanner1[traj_idx][jidx] = uplanner;
					}
				}

			}
		}//if (_ntraj > 2)
	}
}

void JointPlanner::AddDiscontiJpos(MatrixXd* jpos)
{
	_ntraj = (int)jpos->cols();
	for (int traj_idx = 0; traj_idx<_ntraj; traj_idx++)
	{
		_jpos.push_back(jpos->col(traj_idx));
		Vector6d q0, qf;
		q0 = _jpos[traj_idx];
		qf = _jpos[traj_idx+1];
		vector<int> planidx, unplanidx;
		for (int jidx = 0; jidx<6; jidx++)
		{
			if (fabs(q0(jidx)-qf(jidx))>EPS3)
				planidx.push_back(jidx);
			else
				unplanidx.push_back(jidx);
		}
		_jplanned.push_back(planidx);
		_junplanned.push_back(unplanidx);
		vector<double> tf_jnt;
		vector<SVelProfilePlanner*> vec_uplanner;
		vec_uplanner.resize(6);
		for (int jidx : _jplanned[traj_idx])
		{
			if ((qf(jidx)-_qlimit[2*jidx+1]>0)||(qf(jidx)-_qlimit[2*jidx]<0))
				throw eErrJointOutOfRange;

			SVelProfilePlanner* uplanner = new SVelProfilePlanner(Vector2d(q0(jidx), qf(jidx)),
				_vmax[jidx], _amax[jidx], _jjmax[jidx]);
			vec_uplanner[jidx] = uplanner;
			tf_jnt.push_back(uplanner->GetFinalTime());
		}
		if (tf_jnt.size()>0)
			_tf.push_back(*max_element(tf_jnt.begin(), tf_jnt.end()));
		else
			_tf.push_back(0);
		if (_sync)
		{
			for (int jidx : _jplanned[traj_idx])
			{
				SVelProfilePlanner* uplanner = new SVelProfilePlanner(Vector2d(q0(jidx), qf(jidx)),
					_vmax[jidx], _amax[jidx], _jjmax[jidx], _tf[traj_idx]);
				vec_uplanner[jidx] = uplanner;
			}
		}
		_segplanner.push_back(vec_uplanner);
	}
}

void JointPlanner::GeneratePath(Vector6d& jpos)
{

	if (_continuity)
		GenerateContiMotion(jpos);
	else
		GenerateDiscontiMotion(jpos);
}

void JointPlanner::GenerateContiMotion(Vector6d& jpos)
{
	RobotTools::JAVP javp;
	if ((_traj_idx == _ntraj - 1) && (fabs(_t - _tf[_traj_idx])<EPS3))
		_plan_completed = true;
	else
	{
		//generate joint position
		if (_junplanned[_traj_idx].size() > 0)
		{
			for (int jidx : _junplanned[_traj_idx])
				jpos(jidx) = _jpos[_traj_idx](jidx);
		}
		if (_jplanned[_traj_idx].size() > 0)
		{
			if (_traj_idx == 0)
			{
				for (int jidx : _jplanned[_traj_idx])
				{
					if (_transition[_traj_idx][jidx] == true)
					{
						if (_t < (_tf[_traj_idx] - _transition_time[_traj_idx][jidx]+EPS))
						{
							javp = _segplanner1[_traj_idx][jidx]->GenerateMotion(_t);
							_update_jpos(jidx) = javp.pos;
						}
						else
						{
							RobotTools::JAVP javp_traj1 = _segplanner1[_traj_idx][jidx]->GenerateMotion(_t);
							RobotTools::JAVP javp_traj2 = _segplanner1[_traj_idx+1][jidx]->GenerateMotion(_t-(_tf[_traj_idx] - _transition_time[_traj_idx][jidx]));
							javp.vel = javp_traj1.vel+ javp_traj2.vel;
							javp.pos = _update_jpos(jidx) + javp.vel*_cycle_time;//(_tf[_traj_idx] - _transition_time[_traj_idx][jidx])
							_update_jpos(jidx) = javp.pos;
							javp.acc = 0;//todo  need update vel
							if (_t > (_tf[_traj_idx] - EPS3))
							{
								if (fabs(javp.pos - _transition_pos[_traj_idx][jidx](1))>EPS4)
								javp.pos = _transition_pos[_traj_idx][jidx](1);
							}
						}
					}
					else
					{
						javp = _segplanner1[_traj_idx][jidx]->GenerateMotion(_t);
					}
					jpos(jidx) = javp.pos;
				}
			}
			else if (_traj_idx == _ntraj - 1)
			{
				for (int jidx : _jplanned[_traj_idx])
				{
					if (_transition[_traj_idx-1][jidx] == true)//tf decrease  but vel->0
					{
						double t_new = _t + _transition_time[_traj_idx - 1][jidx];
						javp = _segplanner1[_traj_idx][jidx]->GenerateMotion(t_new);
					}
					else
					{
						javp = _segplanner1[_traj_idx][jidx]->GenerateMotion(_t);
					}
					jpos(jidx) = javp.pos;
				}
			}
			else
			{
				for (int jidx : _jplanned[_traj_idx])
				{
					if (_transition[_traj_idx - 1][jidx] == true && _transition[_traj_idx][jidx] == false)
					{
						double t_new = _t + _transition_time[_traj_idx-1][jidx];
						javp = _segplanner1[_traj_idx][jidx]->GenerateMotion(t_new);
					}
					else if (_transition[_traj_idx - 1][jidx] == true && _transition[_traj_idx][jidx] == true)
					{
						//tf = _tf[_traj_idx] - _transition_time[_traj_idx - 1][jidx]  vel != 0

						if (_t < (_tf[_traj_idx] - _transition_time[_traj_idx][jidx] + EPS))
						{
							javp = _upsegplanner1[_traj_idx][jidx]->GenerateMotion(_t);
							_update_jpos(jidx) = javp.pos;
						}
						else
						{
							RobotTools::JAVP javp_traj1 = _segplanner1[_traj_idx][jidx]->GenerateMotion(_t);
							RobotTools::JAVP javp_traj2 = _segplanner1[_traj_idx + 1][jidx]->GenerateMotion(_t - (_tf[_traj_idx] - _transition_time[_traj_idx][jidx]));
							javp.vel = javp_traj1.vel + javp_traj2.vel;
							javp.pos = _update_jpos(jidx) + javp.vel*_cycle_time;//(_tf[_traj_idx] - _transition_time[_traj_idx][jidx])
							_update_jpos(jidx) = javp.pos;
							javp.acc = 0;//todo  need update vel
							if (_t > (_tf[_traj_idx] - EPS3))
							{
								if (fabs(javp.pos - _transition_pos[_traj_idx][jidx](1))>EPS4)
								javp.pos = _transition_pos[_traj_idx][jidx](1);
							}
						}					
					}
					else if (_transition[_traj_idx - 1][jidx] == false && _transition[_traj_idx][jidx] == false)
					{
						javp = _segplanner1[_traj_idx][jidx]->GenerateMotion(_t);
					}
					else if (_transition[_traj_idx - 1][jidx] == false && _transition[_traj_idx][jidx] == true)
					{
						if (_t < (_tf[_traj_idx] - _transition_time[_traj_idx][jidx]+EPS))
						{
							javp = _segplanner1[_traj_idx][jidx]->GenerateMotion(_t);
							_update_jpos(jidx) = javp.pos;
						}
						else
						{
							RobotTools::JAVP javp_traj1 = _segplanner1[_traj_idx][jidx]->GenerateMotion(_t);
							RobotTools::JAVP javp_traj2 = _segplanner1[_traj_idx + 1][jidx]->GenerateMotion(_t - (_tf[_traj_idx] - _transition_time[_traj_idx][jidx]));
							javp.vel = javp_traj1.vel + javp_traj2.vel;
							javp.pos = _update_jpos(jidx) + javp.vel*_cycle_time;//(_tf[_traj_idx] - _transition_time[_traj_idx][jidx])
							_update_jpos(jidx) = javp.pos;
							javp.acc = 0;//todo  need update vel
							if (_t > (_tf[_traj_idx] - EPS3))
							{
								if (fabs(javp.pos - _transition_pos[_traj_idx][jidx](1))>EPS4)
								javp.pos = _transition_pos[_traj_idx][jidx](1);
							}
						}
					}
					jpos(jidx) = javp.pos;
				}//for (int jidx : _jplanned[_traj_idx])
			}//else
			if (fabs(_t - _tf[_traj_idx]) < EPS3)
			{
				_traj_idx += 1;
				_t = 0;
			}
		}//if (_jplanned[_traj_idx].size() > 0)
		_t += _cycle_time;
		MathTools::LimitMax(_tf[_traj_idx], _t);
	}
}

void JointPlanner::GenerateDiscontiMotion(Vector6d& jpos)
{
	RobotTools::JAVP javp;
	//generate joint position
	if (_jplanned[_traj_idx].size()>0)
	{
		for (int jidx : _jplanned[_traj_idx])
		{
			javp = _segplanner[_traj_idx][jidx]->GenerateMotion(_t);
			jpos(jidx) = javp.pos;
		}
		_t += _cycle_time;
		MathTools::LimitMax(_tf[_traj_idx], _t);
	}
	if (_junplanned[_traj_idx].size()>0)
	{
		for (int jidx : _junplanned[_traj_idx])
			jpos(jidx) = _jpos[_traj_idx](jidx);
	}
	//logical judgment about trajectory index and plan status
	if ((_traj_idx == _ntraj - 1) && (fabs(_t - _tf[_traj_idx])<EPS3))
		_plan_completed = true;
	else
	{
		if (fabs(_t - _tf[_traj_idx])<EPS3)
		{
			_traj_idx += 1;
			_t = 0;
		}
	}
}

void JointPlanner::GenerateMotion(Vector6d& jpos, Vector6d& jvel, Vector6d& jacc)
{
	RobotTools::JAVP javp;
	//generate joint position
	if (_jplanned[_traj_idx].size()>0)
	{
		for (int jidx : _jplanned[_traj_idx])
		{
			javp = _segplanner[_traj_idx][jidx]->GenerateMotion(_t);
			jpos(jidx) = javp.pos;
			jvel(jidx) = javp.vel;
			jacc(jidx) = javp.acc;
			_t += _cycle_time;
			MathTools::LimitMax(_tf[_traj_idx], _t);
		}
	}
	if (_junplanned[_traj_idx].size()>0)
	{
		for (int jidx : _junplanned[_traj_idx])
		{
			jpos(jidx) = _jpos[_traj_idx](jidx);
			jvel(jidx) = 0;
			jacc(jidx) = 0;
		}
	}
	//logical judgment about trajectory index and plan status
	if ((_traj_idx==_ntraj-1)&&(fabs(_t-_tf[_traj_idx])<EPS3))
		_plan_completed = true;
	else
	{
		if (fabs(_t-_tf[_traj_idx])<EPS3)
		{
			_traj_idx += 1;
			_t = 0;
		}
	}

}

void JointPlanner::Reset(Vector6d q0, bool sync)
{
	ClearTmp();
	_jpos.clear();
	_jpos.push_back(q0);
	_tf.clear();
	_jplanned.clear();
	_junplanned.clear();

	_plan_completed = false;
	_t = 0;
	_sync = sync;
	_ntraj = 0;
	_traj_idx = 0;
}

void JointPlanner::ClearTmp()
{
	for (int traj_idx = 0; traj_idx<_ntraj; traj_idx++)
	{
		for (auto it = _segplanner[traj_idx].begin(); it!=_segplanner[traj_idx].end(); it++)
		{
			if (*it!=nullptr)
			{
				delete *it;
				*it = nullptr;
			}
		}
		_segplanner[traj_idx].clear();
	}
	_segplanner.clear();
}

