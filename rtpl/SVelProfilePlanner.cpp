#include "SVelProfilePlanner.h"


SVelProfilePlanner::SVelProfilePlanner(Vector2d pos, double max_vel, double max_acc, double max_jerk)
{
	InitPlanner(pos, max_vel, max_acc, max_jerk);
}

SVelProfilePlanner::SVelProfilePlanner(Vector2d pos, double max_vel, double max_acc, double max_jerk, double tf)
{
	InitPlanner(pos, max_vel, max_acc, max_jerk, tf);
}

SVelProfilePlanner::SVelProfilePlanner(Vector2d pos, double max_vel, double max_acc, double max_jerk,Vector2d duration)
{
	InitPlanner(pos, max_vel, max_acc, max_jerk, duration);
}

SVelProfilePlanner::SVelProfilePlanner(Vector2d pos, double max_vel, double max_acc, double max_jerk, Vector2d duration, Vector2d vel_con)
{
	InitPlanner(pos, max_vel, max_acc, max_jerk, duration, vel_con);
}

void SVelProfilePlanner::InitPlanner(Vector2d pos, double max_vel, double max_acc, double max_jerk)
{
	_dir = MathTools::Sign(pos(1) - pos(0));
	double h = fabs(pos(1) - pos(0));
	Vector2d vel_con(0, 0);
	TransformPVAJ(pos, vel_con, max_vel, max_acc, max_jerk);
	CalcSVelWithNullVel(h);
}

void SVelProfilePlanner::InitPlanner(Vector2d pos, double max_vel, double max_acc, double max_jerk, double tf)
{
	_dir = MathTools::Sign(pos(1) - pos(0));
	double h = fabs(pos(1) - pos(0));
	Vector2d vel_con(0, 0);
	TransformPVAJ(pos, vel_con, max_vel, max_acc, max_jerk);
	CalcSVelWithNullVel(h);
	UpdateSVelProfilePara(tf);
}

void SVelProfilePlanner::InitPlanner(Vector2d pos, double max_vel, double max_acc, double max_jerk, Vector2d duration)
{
	_dir = MathTools::Sign(pos(1) - pos(0));
	double h = fabs(pos(1) - pos(0));
	Vector2d vel_con(0, 0);
	TransformPVAJ(pos, vel_con, max_vel, max_acc, max_jerk);
	CalcSVelWithNullVel(h);
	if (duration(1) > 0)
		UpdateSVelProfilePara(duration(1));
}

void SVelProfilePlanner::InitPlanner(Vector2d pos, double max_vel, double max_acc, double max_jerk, Vector2d duration, Vector2d vel_con)
{
	_dir = MathTools::Sign(pos(1) - pos(0));
	double h = fabs(pos(1) - pos(0));
	TransformPVAJ(pos, vel_con, max_vel, max_acc, max_jerk);
	CalcSVelWithNullVel(h);
	if (duration(1) > 0)
		UpdateSVelProfilePara(duration(1));
}

void SVelProfilePlanner::TransformPVAJ(Vector2d pos, Vector2d vel, double max_vel, double max_acc, double max_jerk)
{
	_pos = pos*_dir;
	_v0 = vel(0)*_dir;
	_vf = vel(1)*_dir;
	_max_vel = ((_dir + 1) / 2)*max_vel - ((_dir - 1) / 2)*max_vel;
	_max_acc = ((_dir + 1) / 2)*max_acc - ((_dir - 1) / 2)*max_acc;
	_max_jerk = ((_dir + 1) / 2)*max_jerk - ((_dir - 1) / 2)*max_jerk;
}

void SVelProfilePlanner::CalcSVelProfilePara(double h)
{
	//max_vel can be or not be reached
	if ((_max_vel - _v0)*_max_jerk < pow(_max_acc, 2))
	{
		_t_j1 = sqrt((_max_vel - _v0) / _max_jerk);//max_acc can not be reached
		_t_acc = 2 * _t_j1;
		_lim_acc = _max_jerk*_t_j1;
		_maxacc_reached = false;
	}
	else
	{
		_t_j1 = _max_acc / _max_jerk;
		_t_acc = _t_j1 + (_max_vel - _v0) / _max_acc;
		_lim_acc = _max_acc;
		_maxacc_reached = true;
	}

	if ((_max_vel - _vf)*_max_jerk < pow(_max_acc, 2))
	{
		_t_j2 = sqrt((_max_vel - _vf) / _max_jerk);//max_acc can not be reached
		_t_dec = 2 * _t_j2;
		_lim_dec = -_max_jerk*_t_j2;
		_maxdec_reached = false;
	}
	else
	{
		_t_j2 = _max_acc / _max_jerk;
		_t_dec = _t_j2 + (_max_vel - _vf) / _max_acc;
		_lim_dec = -_max_acc;
		_maxdec_reached = true;
	}
	//calc constant_vel_segment time
	_t_vel = h / _max_vel - (1 + _v0 / _max_vel)*_t_acc / 2.0 - (1 + _vf / _max_vel)*_t_dec / 2.0;

	if (_t_vel > 0)//max_vel can be reached,constant_vel_segment
	{
		_vlim = _max_vel;
		_tf = _t_acc + _t_vel + _t_dec;
		_maxvel_reached = true;
	}
	else//max_vel can not be reached, _t_vel = 0;
	{
		_maxvel_reached = false;
		_t_vel = 0;
		double t_j = _max_acc / _max_jerk;//assume maxacc maxdec reached
		_t_j1 = t_j;	_t_j2 = t_j;
		double delta = (pow(_max_acc, 4) / pow(_max_jerk, 2)) + 2 * (pow(_v0, 2) + pow(_vf, 2)) + _max_acc*(4 * h - 2 * (_max_acc / _max_jerk)*(_v0 + _vf));
		_t_acc = (pow(_max_acc, 2) / _max_jerk - 2.0 * _v0 + sqrt(delta)) / (2 * _max_acc);
		_t_dec = (pow(_max_acc, 2) / _max_jerk - 2.0 * _vf + sqrt(delta)) / (2 * _max_acc);
		// _t_acc _t_dec
		double temp, new_vf,new_h;
		if ((_t_acc < 0) || (_t_dec < 0))
		{
			if (_t_acc < 0)//no acc,only dec;
			{
				_maxacc_reached = false;
				_t_acc = 0; _t_j1 = 0;
				_t_dec = 2 * h / (_v0 + _vf);
				temp = _max_jerk*(_max_jerk*pow(h, 2) + pow((_v0 + _vf), 2)*(_vf - _v0));
				if (temp <= 0)
					temp = 0;
				_t_j2 = (_max_jerk*h - sqrt(temp)) / (_max_jerk*(_v0 + _vf));
				_lim_acc = 0;
				_lim_dec = -_max_jerk*_t_j2;
				_vlim = _v0;
				if (_lim_dec < -_max_acc)
					throw eErrSVelProfilePlanner;
				new_vf = _v0 + _lim_dec*(_t_dec - _t_j2);
				new_h = (_vlim + new_vf)* _t_dec / 2.0;
				if (new_h > h)
					throw eErrSVelProfilePlanner;
				_tf = _t_acc + _t_vel + _t_dec;
			}
			else if (_t_dec < 0)//no dec,only acc;
			{
				_maxdec_reached = false;
				_t_dec = 0; _t_j2 = 0;
				_t_acc = 2 * h / (_v0 + _vf);
				temp = _max_jerk*(_max_jerk*pow(h, 2) - pow((_v0 + _vf), 2)*(_vf - _v0));
				if (temp <= 0)
					temp = 0;
				_t_j1 = (_max_jerk*h - sqrt(temp)) / (_max_jerk*(_v0 + _vf));
				_lim_dec = 0;
				_lim_acc = _max_jerk*_t_j1;
				if (_lim_acc > _max_acc)
					throw eErrSVelProfilePlanner;
				_vlim = _v0 + _lim_acc*(_t_acc - _t_j1);
				new_h = (_vlim + _v0)* _t_acc / 2.0;
				if (new_h < h)
					throw eErrSVelProfilePlanner;
				_tf = _t_acc + _t_vel + _t_dec;
			}
		}
		else if ((_t_acc >= 2 * t_j) && (_t_dec >= 2 * t_j))//acc_segment,dec_segment both can be reach max_acc
		{
			_maxacc_reached = true; _maxdec_reached = true;
			_lim_acc = _max_acc;
			_lim_dec = -_max_acc;
			_vlim = _v0 + _lim_acc*(_t_acc - _t_j1);
			_tf = _t_acc + _t_vel + _t_dec;
		}
		else//dec_segment or acc_segment can not reach max_acc
		{
			double reduce_acc = 0.95;
			double temp_amax = _max_acc;
			_maxacc_reached = false; _maxdec_reached = false;
			while ((_t_acc < 2 * t_j) || (_t_dec < 2 * t_j))
			{
				temp_amax *= reduce_acc;
				_t_vel = 0; t_j = temp_amax / _max_jerk;
				_t_j1 = t_j;
				_t_j2 = t_j;
				delta = (pow(temp_amax, 4) / pow(_max_jerk, 2)) + 2 * (pow(_v0, 2) + pow(_vf, 2)) + temp_amax*(4 * h - 2 * (temp_amax / _max_jerk)*(_v0 + _vf));
				_t_acc = (pow(temp_amax, 2) / _max_jerk - 2.0 * _v0 + sqrt(delta)) / (2 * temp_amax);
				_t_dec = (pow(temp_amax, 2) / _max_jerk - 2.0 * _vf + sqrt(delta)) / (2 * temp_amax);

				if ((_t_acc < 0) || (_t_dec < 0))
				{
					if (_t_acc < 0)//no acc,only dec;
					{
						_t_acc = 0; _t_j1 = 0;
						_t_dec = 2 * h / (_v0 + _vf);
						temp = _max_jerk*(_max_jerk*pow(h, 2) + pow((_v0 + _vf), 2)*(_vf - _v0));
						if (temp <= 0)
							temp = 0;
						_t_j2 = (_max_jerk*h - sqrt(temp)) / (_max_jerk*(_v0 + _vf));
						_lim_acc = 0;
						_lim_dec = -_max_jerk*_t_j2;
						_vlim = _v0;
						if (_lim_dec < -_max_acc)
							throw eErrSVelProfilePlanner;
						new_vf = _v0 + _lim_dec*(_t_dec - _t_j2);
						new_h = (_vlim + new_vf)* _t_dec / 2.0;
						if (new_h > h)
							throw eErrSVelProfilePlanner;
						_tf = _t_acc + _t_vel + _t_dec;
						break;
					}
					else if (_t_dec < 0)//no dec,only acc;
					{
						_t_dec = 0; _t_j2 = 0;
						_t_acc = 2 * h / (_v0 + _vf);
						temp = _max_jerk*(_max_jerk*pow(h, 2) - pow((_v0 + _vf), 2)*(_vf - _v0));
						if (temp <= 0)
							temp = 0;
						_t_j1 = (_max_jerk*h - sqrt(temp)) / (_max_jerk*(_v0 + _vf));
						_lim_dec = 0;
						_lim_acc = _max_jerk*_t_j1;
						if (_lim_acc > _max_acc)
							throw eErrSVelProfilePlanner;
						_vlim = _v0 + _lim_acc*(_t_acc - _t_j1);
						new_h = (_vlim + _v0)* _t_acc / 2.0;
						if (new_h < h)
							throw eErrSVelProfilePlanner;
						_tf = _t_acc + _t_vel + _t_dec;
						break;
					}
				}
				else if ((_t_acc >= 2 * t_j) && (_t_dec >= 2 * t_j))//acc_segment,dec_segment both can be reach max_acc
				{
					_lim_acc = temp_amax;
					_lim_dec = -temp_amax;
					_vlim = _v0 + _lim_acc*(_t_acc - _t_j1);
					_tf = _t_acc + _t_vel + _t_dec;
					break;
				}
			} //end loop
		}
	} //end if tv>0
}

void SVelProfilePlanner::CalcSVelWithNullVel(double h)
{
	_v0 = 0; _vf = 0;
	//max_vel can be or not be reached
	if (_max_vel*_max_jerk < _max_acc * _max_acc)
	{
		_t_j1 = sqrt(_max_vel / _max_jerk);//max_acc can not be reached
		_t_acc = 2.0 * _t_j1;
		_maxacc_reached = false;
	}
	else
	{
		_t_j1 = _max_acc / _max_jerk;
		_t_acc = _t_j1 + _max_vel / _max_acc;
		_maxacc_reached = true;
	}
	_lim_acc = _max_jerk*_t_j1;

	//calc constant_vel_segment time
	_t_vel = h / _max_vel - _t_acc;

	if (_t_vel > 0)//max_vel can be reached,constant_vel_segment
	{
		_vlim = _max_vel;
		_maxvel_reached = true;
	}
	else//max_vel can not be reached, _t_vel = 0;
	{
		_maxvel_reached = false;
		_t_vel = 0;
		double temp = 2.0 * _max_acc * _max_acc * _max_acc / (_max_jerk * _max_jerk);
		if (h >= temp)
		{
			_t_j1 = _max_acc / _max_jerk;
			_t_acc = 0.5 * _t_j1 + sqrt(0.25 * _t_j1 * _t_j1 + h / _max_acc);
			_maxacc_reached = true;
		}
		else
		{
			_t_j1 = pow(0.5*h / _max_jerk, 1.0 / 3);
			_t_acc = 2.0 * _t_j1;
			_maxacc_reached = false;
		}
		_lim_acc = _max_jerk*_t_j1;
		_vlim = (_t_acc - _t_j1)*_lim_acc;
	} //end if tv>0
	_t_j2 = _t_j1;
	_t_dec = _t_acc;
	_lim_dec = -_lim_acc;
	_maxdec_reached = _maxacc_reached;
	_tf = _t_acc + _t_vel + _t_dec;
}

void SVelProfilePlanner::UpdateSVelProfilePara(double duration)
{
	double lama = _tf / duration;
	double lama1 = 1.0 / lama;
	_max_vel *= lama;
	_max_acc *= (lama*lama);
	_max_jerk *= (lama*lama*lama);
	_vlim *= lama;
	_lim_acc *= (lama*lama);
	_lim_dec *= (lama*lama);
	_tf *= lama1;
	_t_acc *= lama1;
	_t_dec *= lama1;
	_t_vel *= lama1;
	_t_j1 *= lama1;
	_t_j2 *= lama1;

}

RobotTools::JAVP SVelProfilePlanner::GenerateMotion(double time)
{
	RobotTools::JAVP avp; double t;
	t = MathTools::LimitMaxValue<double>(_tf, time);
	//acc
	if ((t >= 0) && (t < _t_j1))
	{
		avp.pos = _pos(0) + _v0*t + _max_jerk*(t*t*t) / 6.0;
		avp.vel = _v0 + 0.5*_max_jerk*(t*t);
		avp.acc = _max_jerk*t;
	}
	else if ((t >= _t_j1) && (t < (_t_acc - _t_j1)))
	{
		avp.pos = _pos(0) + _v0*t + (3*t*t - 3*_t_j1*t + _t_j1*_t_j1) *_lim_acc / 6.0;
		avp.vel = _v0 + _lim_acc*(t - (0.5 * _t_j1));
		avp.acc = _lim_acc;
	}
	else if ((t >= (_t_acc - _t_j1)) && (t < _t_acc))
	{
		avp.pos = _pos(0) + 0.5* (_vlim + _v0)*_t_acc - _vlim*(_t_acc - t) + _max_jerk*pow((_t_acc - t), 3) / 6.0;
		avp.vel = _vlim - 0.5* _max_jerk*(pow((_t_acc - t),2));
		avp.acc = _max_jerk*(_t_acc - t);
	}
	else if ((t >= _t_acc) && (t < (_t_acc + _t_vel)))//constant_vel
	{
		avp.pos = _pos(0) + 0.5* (_vlim + _v0)*_t_acc + _vlim*(t - _t_acc);
		avp.vel = _vlim;
		avp.acc = 0;
	}
	//dec
	else if ((t >= (_t_acc + _t_vel)) && (t < (_tf - _t_dec + _t_j2)))
	{
		avp.pos = _pos(1) - 0.5* (_vlim + _vf)*_t_dec + _vlim*(t - _tf + _t_dec) -_max_jerk*(pow((t - _tf + _t_dec),3)) / 6.0;
		avp.vel = _vlim - _max_jerk*(pow((t - _tf + _t_dec),2)*0.5);
		avp.acc = -_max_jerk*(t - _tf + _t_dec);
	}
	else if ((t >= (_tf - _t_dec + _t_j2)) && (t < (_tf - _t_j2)))
	{
		avp.pos = _pos(1) - (_vlim + _vf)*_t_dec *0.5 + _vlim*(t - _tf + _t_dec) + _lim_dec / 6.0 *(3*pow(t - _tf + _t_dec,2) - 3* _t_j2*(t - _tf + _t_dec) + _t_j2*_t_j2);
		avp.vel = _vlim + _lim_dec*(t - _tf + _t_dec - _t_j2 *0.5);
		avp.acc = _lim_dec;
	}
	else if ((t >= (_tf - _t_j2)) && (t <= _tf))
	{
		avp.pos = _pos(1) - _vf*(_tf - t) - _max_jerk*(pow((_tf - t), 3)) / 6.0;
		avp.vel = _vf + _max_jerk*(pow((t- _tf),2)*0.5);
		avp.acc = -_max_jerk*(_tf -t);
	}
	avp.pos *= _dir;
	avp.vel *= _dir;
	avp.acc *= _dir;
	return avp;
}


double SVelProfilePlanner::GetFinalTime()
{
	return _tf;
}

double SVelProfilePlanner::GetDuratoin()
{
	return _tf;
}

double SVelProfilePlanner::GetDecTime()
{
	return _t_dec;
}

double SVelProfilePlanner::GetAccTime()
{
	return _t_acc;
}