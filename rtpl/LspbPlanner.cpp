#include "LspbPlanner.h"


LspbPlanner::LspbPlanner(Vector2d pos, double max_vel, double max_acc)
{
	InitPlanner(pos, max_vel, max_acc);
	_vel_plan = false;
}

LspbPlanner::LspbPlanner(Vector2d pos, double max_vel, double max_acc, double tf)
{
	InitPlanner(pos, max_vel, max_acc, tf);
	_vel_plan = false;
}

LspbPlanner::LspbPlanner(Vector2d pos, double max_vel, double max_acc, Vector2d duration)
{
	InitPlanner(pos, max_vel, max_acc, duration);
	_vel_plan = false;
}

LspbPlanner::LspbPlanner(Vector2d pos, double max_vel, double max_acc, Vector2d duration, Vector2d vel_con)
{
	InitPlanner(pos, max_vel, max_acc, duration, vel_con);
	_vel_plan = false;
}

LspbPlanner::LspbPlanner(double pos, double max_vel, double max_acc, Vector2d vel_cons)
{
	_dir = MathTools::Sign(vel_cons(1)-vel_cons(0));
	double vel_h = abs(vel_cons(1)-vel_cons(0));
	TransformPVA(Vector2d(pos, 0.0), vel_cons, abs(vel_cons(1)), max_acc);
	_t0 = 0;
	_ta = vel_h/max_acc;
	_tf = _ta;
	_vel_plan = true;
}

LspbPlanner::LspbPlanner(double pos, double max_vel, double max_acc, double tf, Vector2d vel_cons)
{
	_dir = MathTools::Sign(vel_cons(1)-vel_cons(0));
	double vel_h = abs(vel_cons(1)-vel_cons(0));
	TransformPVA(Vector2d(pos, 0.0), vel_cons, abs(vel_cons(1)), max_acc);
	_t0 = 0;
	if(_amax*tf<vel_h)
		throw eErrLspb;
	_ta = tf;
	_tf = tf;
	_amax = vel_h/tf;
	_vel_plan = true;
}

void LspbPlanner::InitPlanner(Vector2d pos, double max_vel, double max_acc, Vector2d duration, Vector2d vel_con)
{
	_np = (int)pos.size();
	_dir = MathTools::Sign(pos(1)-pos(0));
	double h = fabs(pos(1)-pos(0));
	if (h>pow(max_vel, 2)/max_acc)
		_maxvel_reached = true;
	else
		_maxvel_reached = false;
	TransformPVA(pos, vel_con, max_vel, max_acc);
    SetVelConstraint(h,duration(1));
}

void LspbPlanner::InitPlanner(Vector2d pos, double max_vel, double max_acc, double tf)
{
	_np = (int)pos.size();
	_dir = MathTools::Sign(pos(1)-pos(0));
	double h = fabs(pos(1)-pos(0));
	if (h>pow(max_vel, 2)/max_acc)
		_maxvel_reached = true;
	else
		_maxvel_reached = false;
	Vector2d vel_con(0, 0);
	TransformPVA(pos, vel_con, max_vel, max_acc);
	Vector2d duration(0, tf);
	SetTimeLimit(h, duration);
}

void LspbPlanner::InitPlanner(Vector2d pos, double max_vel, double max_acc, Vector2d duration)
{
	_np = (int)pos.size();
	_dir = MathTools::Sign(pos(1)-pos(0));
	double h = fabs(pos(1)-pos(0));
	if (h>pow(max_vel, 2)/max_acc)
		_maxvel_reached = true;
	else
		_maxvel_reached = false;
	Vector2d vel_con(0, 0);
	TransformPVA(pos, vel_con, max_vel, max_acc);
	SetTimeLimit(h, duration);
}

void LspbPlanner::InitPlanner(Vector2d pos, double max_vel, double max_acc)
{
	_np = (int)pos.size();
	_dir = MathTools::Sign(pos(1)-pos(0));
	double h = fabs(pos(1)-pos(0));
	if (h>pow(max_vel, 2)/max_acc)
		_maxvel_reached = true;
	else
		_maxvel_reached = false;
	Vector2d vel_con(0, 0);
	TransformPVA(pos, vel_con, max_vel, max_acc);
	SetNoTimeLimit(h);
}

void LspbPlanner::TransformPVA(Vector2d pos, Vector2d vel, double max_vel, double max_acc)
{
	_q0 = pos(0)*_dir;
	_qf = pos(1)*_dir;
	_v0 = vel(0)*_dir;
	_vf = vel(1)*_dir;
	_vmax = 0.5*(_dir+1)*max_vel-0.5*(_dir-1)*max_vel;
	_amax = 0.5*(_dir+1)*max_acc-0.5*(_dir-1)*max_acc;
}

void LspbPlanner::SetNoTimeLimit(double h)
{
	_t0 = 0;
	if (_maxvel_reached)
	{
		_ta = _vmax/_amax;
		_tf = h/_vmax+_vmax/_amax;
	}
	else
	{
		_ta = sqrt(h/_amax);
		_tf = 2*_ta;
		_vmax = h/_ta;
	}
	_td = _ta;
}

void LspbPlanner::SetTimeLimit(double h, Vector2d duration)
{
	_t0 = duration(0);
	_tf = duration(1);
	double tlen = _tf-_t0;
	if (_maxvel_reached)
	{
		//assert(tlen>=h/_vmax+_vmax/_amax);
		if(tlen<h/_vmax+_vmax/_amax)
			throw eErrLspb;

		double a = 1;
		double b = -tlen*_amax;
		double c = h*_amax;
		_vmax = (-b-sqrt(pow(b, 2)-4*a*c))/2/a;
		_ta = _vmax/_amax;
	}
	else
	{
		//assert(tlen>2*sqrt(h/_amax));
		if(tlen<2*sqrt(h/_amax))
			throw eErrLspb;

		_vmax = 2*h/tlen;
		_ta = 0.5*tlen;
		_amax = _vmax/_ta;
	}
	_td = _ta;
}

void LspbPlanner::SetVelConstraint(double h, double tf)
{
	_t0 = 0;
    if(h*_amax<0.5*fabs(pow(_v0, 2)-pow(_vf, 2)))
    {
        throw eErrLspb;
    }

	if (tf<0)
	{
		if (h*_amax>pow(_vmax, 2)-0.5*(pow(_v0, 2)+pow(_vf, 2)))
		{
			_maxvel_reached = true;
			_ta = (_vmax-_v0)/_amax;
			_td = (_vmax-_vf)/_amax;
			_tf = h/_vmax+0.5*_vmax/_amax*pow(1-_v0/_vmax, 2)+0.5*_vmax/_amax*pow(1-_vf/_vmax, 2);
		}
		else
		{
			_maxvel_reached = false;
			_vmax = sqrt(h*_amax+0.5*(pow(_v0, 2)+pow(_vf, 2)));
			_ta = (_vmax-_v0)/_amax;
			_td = (_vmax-_vf)/_amax;
			_tf = _ta+_td;
		}
	}
	else
	{
		_tf = tf;
		double tmp_v = pow(_amax, 2)*pow(_tf, 2)-4*_amax*h+2*_amax*(_v0+_vf)*_tf-pow(_v0-_vf, 2);
        if(tmp_v<=0)
            throw eErrLspb;

		double vv = 0.5*(_v0+_vf+_amax*_tf-sqrt(tmp_v));
		_ta = (vv-_v0)/_amax;
		_td = (vv-_vf)/_amax;
		double tmp_a = 4*pow(h, 2)-4*h*(_v0+_vf)*_tf+2*(pow(_v0, 2)+pow(_vf, 2))*pow(_tf, 2);
		double alim = (2*h-_tf*(_v0+_vf)+sqrt(tmp_a))/pow(_tf, 2);
		if (fabs(_amax-alim)<EPS)
			_maxvel_reached = false;
		else
			_maxvel_reached = true;
		_vmax = vv;
	}

}


RobotTools::JAVP LspbPlanner::GenerateMotion(double time)
{
	double t;
	RobotTools::JAVP avp;
	if (_vel_plan)
	{
		t = time;
		if ((t>=_t0) && (t<_t0+_ta))
		{
			avp.acc = _amax;
			avp.vel = _v0+_amax*(t-_t0);
			avp.pos = _q0+0.5*(t-_t0)*(_v0+avp.vel);
		}
		else
		{
			avp.acc = 0;
			avp.vel = _vmax;
			avp.pos = _q0+0.5*_ta*(_v0+_vmax)+_vmax*(t-_t0-_ta);
		}
	}
	else
	{
		t = MathTools::LimitMaxValue<double>(_tf, time);
		if (_maxvel_reached)
		{
			if ((t>=_t0) && (t<_t0+_ta))
			{
				avp.pos = _q0+_v0*(t-_t0)+0.5*(_vmax-_v0)/_ta*pow(t-_t0, 2);
				avp.vel = _v0+(_vmax-_v0)/_ta*(t-_t0);
				avp.acc = _amax;
			}
			else if ((t>=_t0+_ta) && (t<(_tf-_td)))
			{
				avp.pos = _q0+0.5*_v0*_ta+_vmax*(t-_t0-0.5*_ta);
				avp.vel = _vmax;
				avp.acc = 0;
			}
			else if ((t>=(_tf-_td)) && (t<=_tf))
			{
				avp.pos = _qf-_vf*(_tf-t)-0.5*(_vmax-_vf)/_td*pow(_tf-t, 2);
				avp.vel = _vf+(_vmax-_vf)/_td*(_tf-t);
				avp.acc = -_amax;
			}
		}
		else
		{
			if (fabs(_tf-_t0)<EPS)
			{
				avp.pos = _q0;
				avp.vel = 0;
				avp.acc = 0;
			}
			else
			{
				if ((t>=_t0)&&(t<=_t0+_ta))
				{
					avp.pos = _q0+_v0*(t-_t0)+0.5*(_vmax-_v0)/_ta*pow(t-_t0, 2);
					avp.vel = _v0+(_vmax-_v0)/_ta*(t-_t0);
					avp.acc = _amax;
				}
				else if ((t>_tf-_td)&&(t<=_tf))
				{
					avp.pos = _qf-_vf*(_tf-t)-0.5*(_vmax-_vf)/_td*pow(_tf-t, 2);
					avp.vel = _vf+(_vmax-_vf)/_td*(_tf-t);
					avp.acc = -_amax;
				}
			}
		}
	}

	avp.pos *= _dir;
	avp.vel *= _dir;
	avp.acc *= _dir;

	return avp;
}

double LspbPlanner::GetFinalTime()
{
	return _tf;
}

double LspbPlanner::GetDuratoin()
{
	return (_tf-_t0);
}

double LspbPlanner::GetDecTime()
{
	return _td;
}

double LspbPlanner::GetAccTime()
{
	return _ta;
}
