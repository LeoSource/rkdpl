#include <math.h>
#include "Biquad.h"


#ifndef MC_PI
#define MC_PI 3.1415926535897932384626
#endif

Biquad::Biquad()
{
	_type = eLowpass;
	_a0 = 1.0;
	_a1 = _a2 = _b1 = _b2 = 0.0;
	_Fc = 0.50;
	_Q = 0.707;
	_peakGain = 0.0;
	_z1 = _z2 = 0.0;
}

Biquad::Biquad(BiquadType type, double Fc, double Q, double _peakGainDB)
{
	SetBiquad(type, Fc, Q, _peakGainDB);
	_z1 = _z2 = 0;
}

void Biquad::SetType(BiquadType type)
{
	_type = type;
	CalcBiquad();
}

void Biquad::SetQ(double Q)
{
	_Q = Q;
	CalcBiquad();
}

void Biquad::SetFc(double Fc)
{
	_Fc = Fc;
	CalcBiquad();
}

void Biquad::Set_peakGain(double _peakGainDB)
{
	_peakGain = _peakGainDB;
	CalcBiquad();
}

void Biquad::SetBiquad(BiquadType type, double Fc, double Q, double _peakGain)
{
	_type = type;
	_Q = Q;
	_Fc = Fc;
	Set_peakGain(_peakGain);
}

double Biquad::Process(double in)
{
	double out = in * _a0+_z1;
	_z1 = in * _a1+_z2-_b1 * out;
	_z2 = in * _a2-_b2 * out;
	return out;
}

void Biquad::Reset()
{
	_z1 = 0.0;
	_z2 = 0.0;
}

void Biquad::CalcBiquad()
{
	double norm;
	double V = pow(10, fabs(_peakGain)/20.0);
	double K = tan(MC_PI * _Fc);
	double  sqrt2 = sqrt(2.0);
	switch (_type)
	{
	case eLowpass:
		norm = 1/(1+K/_Q+K * K);
		_a0 = K * K * norm;
		_a1 = 2*_a0;
		_a2 = _a0;
		_b1 = 2*(K * K-1) * norm;
		_b2 = (1-K/_Q+K * K) * norm;
		break;

	case eHighpass:
		norm = 1/(1+K/_Q+K * K);
		_a0 = 1*norm;
		_a1 = -2*_a0;
		_a2 = _a0;
		_b1 = 2*(K * K-1) * norm;
		_b2 = (1-K/_Q+K * K) * norm;
		break;

	case eBandpass:
		norm = 1/(1+K/_Q+K * K);
		_a0 = K/_Q * norm;
		_a1 = 0;
		_a2 = -_a0;
		_b1 = 2*(K * K-1) * norm;
		_b2 = (1-K/_Q+K * K) * norm;
		break;

	case eNotch:
		norm = 1/(1+K/_Q+K * K);
		_a0 = (1+K * K) * norm;
		_a1 = 2*(K * K-1) * norm;
		_a2 = _a0;
		_b1 = _a1;
		_b2 = (1-K/_Q+K * K) * norm;
		break;

	case ePeak:
		if (_peakGain>=0)
		{    // boost
			norm = 1/(1+1/_Q * K+K * K);
			_a0 = (1+V/_Q * K+K * K) * norm;
			_a1 = 2*(K * K-1) * norm;
			_a2 = (1-V/_Q * K+K * K) * norm;
			_b1 = _a1;
			_b2 = (1-1/_Q * K+K * K) * norm;
		}
		else
		{    // cut
			norm = 1/(1+V/_Q * K+K * K);
			_a0 = (1+1/_Q * K+K * K) * norm;
			_a1 = 2*(K * K-1) * norm;
			_a2 = (1-1/_Q * K+K * K) * norm;
			_b1 = _a1;
			_b2 = (1-V/_Q * K+K * K) * norm;
		}
		break;
	case eLowshelf:
		if (_peakGain>=0)
		{    // boost
			norm = 1/(1+sqrt2 * K+K * K);
			_a0 = (1+sqrt(2*V) * K+V * K * K) * norm;
			_a1 = 2*(V * K * K-1) * norm;
			_a2 = (1-sqrt(2*V) * K+V * K * K) * norm;
			_b1 = 2*(K * K-1) * norm;
			_b2 = (1-sqrt2 * K+K * K) * norm;
		}
		else
		{    // cut
			norm = 1/(1+sqrt(2*V) * K+V * K * K);
			_a0 = (1+sqrt2 * K+K * K) * norm;
			_a1 = 2*(K * K-1) * norm;
			_a2 = (1-sqrt2 * K+K * K) * norm;
			_b1 = 2*(V * K * K-1) * norm;
			_b2 = (1-sqrt(2*V) * K+V * K * K) * norm;
		}
		break;
	case eHighshelf:
		if (_peakGain>=0)
		{    // boost
			norm = 1/(1+sqrt2 * K+K * K);
			_a0 = (V+sqrt(2*V) * K+K * K) * norm;
			_a1 = 2*(K * K-V) * norm;
			_a2 = (V-sqrt(2*V) * K+K * K) * norm;
			_b1 = 2*(K * K-1) * norm;
			_b2 = (1-sqrt2 * K+K * K) * norm;
		}
		else
		{    // cut
			norm = 1/(V+sqrt(2*V) * K+K * K);
			_a0 = (1+sqrt2 * K+K * K) * norm;
			_a1 = 2*(K * K-1) * norm;
			_a2 = (1-sqrt2 * K+K * K) * norm;
			_b1 = 2*(K * K-V) * norm;
			_b2 = (V-sqrt(2*V) * K+K * K) * norm;
		}
		break;
	}

}

