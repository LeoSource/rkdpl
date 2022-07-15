#pragma once

/**
* @file		Biquad.h
* @brief	biquad filter for processing data
* @version	1.0.0
* @author	zxliao
* @email	zhixiangleo@163.com
* @date		2021/12/17
**/



enum BiquadType
{
	eLowpass,
	eHighpass,
	eBandpass,
	eNotch,
	ePeak,
	eLowshelf,
	eHighshelf
};

class Biquad 
{
protected:
	BiquadType _type;
	double _a0, _a1, _a2, _b1, _b2;
	double _Fc, _Q, _peakGain;
	double _z1, _z2;

public:
	Biquad();

	Biquad(BiquadType type, double Fc, double Q, double _peakGainDB);

	~Biquad() = default;

	void SetType(BiquadType type);

	void SetQ(double Q);

	void SetFc(double Fc);

	void Set_peakGain(double _peakGainDB);

	void SetBiquad(BiquadType type, double Fc, double Q, double _peakGain);

	double Process(double in);

	void Reset();

protected:
	void CalcBiquad();

};


