#pragma once

#include <Eigen/Dense>
#include <stdint.h>

template<int NJOINT>
struct ServoParams
{
	double offset[NJOINT];
	double resolution[NJOINT];
	double dir[NJOINT];
	double rated_torque[NJOINT];
	double rated_current[NJOINT];
	double motor_torque_coeff[NJOINT];
	double gear_ratio[NJOINT];
	double gear_efficiency[NJOINT];
	int multi_turns[NJOINT];
};

template<int NJOINT>
class JointDataProcessor
{
private:
	double _offset[NJOINT], _resolution[NJOINT], _dir[NJOINT];
	double _rated_torque[NJOINT], _rated_current[NJOINT];
	double _motor_torque_coeff[NJOINT], _gear_ratio[NJOINT], _gear_efficiency[NJOINT];
	int _turns_cal[NJOINT], _turns_save[NJOINT];
	bool _first_calculation;

private:
	double _jpos_fdb[NJOINT], _jvel_fdb[NJOINT], _jtau_fdb[NJOINT];
	long _encoder_pos_cmd[NJOINT], _encoder_vel_cmd[NJOINT], _servo_tau_cmd[NJOINT];
	long _pre_encoder_pos[NJOINT];
	
public:
	JointDataProcessor();
	template<typename TYPEPOS>
	void Initialize(const ServoParams<NJOINT>& servo_params, bool is_multiturn_encoder,
		const TYPEPOS(&encoder_pos)[NJOINT]);

	template<typename TYPEPOS,typename TYPETAU>
	void InputFromServo(const TYPEPOS(&encoder_pos)[NJOINT], const TYPEPOS(&encoder_vel)[NJOINT],
					const TYPETAU(&motor_tau)[NJOINT]);

	//support for Eigen and STL
	template<typename T>
	void OutputToServo(const T& jpos_cmd, const T& jvel_cmd, const T& jtau_cmd);

	//support for Eigen and STL
	template<typename T>
	void GetJointData(T& jpos, T& jvel, T& jtau);

	template<typename TYPEPOS,typename TYPETAU>
	void GetServoData(TYPEPOS(&encoder_pos)[NJOINT], TYPEPOS(&encoder_vel)[NJOINT],
					TYPETAU(&servo_tau)[NJOINT]);

	void GetUpdatedTurns(int(&updated_turns)[NJOINT]);

	~JointDataProcessor();
private:
	template<typename TYPEPOS>
	void CalcEncoderTurns(const TYPEPOS(&encoder_pos)[NJOINT]);

	template<typename T>
	int SignNegative(T val);
};

//template class JointDataProcessor<6>;