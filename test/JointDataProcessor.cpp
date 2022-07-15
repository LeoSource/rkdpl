#define _USE_MATH_DEFINES
#include <math.h>
#include "JointDataProcessor.h"

template<int NJOINT>
JointDataProcessor<NJOINT>::JointDataProcessor()
{
}

template<int NJOINT>
JointDataProcessor<NJOINT>::~JointDataProcessor()
{
}

template<int NJOINT>
template<typename TYPEPOS>
void JointDataProcessor<NJOINT>::Initialize(const ServoParams<NJOINT>& servo_params,
	bool is_multiturn_encoder, const TYPEPOS(&encoder_pos)[NJOINT])
{
	memcpy(_offset, servo_params.offset, sizeof(_offset));
	memcpy(_resolution, servo_params.resolution, sizeof(_resolution));
	memcpy(_dir, servo_params.dir, sizeof(_dir));
	memcpy(_rated_torque, servo_params.rated_torque, sizeof(_rated_torque));
	memcpy(_rated_current, servo_params.rated_current, sizeof(_rated_current));
	memcpy(_motor_torque_coeff, servo_params.motor_torque_coeff, sizeof(_motor_torque_coeff));
	memcpy(_gear_ratio, servo_params.gear_ratio, sizeof(_gear_ratio));
	memcpy(_gear_efficiency, servo_params.gear_efficiency, sizeof(_gear_efficiency));
	if (is_multiturn_encoder)
	{
		memset(_turns_save, 0, sizeof(_turns_save));
		_first_calculation = false;
	}
	else
	{
		memcpy(_turns_save, servo_params.multi_turns, sizeof(_turns_save));
		_first_calculation = true;
	}
	for (int jidx = 0; jidx<NJOINT; jidx++)
		_pre_encoder_pos[jidx] = (long)encoder_pos[jidx];
}

template<int NJOINT>
template<typename TYPEPOS, typename TYPETAU>
void JointDataProcessor<NJOINT>::InputFromServo(const TYPEPOS(&encoder_pos)[NJOINT],
	const TYPEPOS(&encoder_vel)[NJOINT], const TYPETAU(&motor_tau)[NJOINT])
{
	CalcEncoderTurns<TYPEPOS>(encoder_pos);
	for (int jidx = 0; jidx<NJOINT; jidx++)
	{
		_turns_save[jidx] += (int)(floor((double)encoder_pos[jidx]/_resolution[jidx])
			-floor((double)_pre_encoder_pos[jidx]/_resolution[jidx]));
		_pre_encoder_pos[jidx] = encoder_pos[jidx];

		_jpos_fdb[jidx] = (encoder_pos[jidx]+_turns_cal[jidx]*_resolution[jidx]-_offset[jidx])
			/_resolution[jidx]*_dir[jidx]*2*M_PI;
		_jvel_fdb[jidx] = encoder_vel[jidx]/_resolution[jidx]*_dir[jidx]*2*M_PI;
		double motor_current = motor_tau[jidx]*_rated_current[jidx]/1000.0;
		_jtau_fdb[jidx] = motor_current*_motor_torque_coeff[jidx]*_gear_ratio[jidx]
			*_gear_efficiency[jidx]*_dir[jidx];
	}
}

template<int NJOINT>
template<typename T>
void JointDataProcessor<NJOINT>::OutputToServo(const T& jpos_cmd,
	const T& jvel_cmd, const T& jtau_cmd)
{
	for (int jidx = 0; jidx<NJOINT; jidx++)
	{
		_encoder_pos_cmd[jidx] = (long)(jpos_cmd[jidx]*_dir[jidx]*_resolution[jidx]/(2*M_PI)
			+_offset[jidx]-_turns_cal[jidx]*_resolution[jidx]);
		_encoder_vel_cmd[jidx] = (long)(jvel_cmd[jidx]*_resolution[jidx]*_dir[jidx]/(2*M_PI));
		double motor_current = jtau_cmd[jidx]*_dir[jidx]
			/(_motor_torque_coeff[jidx]*_gear_ratio[jidx]*_gear_efficiency[jidx]);
		_servo_tau_cmd[jidx] = (long)(motor_current*1000.0/_rated_current[jidx]);
	}
}

template<int NJOINT>
template<typename T>
void JointDataProcessor<NJOINT>::GetJointData(T& jpos, T& jvel, T& jtau)
{
	for (int jidx = 0; jidx<NJOINT; jidx++)
	{
		jpos[jidx] = _jpos_fdb[jidx];
		jvel[jidx] = _jvel_fdb[jidx];
		jtau[jidx] = _jtau_fdb[jidx];
	}
}

template<int NJOINT>
template<typename TYPEPOS, typename TYPETAU>
void JointDataProcessor<NJOINT>::GetServoData(TYPEPOS(&encoder_pos)[NJOINT],
	TYPEPOS(&encoder_vel)[NJOINT], TYPETAU(&servo_tau)[NJOINT])
{
	for (int jidx = 0; jidx<NJOINT; jidx++)
	{
		encoder_pos[jidx] = (TYPEPOS)_encoder_pos_cmd[jidx];
		encoder_vel[jidx] = (TYPEPOS)_encoder_vel_cmd[jidx];
		servo_tau[jidx] = (TYPETAU)_servo_tau_cmd[jidx];
	}
}

template<int NJOINT>
template<typename TYPEPOS>
void JointDataProcessor<NJOINT>::CalcEncoderTurns(const TYPEPOS(&encoder_pos)[NJOINT])
{
	if (_first_calculation)
	{
		//bool continuous_restart = false;
		for (int jidx = 0; jidx<NJOINT; jidx++)
		{
			if ((encoder_pos[jidx]>_resolution[jidx])||(encoder_pos[jidx]<0))
			{
				_turns_cal[jidx] = _turns_save[jidx]-SignNegative<int>(_turns_save[jidx]);
				//continuous_restart = true;
				//break;
			}
			else
				_turns_cal[jidx] = _turns_save[jidx];
		}
		//if (continuous_restart)
		//{
		//	for (int jidx = 0; jidx<NJOINT; jidx++)
		//	{
		//		_turns_cal[jidx] = _turns_save[jidx]-MathTools::Sign(_turns_save[jidx]);
		//	}
		//}
		//else
		//	memcpy(_turns_cal, _turns_save, sizeof(_turns_cal));
		memcpy(_turns_save, _turns_cal, sizeof(_turns_cal));
		_first_calculation = false;
	}
}

template<int NJOINT>
template<typename T>
int JointDataProcessor<NJOINT>::SignNegative(T val)
{
	if (val>0)
		return 1;
	else
		return -1;
}

template<int NJOINT>
void JointDataProcessor<NJOINT>::GetUpdatedTurns(int(&updated_turns)[NJOINT])
{
	memcpy(updated_turns, _turns_save, sizeof(_turns_save));
}

template class JointDataProcessor<6>;
template void JointDataProcessor<6>::Initialize(const ServoParams<6>& servo_params,
	bool is_multiturn_encoder, const int32_t(&encoder_pos)[6]);
template void JointDataProcessor<6>::OutputToServo(const Eigen::VectorXd& jpos_cmd,
	const Eigen::VectorXd& jvel_cmd, const Eigen::VectorXd& jtau_cmd);
template void JointDataProcessor<6>::OutputToServo(const Eigen::Matrix<double,6,1>& jpos_cmd,
	const Eigen::Matrix<double,6,1>& jvel_cmd, const Eigen::Matrix<double,6,1>& jtau_cmd);
template void JointDataProcessor<6>::GetJointData(Eigen::VectorXd& jpos,
	Eigen::VectorXd& jvel, Eigen::VectorXd& jtau);
template void JointDataProcessor<6>::GetJointData(Eigen::Matrix<double,6,1>& jpos,
	Eigen::Matrix<double,6,1>& jvel, Eigen::Matrix<double,6,1>& jtau);
template void JointDataProcessor<6>::InputFromServo(const int32_t(&encoder_pos)[6],
	const int32_t(&encoder_vel)[6], const int16_t(&motor_tau)[6]);
template void JointDataProcessor<6>::GetServoData(int32_t(&encoder_pos)[6],
	int32_t(&encoder_vel)[6], int16_t(&servo_tau)[6]);

