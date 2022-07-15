#pragma once

#define __IN
#define __OUT
#define __IN_OUT

enum ERROR_ID
{
	eNoErr = 0,
	eErrDivisionByZero = 100,
	eArmSingularity = 101,
	eWristSingularity = 102,
	eErrCalOutsideReach = 103,
	eErrLinePlanner = 104,
	eErrArcPlanner = 105,
	eErrGenerateMotionLinePlanner = 106,
	eErrGenerateMotionArcPlanner = 107,
	eErrJointOutOfRange = 108,
	eErrLspb = 109,
	eErrBSpline = 110,
	eErrSVelProfilePlanner = 111,
};
