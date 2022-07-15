/**
* @file		MirrorCleanPlanner.h
* @brief	Plan clean path for several kinds of mirror
* @version	1.0.0
* @author	zxliao
* @email	zhixiangleo@163.com
* @date		2022/5/9
**/
#pragma once

#include "RobotMath.h"
#include "PlanTools.h"
#include <map>


enum class MirrorType
{
	eRectangle,
	eCircle,
	eEllipse,
	eRunway,
	eOctagon
};

struct CleanParams
{
	std::string path_type;
	std::string camera_ori;
	double dis_trans;
	double pitch_angle[2];
	double yaw_angle[2];
	double clean_tool[2];
	double trans_angle;
};

struct CircleMirrorParams
{
	Vector3d origin;
	double radius;
};

struct EllipseMirrorParams
{
	Vector3d origin;
	double a;
	double b;
};

class MirrorCleanPlanner
{
private:
	MirrorType _type;
	MatrixXd _vertices;
	CleanParams _clean_params, _clean_params_default;
	int _hori_times;
	CircleMirrorParams _circle_params;
	EllipseMirrorParams _ellipse_params;
	Matrix3d _rot_plane;

	using PtrPlanMethod = void (MirrorCleanPlanner::*)(MatrixXd&, MatrixXd&, MatrixXd&);
	std::map<MirrorType, PtrPlanMethod> _planmethod_map;
public:
	MirrorCleanPlanner();

	void SetCleanParams(const MatrixXd& vertices, MirrorType type);

	void SetCleanParams(const MatrixXd& vertices, MirrorType type, CleanParams clean_params);

	void PlanCleanPath(MatrixXd& via_posrpy_up, MatrixXd& via_posrpy_middle, MatrixXd& via_posrpy_down);

	~MirrorCleanPlanner();
private:
	void InitPlanMethodMap();

	void PlanRectangleMirror(MatrixXd& via_posrpy_up, MatrixXd& via_posrpy_middle, MatrixXd& via_posrpy_down);

	void PlanCircleMirror(MatrixXd& via_posrpy_up, MatrixXd& via_posrpy_middle, MatrixXd& via_posrpy_down);
	void CalcCircleParams(const MatrixXd& vertices);

	void PlanEllipseMirror(MatrixXd& via_posrpy_up, MatrixXd& via_posrpy_middle, MatrixXd& via_posrpy_down);
	void CalcRotEllipse(const MatrixXd& vertices);
	void CalcEllipseParams(const MatrixXd& vertices);

	void PlanRunwayMirror(MatrixXd& via_posrpy_up, MatrixXd& via_posrpy_middle, MatrixXd& via_posrpy_down);
	void CalcRotRunway(const MatrixXd& vertices);

	void PlanOctagonMirror(MatrixXd& via_posrpy_up, MatrixXd& via_posrpy_middle, MatrixXd& via_posrpy_down);


};

