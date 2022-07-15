#include "MirrorCleanPlanner.h"
#include <iostream>

MirrorCleanPlanner::MirrorCleanPlanner()
{
	_clean_params_default.path_type = "N";
	_clean_params_default.camera_ori = "top";
	_clean_params_default.dis_trans = 0.1;
	_clean_params_default.pitch_angle[0] = 30*D2R;
	_clean_params_default.pitch_angle[1] = 60*D2R;
	_clean_params_default.yaw_angle[0] = 0*D2R;
	_clean_params_default.yaw_angle[1] = 0*D2R;
	_clean_params_default.clean_tool[0] = 0.25;
	_clean_params_default.clean_tool[1] = 0;
	_clean_params_default.trans_angle = 30*D2R;
	InitPlanMethodMap();
}

void MirrorCleanPlanner::InitPlanMethodMap()
{
	struct PlanMethodInfo
	{
		MirrorType MethodID;
		PtrPlanMethod PlanMethod;
	};
	PlanMethodInfo plan_config[] =
	{
		{ MirrorType::eRectangle,		&MirrorCleanPlanner::PlanRectangleMirror },
		{ MirrorType::eCircle,			&MirrorCleanPlanner::PlanCircleMirror },
		{ MirrorType::eEllipse,			&MirrorCleanPlanner::PlanEllipseMirror },
		{ MirrorType::eRunway,			&MirrorCleanPlanner::PlanRunwayMirror },
		{ MirrorType::eOctagon,			&MirrorCleanPlanner::PlanOctagonMirror }
	};
	int add_count = sizeof(plan_config)/sizeof(*plan_config);
	while (add_count--)
		_planmethod_map[plan_config[add_count].MethodID] = plan_config[add_count].PlanMethod;
}

void MirrorCleanPlanner::SetCleanParams(const MatrixXd& vertices, MirrorType type, CleanParams clean_params)
{
	_vertices = vertices;
	_type = type;
	_clean_params = clean_params;
	_hori_times = 1;
	switch (type)
	{
	case MirrorType::eRectangle:
		break;
	case MirrorType::eCircle:
	{
		CalcRotEllipse(vertices);
		CalcCircleParams(vertices);
	}
		break;
	case MirrorType::eEllipse:
	{
		CalcRotEllipse(vertices);
		CalcEllipseParams(vertices);
	}
		break;
	case MirrorType::eRunway:
	{
		CalcRotRunway(vertices);
	}
		break;
	case MirrorType::eOctagon:
		break;
	default:
		break;
	}
}

void MirrorCleanPlanner::SetCleanParams(const MatrixXd& vertices, MirrorType type)
{
	_vertices = vertices;
	_type = type;
	_hori_times = 1;
	switch (type)
	{
	case MirrorType::eRectangle:
	{
		_clean_params = _clean_params_default;
	}
		break;
	case MirrorType::eCircle:
	{
		_clean_params = _clean_params_default;
		CalcRotEllipse(vertices);
		CalcCircleParams(vertices);
	}
		break;
	case MirrorType::eEllipse:
	{
		_clean_params = _clean_params_default;
		CalcRotEllipse(vertices);
		CalcEllipseParams(vertices);
	}
		break;
	case MirrorType::eRunway:
	{
		_clean_params = _clean_params_default;
		CalcRotRunway(vertices);
	}
		break;
	case MirrorType::eOctagon:
	{
		_clean_params = _clean_params_default;
	}
		break;
	default:
		break;
	}
}

void MirrorCleanPlanner::PlanCleanPath(MatrixXd& via_posrpy_up, MatrixXd& via_posrpy_middle,
	MatrixXd& via_posrpy_down)
{
	(this->*_planmethod_map[_type])(via_posrpy_up, via_posrpy_middle, via_posrpy_down);
}

void MirrorCleanPlanner::PlanRectangleMirror(MatrixXd& via_posrpy_up, MatrixXd& via_posrpy_middle,
	MatrixXd& via_posrpy_down)
{
	PlanTools::QuadranglePlanner rect_planner;
	rect_planner.PlanMirror(via_posrpy_middle, &_vertices, _hori_times);
	via_posrpy_up.setZero(6, 1);
	via_posrpy_down.setZero(6, 1);
}

void MirrorCleanPlanner::PlanCircleMirror(MatrixXd& via_posrpy_up, MatrixXd& via_posrpy_middle,
	MatrixXd& via_posrpy_down)
{
	PlanTools::QuadranglePlanner rect_planner;
	// plan for middle zone
	double truncation_width = 0.55;
	double tmp_x = 0.5*truncation_width;
	double tmp_y = sqrt(_circle_params.radius*_circle_params.radius-tmp_x*tmp_x);
	MatrixXd vertices_rect(3, 4);
	Vector3d tmp_vec(tmp_x, -tmp_y, 0);
	vertices_rect.col(0) = _circle_params.origin+_rot_plane*tmp_vec;
	tmp_vec<<-tmp_x, -tmp_y, 0;
	vertices_rect.col(1) = _circle_params.origin+_rot_plane*tmp_vec;
	tmp_vec<<-tmp_x, tmp_y, 0;
	vertices_rect.col(2) = _circle_params.origin+_rot_plane*tmp_vec;
	tmp_vec<<tmp_x, tmp_y, 0;
	vertices_rect.col(3) = _circle_params.origin+_rot_plane*tmp_vec;
	MatrixXd via_posrpy_rect;
	rect_planner.UniversalPlan(via_posrpy_rect, &vertices_rect, _clean_params.clean_tool,
		_clean_params.pitch_angle, _clean_params.yaw_angle, _clean_params.dis_trans,
		_clean_params.camera_ori, _clean_params.path_type, _clean_params.trans_angle);
	Vector3d arc_up = vertices_rect.col(3)-0.5*_clean_params.clean_tool[0]*_rot_plane.col(0);
	Vector3d arc_middle = _vertices.col(0)-0.5*_clean_params.clean_tool[0]*_rot_plane.col(0);
	Vector3d arc_down = vertices_rect.col(0)-0.5*_clean_params.clean_tool[0]*_rot_plane.col(0);
	MatrixXd via_posrpy_right(6, 5);
	via_posrpy_right.block(0, 1, 3, 3)<<arc_up, arc_middle, arc_down;
	via_posrpy_right.topLeftCorner(3, 1) = arc_up+_clean_params.dis_trans*_rot_plane.col(2);
	via_posrpy_right.topRightCorner(3, 1) = arc_down+_clean_params.dis_trans*_rot_plane.col(2);
	Vector3d rpy_arc = via_posrpy_rect.block(3, 1, 3, 1);
	via_posrpy_right.bottomRows(3)<<rpy_arc, rpy_arc, rpy_arc, rpy_arc, rpy_arc;
	arc_up = vertices_rect.col(2)+0.5*_clean_params.clean_tool[0]*_rot_plane.col(0);
	arc_middle = _vertices.col(2)+0.5*_clean_params.clean_tool[0]*_rot_plane.col(0);
	arc_down = vertices_rect.col(1)+0.5*_clean_params.clean_tool[0]*_rot_plane.col(0);
	MatrixXd via_posrpy_left(6, 5);
	via_posrpy_left.block(0, 1, 3, 3)<<arc_up, arc_middle, arc_down;
	via_posrpy_left.topLeftCorner(3, 1) = arc_up+_clean_params.dis_trans*_rot_plane.col(2);
	via_posrpy_left.topRightCorner(3, 1) = arc_down+_clean_params.dis_trans*_rot_plane.col(2);
	via_posrpy_left.bottomRows(3)<<rpy_arc, rpy_arc, rpy_arc, rpy_arc, rpy_arc;
	int n_rect = (int)(via_posrpy_rect.cols());
	via_posrpy_middle.setZero(6, n_rect+10);
	via_posrpy_middle<<via_posrpy_right, via_posrpy_rect, via_posrpy_left;
	// plan for up zone
	Vector3d arc_left = vertices_rect.col(2)-0.5*_clean_params.clean_tool[0]*_rot_plane.col(1);
	arc_up = _vertices.col(3)-0.5*_clean_params.clean_tool[0]*_rot_plane.col(1);
	Vector3d arc_right = vertices_rect.col(3)-0.5*_clean_params.clean_tool[0]*_rot_plane.col(1);
	via_posrpy_up.setZero(6, 5);
	via_posrpy_up.block(0, 1, 3, 3)<<arc_left, arc_up, arc_right;
	via_posrpy_up.topLeftCorner(3, 1) = arc_left+_clean_params.dis_trans*_rot_plane.col(2);
	via_posrpy_up.topRightCorner(3, 1) = arc_right+_clean_params.dis_trans*_rot_plane.col(2);
	Matrix3d mat_tmp;
	mat_tmp<<0, -1, 0, -1, 0, 0, 0, 0, -1;
	rpy_arc = RobotTools::Tr2FixedZYX(_rot_plane*mat_tmp*RobotTools::RotX(-50*D2R));
	via_posrpy_up.bottomRows(3)<<rpy_arc, rpy_arc, rpy_arc, rpy_arc, rpy_arc;
	// plan for down zone
	arc_left = vertices_rect.col(1)+0.5*_clean_params.clean_tool[0]*_rot_plane.col(1);
	arc_down = _vertices.col(1)+0.5*_clean_params.clean_tool[0]*_rot_plane.col(1);
	arc_right = vertices_rect.col(0)+0.5*_clean_params.clean_tool[0]*_rot_plane.col(1);
	via_posrpy_down.setZero(6, 5);
	via_posrpy_down.block(0, 1, 3, 3)<<arc_left, arc_down, arc_right;
	via_posrpy_down.topLeftCorner(3, 1) = arc_left+_clean_params.dis_trans*_rot_plane.col(2);
	via_posrpy_down.topRightCorner(3, 1) = arc_right+_clean_params.dis_trans*_rot_plane.col(2);
	via_posrpy_down.bottomRows(3)<<rpy_arc, rpy_arc, rpy_arc, rpy_arc, rpy_arc;
}

void MirrorCleanPlanner::CalcCircleParams(const MatrixXd& vertices)
{
	_circle_params.origin = 0.5*(0.5*(vertices.col(0)+vertices.col(2))
		+0.5*(vertices.col(1)+vertices.col(3)));
	_circle_params.radius = 0.25*((vertices.col(0)-_circle_params.origin).norm()
		+(vertices.col(1)-_circle_params.origin).norm()
		+(vertices.col(2)-_circle_params.origin).norm()
		+(vertices.col(3)-_circle_params.origin).norm());
}

void MirrorCleanPlanner::PlanEllipseMirror(MatrixXd& via_posrpy_up, MatrixXd& via_posrpy_middle,
	MatrixXd& via_posrpy_down)
{
	PlanTools::QuadranglePlanner rect_planner;
	// plan for middle zone
	double truncation_width = 0.4;
	double tmp_x = 0.5*truncation_width;
	double tmp_y = sqrt(_ellipse_params.a*_ellipse_params.a
		*(1-tmp_x*tmp_x/_ellipse_params.b/_ellipse_params.b));
	MatrixXd vertices_rect(3, 4);
	Vector3d tmp_vec(tmp_x, -tmp_y, 0);
	vertices_rect.col(0) = _ellipse_params.origin+_rot_plane*tmp_vec;
	tmp_vec<<-tmp_x, -tmp_y, 0;
	vertices_rect.col(1) = _ellipse_params.origin+_rot_plane*tmp_vec;
	tmp_vec<<-tmp_x, tmp_y, 0;
	vertices_rect.col(2) = _ellipse_params.origin+_rot_plane*tmp_vec;
	tmp_vec<<tmp_x, tmp_y, 0;
	vertices_rect.col(3) = _ellipse_params.origin+_rot_plane*tmp_vec;
	MatrixXd via_posrpy_rect;
	rect_planner.UniversalPlan(via_posrpy_rect, &vertices_rect, _clean_params.clean_tool,
		_clean_params.pitch_angle, _clean_params.yaw_angle, _clean_params.dis_trans,
		_clean_params.camera_ori, _clean_params.path_type, _clean_params.trans_angle);
	Vector3d tmp_rpy = via_posrpy_rect.block(3, 1, 3, 1);
	Vector3d via_pos_right = _vertices.col(0)-_rot_plane.col(0)*0.5*_clean_params.clean_tool[0];
	Vector3d via_pos_left = _vertices.col(2)+_rot_plane.col(0)*0.5*_clean_params.clean_tool[0];
	MatrixXd via_posrpy_right(6, 5);
	via_posrpy_right.leftCols(2) = via_posrpy_rect.leftCols(2);
	via_posrpy_right.rightCols(2) = via_posrpy_rect.middleCols(2, 2);
	via_posrpy_right.middleCols(2, 1)<<via_pos_right, tmp_rpy;
	MatrixXd via_posrpy_left(6, 5);
	via_posrpy_left.leftCols(2) = via_posrpy_rect.middleCols(via_posrpy_rect.cols()-4, 2);
	via_posrpy_left.rightCols(2) = via_posrpy_rect.rightCols(2);
	via_posrpy_left.middleCols(2, 1)<<via_pos_left, tmp_rpy;
	via_posrpy_middle.setZero(6, via_posrpy_rect.cols()+10);
	via_posrpy_middle<<via_posrpy_right, via_posrpy_rect, via_posrpy_left;
	// plan for up zone
	double r = 0.5*((vertices_rect.col(2)-_ellipse_params.origin).norm()
		+(vertices_rect.col(3)-_ellipse_params.origin).norm());
	Vector3d arc_up = _ellipse_params.origin+_rot_plane.col(1)*r;
	Vector3d vec_tmp = _ellipse_params.origin-vertices_rect.col(2);
	Vector3d arc_left = vertices_rect.col(2)
		+0.5*_clean_params.clean_tool[0]*vec_tmp.normalized();
	vec_tmp = _ellipse_params.origin-arc_up;
	arc_up += 0.5*_clean_params.clean_tool[0]*vec_tmp.normalized();
	vec_tmp = _ellipse_params.origin-vertices_rect.col(3);
	Vector3d arc_right = vertices_rect.col(3)
		+0.5*_clean_params.clean_tool[0]*vec_tmp.normalized();
	via_posrpy_up.setZero(6, 5);
	via_posrpy_up.block(0, 1, 3, 3)<<arc_left, arc_up, arc_right;
	via_posrpy_up.topLeftCorner(3, 1) = vertices_rect.col(2)+_clean_params.dis_trans*_rot_plane.col(2);
	via_posrpy_up.topRightCorner(3, 1) = vertices_rect.col(3)+_clean_params.dis_trans*_rot_plane.col(2);
	Matrix3d mat_tmp;
	mat_tmp<<0, -1, 0, -1, 0, 0, 0, 0, -1;
	Vector3d rpy_arc = RobotTools::Tr2FixedZYX(_rot_plane*mat_tmp*RobotTools::RotX(-50*D2R));
	via_posrpy_up.bottomRows(3)<<rpy_arc, rpy_arc, rpy_arc, rpy_arc, rpy_arc;
	// plan for down zone
	r = 0.5*((vertices_rect.col(0)-_ellipse_params.origin).norm()
		+(vertices_rect.col(1)-_ellipse_params.origin).norm());
	Vector3d arc_down = _ellipse_params.origin-_rot_plane.col(1)*r;
	vec_tmp = _ellipse_params.origin-vertices_rect.col(1);
	arc_left = vertices_rect.col(1)+0.5*_clean_params.clean_tool[0]*vec_tmp.normalized();
	vec_tmp = _ellipse_params.origin-arc_down;
	arc_down += 0.5*_clean_params.clean_tool[0]*vec_tmp.normalized();
	vec_tmp = _ellipse_params.origin-vertices_rect.col(0);
	arc_right = vertices_rect.col(0)+0.5*_clean_params.clean_tool[0]*vec_tmp.normalized();
	via_posrpy_down.setZero(6, 5);
	via_posrpy_down.block(0, 1, 3, 3)<<arc_left, arc_down, arc_right;
	via_posrpy_down.topLeftCorner(3, 1) = vertices_rect.col(1)+_clean_params.dis_trans*_rot_plane.col(2);
	via_posrpy_down.topRightCorner(3, 1) = vertices_rect.col(0)+_clean_params.dis_trans*_rot_plane.col(2);
	mat_tmp<<0, -1, 0, -1, 0, 0, 0, 0, -1;
	rpy_arc = RobotTools::Tr2FixedZYX(_rot_plane*mat_tmp*RobotTools::RotX(-50*D2R));
	via_posrpy_down.bottomRows(3)<<rpy_arc, rpy_arc, rpy_arc, rpy_arc, rpy_arc;
}

void MirrorCleanPlanner::CalcRotEllipse(const MatrixXd& vertices)
{
	Vector3d origin = 0.5*(0.5*(vertices.col(0)+vertices.col(2))
		+0.5*(vertices.col(1)+vertices.col(3)));
	Vector3d x0_ellipse = vertices.col(0)-origin;
	Vector3d z0_ellipse = x0_ellipse.cross(static_cast<Vector3d>(vertices.col(3)-vertices.col(1)));
	Vector3d y0_ellipse = z0_ellipse.cross(x0_ellipse);
	_rot_plane<<x0_ellipse.normalized(), y0_ellipse.normalized(), z0_ellipse.normalized();
}

void MirrorCleanPlanner::CalcEllipseParams(const MatrixXd& vertices)
{
	_ellipse_params.origin = 0.5*(0.5*(vertices.col(0)+vertices.col(2))
		+0.5*(vertices.col(1)+vertices.col(3)));
	_ellipse_params.a = 0.5*((_ellipse_params.origin-vertices.col(1)).norm()
		+(_ellipse_params.origin-vertices.col(3)).norm());
	_ellipse_params.b = 0.5*((_ellipse_params.origin-vertices.col(0)).norm()
		+(_ellipse_params.origin-vertices.col(2)).norm());
}

void MirrorCleanPlanner::PlanRunwayMirror(MatrixXd& via_posrpy_up, MatrixXd& via_posrpy_middle,
	MatrixXd& via_posrpy_down)
{
	PlanTools::QuadranglePlanner rect_planner;
	// plan for middle zone
	rect_planner.UniversalPlan(via_posrpy_middle, &_vertices, _clean_params.clean_tool, _clean_params.pitch_angle,
		_clean_params.yaw_angle, _clean_params.dis_trans, _clean_params.camera_ori, _clean_params.path_type,_clean_params.trans_angle);
	// plan for up zone
	double r = 0.5*(_vertices.col(2)-_vertices.col(3)).norm();
	Vector3d origin = 0.5*(_vertices.col(2)+_vertices.col(3));
	Vector3d arc_up = origin+r*_rot_plane.col(0);
	arc_up -= 0.5*_clean_params.clean_tool[0]*_rot_plane.col(0);
	via_posrpy_up.setZero(6, 5);
	via_posrpy_up.block(0, 1, 3, 3)<<_vertices.col(2)-0.5*_clean_params.clean_tool[0]*_rot_plane.col(0),
		arc_up, _vertices.col(3)-0.5*_clean_params.clean_tool[0]*_rot_plane.col(0);
	via_posrpy_up.topLeftCorner(3, 1) = _vertices.col(2)+_clean_params.dis_trans*_rot_plane.col(2);
	via_posrpy_up.topRightCorner(3, 1) = _vertices.col(3)+_clean_params.dis_trans*_rot_plane.col(2);
	Matrix3d mat_tmp;
	mat_tmp<<-1, 0, 0, 0, 1, 0, 0, 0, -1;
	Vector3d rpy_arc = RobotTools::Tr2FixedZYX(_rot_plane*mat_tmp*RobotTools::RotX(-50*D2R));
	via_posrpy_up.bottomRows(3)<<rpy_arc, rpy_arc, rpy_arc, rpy_arc, rpy_arc;
	// plan for down zone
	r = 0.5*(_vertices.col(0)-_vertices.col(1)).norm();
	origin = 0.5*(_vertices.col(0)+_vertices.col(1));
	Vector3d arc_down = origin-r*_rot_plane.col(0);
	arc_down += 0.5*_clean_params.clean_tool[0]*_rot_plane.col(0);
	via_posrpy_down.setZero(6, 5);
	via_posrpy_down.block(0, 1, 3, 3)<<_vertices.col(1)+0.5*_clean_params.clean_tool[0]*_rot_plane.col(0),
		arc_down, _vertices.col(0)+0.5*_clean_params.clean_tool[0]*_rot_plane.col(0);
	via_posrpy_down.topLeftCorner(3, 1) = _vertices.col(1)+_clean_params.dis_trans*_rot_plane.col(2);
	via_posrpy_down.topRightCorner(3, 1) = _vertices.col(0)+_clean_params.dis_trans*_rot_plane.col(2);
	mat_tmp<<-1, 0, 0, 0, 1, 0, 0, 0, -1;
	rpy_arc = RobotTools::Tr2FixedZYX(_rot_plane*mat_tmp*RobotTools::RotX(-50*D2R));
	via_posrpy_down.bottomRows(3)<<rpy_arc, rpy_arc, rpy_arc, rpy_arc, rpy_arc;
}

void MirrorCleanPlanner::CalcRotRunway(const MatrixXd& vertices)
{
	Vector3d x0_runway = vertices.col(3)-vertices.col(0);
	Vector3d z0_runway = static_cast<Vector3d>(vertices.col(3)-vertices.col(1)).cross(
		static_cast<Vector3d>(vertices.col(2)-vertices.col(0)));
	Vector3d y0_runway = z0_runway.cross(x0_runway);
	_rot_plane<<x0_runway.normalized(), y0_runway.normalized(), z0_runway.normalized();
}

void MirrorCleanPlanner::PlanOctagonMirror(MatrixXd& via_posrpy_up, MatrixXd& via_posrpy_middle,
	MatrixXd& via_posrpy_down)
{
	PlanTools::QuadranglePlanner rect_planner;
	via_posrpy_up.setZero(6, 1);
	// plan for up zone
	MatrixXd rect_vertices(3, 4);
	rect_vertices<<_vertices.col(0), _vertices.middleCols(2, 3);
	rect_planner.UniversalPlan(via_posrpy_middle, &rect_vertices, _clean_params.clean_tool, _clean_params.pitch_angle,
		_clean_params.yaw_angle, _clean_params.dis_trans, _clean_params.camera_ori, _clean_params.path_type, _clean_params.trans_angle);
	_rot_plane = rect_planner.GetPlaneRotation();
	// plan for down zone
	Vector3d pos_tmp1 = _vertices.col(2)+0.5*_clean_params.clean_tool[0]*_rot_plane.col(0);
	Vector3d pos_tmp2 = _vertices.col(1)+0.5*_clean_params.clean_tool[0]*_rot_plane.col(0);
	MatrixXd pos_left(3, 4);
	pos_left<<pos_tmp1+_clean_params.dis_trans*_rot_plane.col(2), pos_tmp1,
		pos_tmp2, pos_tmp2+_clean_params.dis_trans*_rot_plane.col(2);
	Matrix3d mat_tmp;
	mat_tmp<<-1, 0, 0, 0, 1, 0, 0, 0, -1;
	Vector3d rpy_left = RobotTools::Tr2FixedZYX(_rot_plane*mat_tmp*RobotTools::RotX(-50*D2R));
	mat_tmp<<0, 0, 1, 1, 0, 0, 0, 1, 0;
	Vector3d rpy_tmp = RobotTools::Tr2FixedZYX(mat_tmp);
	MatrixXd rpy_mat(3, 4);
	rpy_mat<<rpy_left, rpy_left, rpy_left, rpy_tmp;
	MatrixXd posrpy_left(6, 4);
	posrpy_left<<pos_left, rpy_mat;
	pos_tmp1 = _vertices.col(0)+0.5*_clean_params.clean_tool[0]*_rot_plane.col(0);
	MatrixXd pos_right(3, 4);
	pos_right<<pos_tmp1+_clean_params.dis_trans*_rot_plane.col(2), pos_tmp1,
		pos_tmp2, pos_tmp2+_clean_params.dis_trans*_rot_plane.col(2);
	mat_tmp<<-1, 0, 0, 0, 1, 0, 0, 0, -1;
	Vector3d rpy_right = RobotTools::Tr2FixedZYX(_rot_plane*mat_tmp*RobotTools::RotX(50*D2R));
	rpy_mat<<rpy_right, rpy_right, rpy_right, rpy_tmp;
	MatrixXd posrpy_right(6, 4);
	posrpy_right<<pos_right, rpy_mat;
	via_posrpy_down.setZero(6, 8);
	via_posrpy_down<<posrpy_left, posrpy_right;
}



MirrorCleanPlanner::~MirrorCleanPlanner()
{
}
