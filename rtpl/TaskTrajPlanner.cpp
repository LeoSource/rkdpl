#include "TaskTrajPlanner.h"
#include "BaseTrajPlanner.h"
#include "CartesianPlanner.h"
#include "JointPlanner.h"
#include "CubicBSplinePlanner.h"
#include "CirclePlanner.h"
#include "EllipsePlanner.h"

TaskTrajPlanner::TaskTrajPlanner(UrRobot* robot_model, Vector6d q0, double cycle_time,
			double* base_jvmax, double* base_jamax, double* base_jjmax, double* base_cvmax, double* base_camax, double* base_cjmax)
	:_robot(robot_model),_pre_q(q0),_pre_trajq(q0),_cycle_time(cycle_time)
{
	_cvmax = base_cvmax;
	_camax = base_camax;
	_cjmax = base_cjmax;

	_jvmax = base_jvmax;
	_jamax = base_jamax;
	_jjmax= base_jjmax;

	_task_completed = false;
	_ntraj = 0;
	_traj_idx = 0;
	_pre_trajpose = _robot->FKSolveTool(q0);
	InitAddMap();
}

void TaskTrajPlanner::SetPlanningScene(const MatrixXd& vertices, double slant)
{
	_clean_toilet_center(0) = vertices.row(0).mean();
	_clean_toilet_center(1) = vertices.row(1).mean();
	_clean_toilet_center(2) = vertices.row(2).mean();
	_clean_toilet_slant = slant;
	// Vector3d center;
	// center(0) = vertices.row(0).mean();
	// center(1) = vertices.row(1).mean();
	// center(2) = vertices.row(2).mean();
	// MatrixXd peak(vertices);
	// for (int idx = 0; idx<(int)(vertices.cols()); idx++)
	// {
	// 	Vector3d pos_tmp = vertices.col(idx);
	// 	Vector3d center_tmp(center(0), center(1), pos_tmp(2));
	// 	double len = (center_tmp-pos_tmp).norm();
	// 	peak.col(idx) = center_tmp;
	// 	peak.col(idx)(2) = center_tmp(2)+len/tan(slant);
	// }
	// _peak_clean_toilet(0) = peak.row(0).mean();
	// _peak_clean_toilet(1) = peak.row(1).mean();
	// _peak_clean_toilet(2) = peak.row(2).mean();
}

void TaskTrajPlanner::AddTraj(DynIdenTrajPlanner* dyniden_planner, TrajType traj_type)
{
	// add joint trajectory for transition
	MatrixXd vp = dyniden_planner->_q0.at(0);
	AddJointTraj(&vp, true, _jvmax, _jamax);
	// add robot dynamic identification trajectory
	_dyniden_planner = dyniden_planner;
	_ntraj += 1;
	_traj_type.push_back(eDynIden);
}

void TaskTrajPlanner::InitAddMap()
{
	struct AddMethodInfo
	{
		RobotTools::TrajType MethodID;
		PtrAddMethod AddMethod;
	};
	AddMethodInfo add_config[] =
	{
		{ eJointSpace,		&TaskTrajPlanner::AddJointTraj		},
		{ eCartesianSpace,	&TaskTrajPlanner::AddCartesianTraj	},
		{ eCartesianArc,	&TaskTrajPlanner::AddArcTraj		},
		{ eBSpline,			&TaskTrajPlanner::AddBSplineTraj	},
		{ eCircle,			&TaskTrajPlanner::AddCircleTraj		},
		{ eEllipse,			&TaskTrajPlanner::AddEllipseTraj	}
	};
	int add_count = sizeof(add_config)/sizeof(*add_config);
	while (add_count--)
		_addmethod_map[add_config[add_count].MethodID] = add_config[add_count].AddMethod;
}

void TaskTrajPlanner::AddTraj(MatrixXd* via_pos, TrajType traj_type, bool traj_opt,
							double vel_ratio, double acc_ratio, double angle_start, double angle_end)
{
	if(traj_type==eJointSpace)
	{
		for(int idx=0;idx<6;idx++)
		{
			_jvmax[idx] = _jvmax[idx]*vel_ratio;
			_jamax[idx] = _jamax[idx]*acc_ratio;
		}
		(this->*_addmethod_map[traj_type])(via_pos, traj_opt, _jvmax, _jamax);
	}
	else
	{
		for (int idx = 0; idx<2; idx++)
		{
			_cvmax[idx] = _cvmax[idx]*vel_ratio;
			_camax[idx] = _camax[idx]*acc_ratio;
		}
		(this->*_addmethod_map[traj_type])(via_pos, traj_opt, _cvmax, _camax);
		_angle = (traj_type==eCircle) ? angle_start : 0;
		_angle_start = (traj_type==eEllipse) ? angle_start : 0;
		_angle_end = (traj_type==eEllipse) ? angle_end : 0;
	}
}

void TaskTrajPlanner::AddTraj(MatrixXd* via_pos, TrajType traj_type, bool traj_opt,
								double* vmax, double* amax)
{
	(this->*_addmethod_map[traj_type])(via_pos, traj_opt, vmax, amax);
}

void TaskTrajPlanner::AddJointTraj(MatrixXd* via_pos, bool traj_opt, double* vmax, double* amax)
{
	double qlimit[12];
	for (int idx = 0; idx<6; idx++)
	{
		qlimit[2*idx] = _robot->_qlimit(idx, 0);
		qlimit[2*idx+1] = _robot->_qlimit(idx, 1);
	}
	BaseTrajPlanner* jplanner = new JointPlanner(_pre_trajq, traj_opt,_cycle_time,qlimit,_jjmax);
	jplanner->AddViaPos(via_pos,vmax,amax);
	_segplanner.push_back(jplanner);
	_ntraj += 1;
	_traj_type.push_back(eJointSpace);
	_pre_trajq = via_pos->rightCols(1);
	_pre_trajpose = _robot->FKSolveTool(_pre_trajq);
}

void TaskTrajPlanner::AddCartesianTraj(MatrixXd* via_pos, bool traj_opt, double* vmax, double* amax)
{
	int np = static_cast<int>(via_pos->cols());
	for (int idx = 0; idx < np; idx++)
	{
		Vector6d pos_rpy = via_pos->col(idx);
		CheckCartWorkspace(pos_rpy.head(3));
	}
	Vector3d pos0 = _pre_trajpose.pos;
	Vector3d rpy0 = RobotTools::Tr2FixedZYX(_pre_trajpose.rot);
	Vector6d pos_rpy0;
	pos_rpy0<<pos0, rpy0;
	BaseTrajPlanner* cplanner = new CartesianPlanner(pos_rpy0, traj_opt,_cycle_time,_cjmax);
	cplanner->AddViaPos(via_pos,vmax,amax);
	_segplanner.push_back(cplanner);
	_ntraj += 1;
	_traj_type.push_back(eCartesianSpace);
	Vector6d pos_rpyn = via_pos->rightCols(1);
	_pre_trajpose.pos = pos_rpyn.head(3);
	_pre_trajpose.rot = RobotTools::FixedZYX2Tr(pos_rpyn.tail(3));
	_robot->IKSolve(&_pre_trajq, &_pre_trajpose, &_pre_trajq);
}

void TaskTrajPlanner::AddArcTraj(MatrixXd* via_pos, bool traj_opt, double* vmax, double* amax)
{
	//add line trajectory befor arc for transition
	Vector3d pos0 = _pre_trajpose.pos;
	Vector3d rpy0 = RobotTools::Tr2FixedZYX(_pre_trajpose.rot);
	Vector6d pos_rpy0;
	pos_rpy0<<pos0, rpy0;
	BaseTrajPlanner* lineplanner = new CartesianPlanner(pos_rpy0, false,_cycle_time,_cjmax);
	MatrixXd pos_trans = via_pos->col(0);
	lineplanner->AddViaPos(&pos_trans,vmax,amax);
	_segplanner.push_back(lineplanner);
	_ntraj += 1;
	_traj_type.push_back(eCartesianSpace);
	//add arc trajectory
	Vector6d pos_rpy1 = via_pos->col(0);
	Vector6d pos_rpy3 = via_pos->col(2);
	Vector3d pos2 = via_pos->col(1).head(3);
	BaseTrajPlanner* arcplanner = new CartesianPlanner(&pos_rpy1,&pos2,&pos_rpy3,vmax,amax,_cjmax,_cycle_time);
	_segplanner.push_back(arcplanner);
	_ntraj += 1;
	_traj_type.push_back(eCartesianArc);
	Vector6d pos_rpyn = via_pos->col(2);
	_pre_trajpose.pos = pos_rpyn.head(3);
	_pre_trajpose.rot = RobotTools::FixedZYX2Tr(pos_rpyn.tail(3));
	_robot->IKSolve(&_pre_trajq, &_pre_trajpose, &_pre_trajq);
}

void TaskTrajPlanner::AddBSplineTraj(MatrixXd* via_pos, bool traj_opt, double* vmax, double* amax)
{
	int np = static_cast<int>(via_pos->cols());
	for (int idx = 0; idx < np; idx++)
		CheckCartWorkspace(via_pos->col(idx).head(3));
	// add line trajectory before b-spline for transition
	Vector3d pos0 = _pre_trajpose.pos;
	Vector3d rpy0 = RobotTools::Tr2FixedZYX(_pre_trajpose.rot);
	Vector6d pos_rpy0;
	pos_rpy0<<pos0, rpy0;
	BaseTrajPlanner* lineplanner = new CartesianPlanner(pos_rpy0, false, _cycle_time,_cjmax);
	MatrixXd pos_trans(6, 1);
	pos_trans.topRows(3) = via_pos->col(0).topRows(3);
	pos_trans.bottomRows(3) = CalcCleanToiletRPY(via_pos->col(0));
	lineplanner->AddViaPos(&pos_trans, vmax, amax);
	_segplanner.push_back(lineplanner);
	_ntraj += 1;
	_traj_type.push_back(eCartesianSpace);
	_pre_trajpose.pos = via_pos->col(0);
	_pre_trajpose.rot = RobotTools::FixedZYX2Tr(pos_trans.bottomRows(3));
	_robot->IKSolve(&_pre_trajq, &_pre_trajpose, &_pre_trajq);
	// add b-spline trajectory
	double tf_uk = CalcBSplineTime(via_pos,vmax);
	BaseTrajPlanner* bplanner = new CubicBSplinePlanner(via_pos,traj_opt,tf_uk,_cycle_time);
	_segplanner.push_back(bplanner);
	_ntraj += 1;
	_traj_type.push_back(eBSpline);
}

void TaskTrajPlanner::AddCircleTraj(MatrixXd* via_pos, bool traj_opt, double* vmax, double* amax)
{
	// add line trajectory befor circle for transition
	Vector3d pos0 = _pre_trajpose.pos;
	Vector3d rpy0 = RobotTools::Tr2FixedZYX(_pre_trajpose.rot);
	Vector6d pos_rpy0;
	pos_rpy0 << pos0, rpy0;
	BaseTrajPlanner* lineplanner = new CartesianPlanner(pos_rpy0, false, _cycle_time,_cjmax);
	Vector6d pos_rpy1 = via_pos->col(0);
	Vector3d pos3 = via_pos->col(2).head(3);
	Vector3d pos2 = via_pos->col(1).head(3);
	BaseTrajPlanner* circleplanner = new CirclePlanner(&pos_rpy1, &pos2, &pos3, traj_opt, vmax, amax, _cjmax, _cycle_time, _angle);
	MatrixXd pos_trans;
	pos_trans.resize(6,1);
	pos_trans<<via_pos->col(0).head(3),circleplanner->_rpy_start;
	lineplanner->AddViaPos(&pos_trans, vmax, amax);
	_segplanner.push_back(lineplanner);
	_ntraj += 1;
	_traj_type.push_back(eCartesianSpace);

	_segplanner.push_back(circleplanner);
	_ntraj += 1;
	_traj_type.push_back(eCircle);
	Vector6d pos_rpyn = via_pos->col(0);
	_pre_trajpose.pos = pos_rpyn.head(3);
	_pre_trajpose.rot = RobotTools::FixedZYX2Tr(circleplanner->_rpy_end);
	_robot->IKSolve(&_pre_trajq, &_pre_trajpose, &_pre_trajq);
}

void TaskTrajPlanner::AddEllipseTraj(MatrixXd* via_pos, bool traj_opt, double* vmax, double* amax)
{
	//add line trajectory befor ellipse for transition
	Vector3d pos0 = _pre_trajpose.pos;
	Vector3d rpy0 = RobotTools::Tr2FixedZYX(_pre_trajpose.rot);
	Vector6d pos_rpy0;
	pos_rpy0 << pos0, rpy0;
	BaseTrajPlanner* lineplanner = new CartesianPlanner(pos_rpy0, false, _cycle_time,_cjmax);
	Vector6d pos_rpy1 = via_pos->col(0);
	Vector6d pos_rpy3 = via_pos->col(2);
	Vector3d pos2 = via_pos->col(1).head(3);
	BaseTrajPlanner* ellipseplanner = new EllipsePlanner(&pos_rpy1, &pos2, &pos_rpy3, vmax, amax,_cjmax, _cycle_time, _angle_start, _angle_end);
	MatrixXd pos_trans;
	pos_trans.resize(6, 1);
	pos_trans << ellipseplanner->_pos_start, via_pos->col(0).tail(3);
	lineplanner->AddViaPos(&pos_trans, vmax, amax);
	_segplanner.push_back(lineplanner);
	_ntraj += 1;
	_traj_type.push_back(eCartesianSpace);

	_segplanner.push_back(ellipseplanner);
	_ntraj += 1;
	_traj_type.push_back(eEllipse);
	Vector6d pos_rpyn = via_pos->col(2);
	_pre_trajpose.pos = ellipseplanner->_pos_end;
	_pre_trajpose.rot = RobotTools::FixedZYX2Tr(pos_rpyn.tail(3));
	_robot->IKSolve(&_pre_trajq, &_pre_trajpose, &_pre_trajq);
}

void TaskTrajPlanner::GenerateJPath(Vector6d& jpos)
{
	switch (_traj_type[_traj_idx])
	{
	case eJointSpace:
	{
		_segplanner[_traj_idx]->GeneratePath(jpos);
		_pre_q = jpos;
	}		
	break;
	case eCartesianSpace:
	{
		Vector6d cpos;
		_segplanner[_traj_idx]->GeneratePath(cpos);
		Pose cmd_pose;
		cmd_pose.pos = cpos.head(3);
		cmd_pose.rot = RobotTools::FixedZYX2Tr(cpos.tail(3));
		_robot->IKSolve(&jpos, &cmd_pose, &_pre_q);
		_pre_q = jpos;
	}
	break;
	case eBSpline:
	{
		Vector6d cpos;
		_segplanner[_traj_idx]->GeneratePath(cpos);
		cpos.tail(3) = CalcCleanToiletRPY(cpos.head(3));
		Pose cmd_pose;
		cmd_pose.pos = cpos.head(3);
		cmd_pose.rot = RobotTools::FixedZYX2Tr(cpos.tail(3));
		_robot->IKSolve(&jpos, &cmd_pose, &_pre_q);
		_pre_q = jpos;
	}
	break;
	case eCartesianArc:
	case eCircle:
	case eEllipse:
	{
		Vector6d cpos;
		_segplanner[_traj_idx]->GeneratePath(cpos);
		Pose cmd_pose;
		cmd_pose.pos = cpos.head(3);
		cmd_pose.rot = RobotTools::FixedZYX2Tr(cpos.tail(3));
		_robot->IKSolve(&jpos, &cmd_pose, &_pre_q);
		_pre_q = jpos;
	}
	break;
	case eDynIden:
	{
		_dyniden_planner->GenerateTraj(jpos);
		_pre_trajq = jpos;
		_pre_trajpose = _robot->FKSolveTool(jpos);
	}
	break;
	default:
		break;
	}
	bool plan_completed = (_traj_type[_traj_idx]==eDynIden) ? 
						_dyniden_planner->_plan_completed : 
						_segplanner[_traj_idx]->_plan_completed;
	if (plan_completed)
	{
		_pre_trajq = jpos;
		_pre_trajpose = _robot->FKSolveTool(jpos);
		if (_traj_idx==_ntraj-1)
			_task_completed = true;
		else
			_traj_idx += 1;
	}
}

void TaskTrajPlanner::GenerateCPath(Vector6d& cpos)
{
	switch (_traj_type[_traj_idx])
	{
	case eJointSpace:
	{
		Vector6d jpos;
		_segplanner[_traj_idx]->GeneratePath(jpos);
		Pose cpose = _robot->FKSolveTool(jpos);
		cpos.head(3) = cpose.pos;
		cpos.tail(3) = RobotTools::Tr2FixedZYX(cpose.rot);
		break;
	}
	case eCartesianSpace:
	{
		_segplanner[_traj_idx]->GeneratePath(cpos);
		break;
	}
	case eBSpline:
	{
		_segplanner[_traj_idx]->GeneratePath(cpos);

		break;
	}
	case eCartesianArc:
	case eCircle:
	case eEllipse:
	{
		_segplanner[_traj_idx]->GeneratePath(cpos);
		cpos.tail(3) = CalcCleanToiletRPY(cpos.head(3));
		break;
	}
	default:
		break;
	}
	if (_segplanner[_traj_idx]->_plan_completed)
	{
		if (_traj_idx==_ntraj-1)
		{
			_task_completed = true;
			_pre_trajpose.pos = cpos.head(3);
			_pre_trajpose.rot = RobotTools::FixedZYX2Tr(cpos.tail(3));
		}
		else
			_traj_idx += 1;
	}
}

void TaskTrajPlanner::GenerateBothPath(Vector6d& jpos, Vector6d& cpos)
{
	switch (_traj_type[_traj_idx])
	{
	case eJointSpace:
	{
		_segplanner[_traj_idx]->GeneratePath(jpos);
		Pose cpose = _robot->FKSolveTool(jpos);
		cpos.head(3) = cpose.pos;
		cpos.tail(3) = RobotTools::Tr2FixedZYX(cpose.rot);
		_pre_q = jpos;
		break;
	}
	case eCartesianSpace:
	{
		_segplanner[_traj_idx]->GeneratePath(cpos);
		Pose cmd_pose;
		cmd_pose.pos = cpos.head(3);
		cmd_pose.rot = RobotTools::FixedZYX2Tr(cpos.tail(3));
		_robot->IKSolve(&jpos, &cmd_pose, &_pre_q);
		_pre_q = jpos;
		break;
	}
	case eBSpline:
	{
		_segplanner[_traj_idx]->GeneratePath(cpos);
		cpos.tail(3) = CalcCleanToiletRPY(cpos.head(3));
		Pose cmd_pose;
		cmd_pose.pos = cpos.head(3);
		cmd_pose.rot = RobotTools::FixedZYX2Tr(cpos.tail(3));
		_robot->IKSolve(&jpos, &cmd_pose, &_pre_q);
		_pre_q = jpos;
		break;
	}
	case eCartesianArc:
	case eCircle:
	case eEllipse:
	{
		_segplanner[_traj_idx]->GeneratePath(cpos);
		Pose cmd_pose;
		cmd_pose.pos = cpos.head(3);
		cmd_pose.rot = RobotTools::FixedZYX2Tr(cpos.tail(3));
		_robot->IKSolve(&jpos, &cmd_pose, &_pre_q);
		_pre_q = jpos;
		break;
	}
	default:
		break;
	}
	if (_segplanner[_traj_idx]->_plan_completed)
	{
		if (_traj_idx==_ntraj-1)
		{
			_task_completed = true;
			_pre_trajq = jpos;
			_pre_trajpose = _robot->FKSolveTool(jpos);
		}
		else
			_traj_idx += 1;
	}
}

void TaskTrajPlanner::Reset(Vector6d q0)
{
	_pre_q = q0;
	_pre_trajq = q0;
	_pre_trajpose = _robot->FKSolveTool(q0);
	for (int idx = 0; idx<_ntraj; idx++)
	{
		_segplanner[idx]->Reset(q0, false);
	}
	for (auto it = _segplanner.begin(); it!=_segplanner.end(); it++)
	{
		if (*it!=nullptr)
		{
			delete *it;
			*it = nullptr;
		}
	}
	_segplanner.clear();
	_traj_type.clear();
	_traj_idx = 0;
	_ntraj = 0;
	_task_completed = false;
	_angle = 0;
	_angle_start = 0;
	_angle_end = 0;
}


double TaskTrajPlanner::CalcBSplineTime(MatrixXd* via_pos, double* vmax)
{
	double pos_len = 0;
	double rot_len = 0;
	int np = static_cast<int>(via_pos->cols());
	MatrixXd pos_bspline = via_pos->topRows(3);
	MatrixXd rpy_bspline = via_pos->bottomRows(3);
	for (int idx = 0; idx<np-1; idx++)
	{
		pos_len += MathTools::Norm(pos_bspline.col(idx+1)-pos_bspline.col(idx));
		if(via_pos->rows()==6)
			rot_len += RobotTools::CalcAngleAxis(rpy_bspline.col(idx), rpy_bspline.col(idx+1))(3);
	}
	double coef = 3;
	double tf_pos = pos_len/vmax[0]*coef;
	double tf_rot = rot_len/vmax[1]*coef;
	Vector2d tf(tf_pos, tf_rot);
	
	return tf.maxCoeff();
}

Vector3d TaskTrajPlanner::CalcCleanToiletRPY(Vector3d pos)
{
	// Vector3d z0 = pos-_peak_clean_toilet;
	double height = 0.5;
	double len = height*tan(_clean_toilet_slant);
	Vector3d vec_tmp = _clean_toilet_center-pos;
	Vector3d pos_tmp = pos+len*vec_tmp.normalized();
	Vector3d peak(pos_tmp);
	peak(2) = pos_tmp(2)+height;
	Vector3d z0 = pos-peak;
	z0.normalize();
	Vector3d y0 = z0.cross(Vector3d(-1, 0, 0));
	Vector3d x0 = y0.cross(z0);
	Matrix3d rot_mat;
	rot_mat<<x0, y0, z0;
	Vector3d rpy = RobotTools::Tr2FixedZYX(rot_mat);

	return rpy;
}

void TaskTrajPlanner::CheckCartWorkspace(Vector3d cpos)
{
	double radius_ws = 1.1;
	double ratio_ws = 0.9;//0.8
	double zmin = -0.6;
	double len_tool = _robot->GetToolLength();
	Vector3d center(0, 0, 0);
	if ((MathTools::Norm(cpos - center) > radius_ws*ratio_ws + len_tool)
		|| (cpos(2)<zmin - len_tool))
		throw eErrJointOutOfRange;
}
