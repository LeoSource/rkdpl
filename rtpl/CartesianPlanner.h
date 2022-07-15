/**
* @file		CartesianPlanner.h
* @brief	Cartesian space trajectory plan which includes line and arc
* @version	2.0.2
* @author	zxliao
* @email	zhixiangleo@163.com
* @date		2021/10/25
**/
#pragma once

#include "LinePlanner.hpp"
#include "ArcPlanner.hpp"
#include "BaseTrajPlanner.h"
#include <vector>

class CartesianPlanner : public BaseTrajPlanner
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
private:
	vector<BaseCartesianPlanner*> _segtraj_planner;
	int _ntraj;
	vector<Vector3d> _pos_corner;
	vector<Vector3d> _rpy_corner;
	vector<Vector3d> _pos_seg;
	vector<Vector3d> _rpy_seg;
	vector<double> _varc;
	double *_vmax, *_amax;
	double* _cjmax;
	int _traj_idx;
	bool _continuity;
	bool _arc_type;

public:
	CartesianPlanner() = default;
	CartesianPlanner(const CartesianPlanner& planner) = delete;
	CartesianPlanner& operator=(const CartesianPlanner& planner) = delete;

	/**
	* @brief	constructor with initial position and rotation
	* @author	zxliao
	* @date		2021/10/25
	* @param	pos_rpy0	initial position and rotation: rpy
	* @param	conti_type	continuous arc transition between 2 lines
	* @param	cycle_time	cycle time of planner
	**/
	CartesianPlanner(Vector6d pos_rpy0, bool conti_type, double cycle_time, double* cjmax);

	/**
	* @brief	constructor with 3 positions to define arc,
				retained for future
	* @author	zxliao
	* @date		2021/6/28
	* @param	pos_rpy1	1st position and rotation to define arc
	* @param	pos2		2nd position to define arc
	* @param	pos_rpy3	3rd position and ratation to define arc
	**/
	CartesianPlanner(Vector6d* pps_rpy1, Vector3d* pos2, Vector6d* pos_rpy3,
					double* vmax, double* amax,double* jmax, double cycle_time);

	/**
	* @brief	add task corner positino and rpy
	* @author	zxliao
	* @date		2021/6/7
	* @param	pos		corner position
	* @param	rpy		corner rotation: rpy
	**/
	void AddViaPos(MatrixXd* pos_rpy, double* vmax, double* amax) override;

	/**
	* @brief	generate trajectory data: position, rpy,
				spatial velocity and acceleration
	* @author	zxliao
	* @date		2021/8/4
	* @output	cpos	position and rpy
	* @output	cvel	spatial velocity
	* @output	cacc	spatial acceleration
	**/
	void GenerateMotion(Vector6d& cpos, Vector6d& cvel, Vector6d& cacc) override;

	/**
	* @brief	generate trajectory data: position, rpy,
	* @author	zxliao
	* @date		2021/8/4
	* @output	cpos	position and rpy
	**/
	void GeneratePath(Vector6d& cpos) override;

	/**
	* @brief	reset planner with position and rpy
	* @author	zxliao
	* @date		2021/6/7
	* @param	pos_rpy0	initial position and rotation: rpy
	* @param	conti_type	continuous arc transition between 2 lines
	**/
	void Reset(Vector6d pos_rpy0, bool sync) override;

	/**
	* @brief	reset task trajectory planner with arc trajectory type
	* @author	zxliao
	* @date		2021/6/28
	* @param	pos_rpy1	1st position and rotation to define arc
	* @param	pos2		2nd position to define arc
	* @param	pos_rpy3	3rd position and ratation to define arc
	**/
	void Reset(Vector6d* pos_rpy1, Vector3d* pos2, Vector6d* pos_rpy3);

	~CartesianPlanner() {}

private:
	/**
	* @brief	add task corner positino and rpy under continuous condition
	* @author	zxliao
	* @date		2021/6/8
	* @param	pos		corner position
	* @param	rpy		corner rotation: rpy
	**/
	void AddContiPosRPY(Vector3d pos, Vector3d rpy);

	/**
	* @brief	add task corner positino and rpy under discontinuous condition
	* @author	zxliao
	* @date		2021/6/8
	* @param	pos		corner position
	* @param	rpy		corner rotation: rpy
	**/
	void AddDiscontiPosRPY(Vector3d pos, Vector3d rpy);

	/**
	* @brief	generate trajectory data under continuous condition
	* @author	zxliao
	* @date		2021/6/8
	* @return	cavp	trajectory data: position, rpy, spatial velocity and acceleration
	**/
	void GenerateContiMotion(RobotTools::CAVP &cavp);

	/**
	* @brief	generate trajectory data under discontinuous condition
	* @author	zxliao
	* @date		2021/6/8
	* @return	cavp	trajectory data: position, rpy, spatial velocity and acceleration
	**/
	void GenerateDiscontiMotion(RobotTools::CAVP &cavp);

	/**
	* @brief	generate arc trajectory data
	* @author	zxliao
	* @date		2021/6/28
	* @return	cavp	 arc trajectory data: position, rpy, spatial velocity and acceleration
	**/
	void GenerateArcMotion(RobotTools::CAVP &cavp);

	/**
	* @brief	get the 3rd position to define arc
	* @author	zxliao
	* @date		2021/6/7
	* @param	p1			1st position to define arc
	* @param	p2			2nd position to define arc
	* @param	p3_corner	3rd position on the p2p3 extended line
	* @return	pos3		the 3rd position to define arc
	**/
	Vector3d UpdateSegPos(Vector3d p1, Vector3d p2, Vector3d p3_corner);

	/**
	* @brief	clear temp positions and planners
	* @author	zxliao
	* @date		2021/6/7
	**/
	void ClearTemp();

};
