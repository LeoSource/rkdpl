/**
* @file		JointPlanner.h
* @brief	Joint space trajectory plan
* @version	2.0.1
* @author	zxliao
* @email	zhixiangleo@163.com
* @date		2021/10/25
**/
#pragma once
#include "BaseTrajPlanner.h"
#include "LspbPlanner.h"
#include "SVelProfilePlanner.h"
#include <vector>

class JointPlanner : public BaseTrajPlanner
{
private:
	int _ntraj;
	int _traj_idx;
	bool _sync;
	bool _continuity;
	vector<Vector6d> _jpos;
	double *_vmax, *_amax;
	Vector6d _update_jpos;
	vector<vector<SVelProfilePlanner*>> _segplanner;
	vector<vector<SVelProfilePlanner*>> _upsegplanner;
	vector<vector<LspbPlanner*>> _segplanner1;
	vector<vector<LspbPlanner*>> _upsegplanner1;
	vector<double> _tf;
	vector<vector<bool>> _transition;
	vector<vector<double>> _transition_time;
	vector<vector<Vector2d>> _transition_pos;
	vector<vector<int>> _jplanned, _junplanned;
	double* _qlimit;
	double* _jjmax;

public:
	JointPlanner() = default;
	JointPlanner(const JointPlanner& planner) = delete;
	JointPlanner& operator=(const JointPlanner& planner) = delete;

	JointPlanner(Vector6d q0, bool sync, double cycle_time,double* qlimit,double* jjmax);

	void AddViaPos(MatrixXd* jpos, double* vmax, double* amax) override;

	void GenerateMotion(Vector6d& jpos, Vector6d& jvel, Vector6d& jacc) override;

	void GeneratePath(Vector6d& jpos) override;

	void Reset(Vector6d q0, bool sync) override;

	~JointPlanner() {}
private:
	void ClearTmp();

	void AddContiJpos(MatrixXd* jpos);

	void AddDiscontiJpos(MatrixXd* jpos);

	void GenerateContiMotion(Vector6d& jpos);

	void GenerateDiscontiMotion(Vector6d& jpos);
};

