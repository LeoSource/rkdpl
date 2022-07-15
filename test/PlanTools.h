#pragma once

#include "RobotMath.h"


namespace PlanTools
{
/**
* @detail	
			A specific class that generate positions and orientations to guarantee that all the positions
			can cover a given quadrangle area
			the sequence of quadrangle vertices is like this
			3———————4
			|       x^
			|        |
			| y      |
			2<——————1
			clean_tool: the size of clean tool, includes legnth for direction y and width for direction x
			yaw_angle(rotz): when it is bigger than pi or less than -pi, it will be defined
			by an adaptive method, otherwise, use its input variable to define orientation
			pitch_angle(roty): it dose not hava an adaptive method to set pitch
			for example:
			mirror: pitch_angle = [60degree, 60degree] yaw_angle = [0,0]
			table: pitch_angle = [50degree, 130degree] yaw_angle = [-10,0]
			dis_trans: transition distance with surface constructed by vertices
			camera_ori: includes 4 types, front, down, left, right
* @version	2.0.0
* @author	zxliao
* @email	zhixiangleo@163.com
* @date		2021/9/22
**/
	class QuadranglePlanner
	{
	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	private:
		Matrix3d _rot_plane;
		Matrix3d _rot_transform;
		MatrixXd _vertices_new;

		int _cycle_num;
		double _step_size1, _step_size2;
		Vector3d _step_vec1, _step_vec2;
		Vector3d _start_pos1, _start_pos2;

	public:
		QuadranglePlanner() {}

		void UniversalPlan(MatrixXd& via_posrpy,MatrixXd* vertices,double* clean_tool,
							double* pitch_angle,double* yaw_angle,double dis_trans,
							string camera_ori,string path_type,double trans_angle=0);

		void PlanMirror(MatrixXd& via_posrpy,MatrixXd* vertices,int hori_times);

		void PlanTable(MatrixXd& via_posrpy,MatrixXd* vertices);

		void PlanGround(MatrixXd& via_posrpy,MatrixXd* vertices);

		Matrix3d GetPlaneRotation();

	private:
		void PlanNPath(MatrixXd& via_posrpy, double* clean_tool, double* pitch_angle,
						double* yaw_angle, double dis_trans,double trans_angle=0);

		void PlanSPath(MatrixXd& via_posrpy, double* clean_tool, double* pitch_angle,
						double* yaw_angle, double dis_trans,double trans_angle=0);

		void CalcCycleWaypoint(MatrixXd& via_posrpy, double* pitch_angle,
								double* yaw_angle, string path_type);

		void CalcCycleWaypoint(MatrixXd& via_posrpy, double* pitch_angle, double* yaw_angle,
								Vector3d trans_vec, string path_type);

		void CalcCycleInfo(double interval, Vector3d vec1, Vector3d vec2);

		void CalcNPathRot(double* pitch_out, double* yaw_out, double* pitch_in, 
							double* yaw_in, Vector3d pos1, Vector3d pos2, int idx);

		void CalcSPathRot(double* pitch_out, double* yaw_out, double* pitch_in,
							double* yaw_in, Vector3d pos, int idx);

		void CalcPlaneRot(MatrixXd* vertices);

		double CalcNPathYaw(Vector3d pos,double yaw_angle,int index);

		double CalcSPathYaw(Vector3d pos,double* yaw_range,int index);

		double CalcAdaptiveYaw(Vector3d origin,Vector3d pos);

		double CalcSPathPitch(double* pitch_range,int index);

		void CalcCleanAreaVertices(MatrixXd* vertices,double* clean_tool);

		Vector3d CalcNewVertice(Vector3d v1, Vector3d v2, double nlen1, double nlen2,
								Vector3d nv1, Vector3d vertice);

		void SetRotTransformation(string camera_ori);

		void HorizontalScrape(MatrixXd& via_posrpy,MatrixXd* vertices,double* clean_tool,int hori_times);
	};

	class CircPlanner
	{
	public:
		CircPlanner() {}

		/*tap_theta: tap pos
		angle: line area
		dis_trans: interval
		rx ry rz: attitude compensation*/
		int CircularPlaneBasin(MatrixXd& via_posrpy, MatrixXd* vision_pos,
			double rx[6], double ry[6], double rz[6], double dis_trans, double angle, double tap_theta);

		int InnerCylindricalBasin(MatrixXd& via_posrpy, MatrixXd* vision_pos,
			double rx[6], double ry[6], double rz[6], double dis_trans, double angle, double tap_theta);
		/*vision_pos: 3(provide path) + 4(provide rot)
		*/
		int OuterCylindricalBasin(MatrixXd& via_posrpy, MatrixXd* vision_pos,
			double rx[3], double ry[3], double rz[3], double dis_trans, double height);

		int InnerSphericalBasin(MatrixXd& via_posrpy, MatrixXd* vision_pos,
			double rx[6], double ry[6], double rz[6], double dis_angle, double angle, double tap_theta);

		int OuterSphericalBasin(MatrixXd& via_posrpy, MatrixXd* vision_pos,
			double rx[3], double ry[3], double rz[3], double dis_angle, double r1);

		void UpperEdgeBasin(MatrixXd& via_posrpy, MatrixXd* vision_pos,
			double rx[5], double ry[5], double rz[5], double angle, double tap_theta);
	private:
		void CalcArcInfo(Vector3d& center, double& radius, Matrix3d& rot, double& theta,
			Vector3d pos1, Vector3d pos2, Vector3d pos3);

		void CalcArcPoint(Vector3d& pos, Vector3d pos1, Vector3d pos2, Vector3d pos3, double angle);

	};

	class EllipsePlanner
	{
	public:
		EllipsePlanner() {}

		int InnerEllipseBasin(MatrixXd& via_posrpy, MatrixXd* vision_pos,
			double rx[6], double ry[6], double rz[6], double dis_angle, double angle, double tap_theta);

		int OuterEllipseBasin(MatrixXd& via_posrpy, MatrixXd* vision_pos,
			double rx[2], double ry[2], double rz[2], double dis_trans);

		int UpperEdgeBasin(MatrixXd& via_posrpy, MatrixXd* vision_pos,
			double rx[6], double ry[6], double rz[6], double dis_angle, double angle, double tap_theta);
	private:
		void CalcEllipsePoint(Vector3d& pos, Vector3d pos1, Vector3d pos2, Vector3d pos3, double angle);

		void InitEllipseInfo(Vector3d& center, double& a, double& b, Matrix3d& rot,
			Vector3d pos1, Vector3d pos2, Vector3d pos3);
	};

}


namespace PlanTools
{
	Matrix3d CalcPlaneRot(Vector3d center, Vector3d norm_vec);

	void CalcBasinPath(MatrixXd& in_via_posrpy, MatrixXd& out_via_posrpy,
					MatrixXd* vision_pos, double ang, double interval);

	void CalcToiletPath(MatrixXd& via_posrpy, MatrixXd* vision_pos, double theta);

	void CalcTablePath(MatrixXd& via_posrpy, MatrixXd* vision_pos, double theta, double interval);

	void PlanToiletInlierPath(MatrixXd& via_posrpy, MatrixXd* vision_pos,
							double slant, string camera_ori);
	
	/**	argumets examples:
		right config:
		open -> PlanToiletlidPath(vision_pos,110degree,-110degree,90degree,0.03)
		close(toilet lid) -> PlanToiletlidPath(vision_pos,-60degree,0degree,150degree,0.03)
		close(toilet seat) -> PlanToiletlidPath(vision_pos,-60degree,-100degree,20degree,0.03)
		left config:
		open -> PlanToiletlidPath(vision_pos,110degree,-(90+10)degree,0degree,0.03)
		close -> PlanToiletlidPath(vision_pos,-60degree,-(180-30)degree,-40degree,0.03)
	**/
	void PlanToiletlidPath(MatrixXd& via_posrpy, MatrixXd* vision_pos, double arc_angle,
							double point_angle, double slant_angle, double dis_trans);

	void CalcMirrorPath_Line(MatrixXd& vis_posrpy, MatrixXd* corner_pos,
		double lenscraper, double inc_ang,
		double dis_trans, std::string camera_ori);

	void CalcMirrorPath(MatrixXd& via_posrpy, MatrixXd* corner_pos, double lenscraper,
						double slant_ang, double inc_ang);

	int CalcCircleMirrorPath(MatrixXd& circle_via_posrpy, MatrixXd& line_via_posrpy,
							MatrixXd* vision_pos, double lenscraper, double inc_ang);
	/*use tap point*/
	int CalcCircBasinPath(MatrixXd& circle_via_posrpy, MatrixXd& line_via_posrpy,
							MatrixXd* vision_pos, double diameterball);
	/*up
	inner_wall
	cylindrical
	*/
	
	/*up
	inner_bottom
	flat
	*/

	/*up
	outer_wall
	cylindrical
	*/
	int CalcCircBasinPath(MatrixXd& outer_via_posrpy, MatrixXd* vision_pos,
		double rx[6], double ry[6], double rz[6], double dis_angle, double angle, double tap_theta);

	/*up
	upper_edge
	circ
	*/

	void CalcArcInfo(Vector3d& center, double& radius, Matrix3d& rot, double& theta,
						Vector3d pos1, Vector3d pos2, Vector3d pos3);

	void CalcArcPoint(Vector3d& pos, Vector3d pos1, Vector3d pos2, Vector3d pos3, double angle);

	int CalcEllipseBasinPath(MatrixXd& via_posrpy, MatrixXd* vision_pos, 
		double rx[6], double ry[6], double rz[6], double dis_angle, double angle);

	/*	tap_theta  example: right tap  20deg, left tap -20deg
	*	angle should be positive  20 or 10 deg
	*	rx[4] ry[4] rz[4] tap obstacle avoidance
	*/
	int CalcEllipseBasinPath(MatrixXd& via_posrpy, MatrixXd* vision_pos,
		double rx[6], double ry[6], double rz[6], double dis_angle, double angle, double tap_theta);

	void CalcEllipsePoint(Vector3d& pos, Vector3d pos1, Vector3d pos2, Vector3d pos3, double angle);

	void InitEllipseInfo(Vector3d& center, double& a, double& b, Matrix3d& rot,
		Vector3d pos1, Vector3d pos2, Vector3d pos3);
}