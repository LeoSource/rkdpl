#include "PlanTools.h"
#include <iostream>

namespace PlanTools
{
	void QuadranglePlanner::PlanMirror(MatrixXd& via_posrpy,MatrixXd* vertices,int hori_times)
	{
		string path_type = "N";
		string camera_ori = "top";
		double dis_trans = 0.1;
		double pitch_angle[2] = {30*D2R, 50*D2R};
		double yaw_angle[2] = {0, 0};
		double clean_tool[2] = {0.25, 0};
		double trans_angle = 30*D2R;
		if (hori_times>0)
		{
			double len = (hori_times+1)*0.5*clean_tool[0];
			Vector3d v1 = vertices->col(0)+0.7*len*(vertices->col(3)-vertices->col(0)).normalized();
			Vector3d v2 = vertices->col(1)+0.7*len*(vertices->col(2)-vertices->col(1)).normalized();
			MatrixXd via_posrpy_vert, via_posrpy_hori;
			MatrixXd vertices_vert, vertices_hori;
			vertices_vert.setZero(3, 4);
			vertices_hori.setZero(3, 4);
			vertices_vert<<v1, v2, vertices->rightCols(2);
			vertices_hori<<vertices->leftCols(2), v2, v1;
			UniversalPlan(via_posrpy_vert, &vertices_vert, clean_tool, pitch_angle, yaw_angle,
				dis_trans, camera_ori, path_type, trans_angle);
			HorizontalScrape(via_posrpy_hori, &vertices_hori, clean_tool, hori_times);
			via_posrpy.resize(6, via_posrpy_vert.cols()+via_posrpy_hori.cols());
			via_posrpy<<via_posrpy_vert, via_posrpy_hori;
		}
		else
		{
			UniversalPlan(via_posrpy, vertices, clean_tool, pitch_angle, yaw_angle,
				dis_trans, camera_ori, path_type, trans_angle);
		}
	}

	void QuadranglePlanner::PlanTable(MatrixXd& via_posrpy,MatrixXd* vertices)
	{
		string path_type = "N";
		string camera_ori = "top";
		double dis_trans = 0.08;
		double pitch_angle[2] = {50*D2R, pi-50*D2R};
		double yaw_angle[2] = {70*D2R, 10*D2R};
		double clean_tool[2] = {0.1,0.1};
		UniversalPlan(via_posrpy,vertices,clean_tool,pitch_angle,yaw_angle,
					dis_trans,camera_ori,path_type);
	}

	void QuadranglePlanner::PlanGround(MatrixXd& via_posrpy,MatrixXd* vertices)
	{
		string path_type = "N";
		string camera_ori = "top";
		double dis_trans = 0.08;
		double pitch_angle[2] = {50*D2R, 80*D2R};
		double yaw_angle[2] = {70*D2R, 10*D2R};
		double clean_tool[2] = {0.2, 0.15};
		UniversalPlan(via_posrpy,vertices,clean_tool,pitch_angle,yaw_angle,
					dis_trans,camera_ori,path_type);
	}

	void QuadranglePlanner::UniversalPlan(MatrixXd& via_posrpy,MatrixXd* vertices,double* clean_tool,
											double* pitch_angle,double* yaw_angle,double dis_trans,
											string camera_ori,string path_type,double trans_angle)
	{
		CalcPlaneRot(vertices);
		CalcCleanAreaVertices(vertices,clean_tool);
		SetRotTransformation(camera_ori);
		if(path_type=="N")
			PlanNPath(via_posrpy,clean_tool,pitch_angle,yaw_angle,dis_trans,trans_angle);
		else if(path_type=="S")
			PlanSPath(via_posrpy,clean_tool,pitch_angle,yaw_angle,dis_trans,trans_angle);
	}

	Matrix3d QuadranglePlanner::GetPlaneRotation()
	{
		return _rot_plane;
	}

	void QuadranglePlanner::PlanNPath(MatrixXd& via_posrpy, double* clean_tool, double* pitch_angle,
										double* yaw_angle, double dis_trans,double trans_angle)
	{
		CalcCycleInfo(clean_tool[0],_vertices_new.col(2)-_vertices_new.col(3),
						_vertices_new.col(1)-_vertices_new.col(0));
		_start_pos1 = _vertices_new.col(3);
		_start_pos2 = _vertices_new.col(0);
		if(dis_trans>0)
		{
			Matrix3d rot_trans = _rot_plane*RobotTools::RotY(trans_angle);
			Vector3d trans_vec = dis_trans*rot_trans.col(2);
			CalcCycleWaypoint(via_posrpy,pitch_angle,yaw_angle,trans_vec,"N");
		}
		else
			CalcCycleWaypoint(via_posrpy,pitch_angle,yaw_angle,"N");
	}

	void QuadranglePlanner::PlanSPath(MatrixXd& via_posrpy, double* clean_tool, double* pitch_angle,
										double* yaw_angle, double dis_trans,double trans_angle)
	{
		CalcCycleInfo(clean_tool[1],_vertices_new.col(0)-_vertices_new.col(3),
						_vertices_new.col(1)-_vertices_new.col(2));
		_start_pos1 = _vertices_new.col(3);
		_start_pos2 = _vertices_new.col(2);
		if(dis_trans>0)
		{
			Matrix3d rot_trans = _rot_plane*RobotTools::RotY(trans_angle);
			Vector3d trans_vec = dis_trans*rot_trans.col(2);
			CalcCycleWaypoint(via_posrpy,pitch_angle,yaw_angle,trans_vec,"S");
		}
		else
			CalcCycleWaypoint(via_posrpy,pitch_angle,yaw_angle,"S");
	}

	void QuadranglePlanner::CalcCycleWaypoint(MatrixXd& via_posrpy, double* pitch_angle,
												double* yaw_angle, string path_type)
	{
		via_posrpy.resize(6,2*_cycle_num);
		for(int idx=0;idx<_cycle_num;idx++)
		{
			Vector3d pos1 = _start_pos1+idx*_step_size1*_step_vec1;
			Vector3d pos2 = _start_pos2+idx*_step_size2*_step_vec2;
			double pitch[2], yaw[2];
			if(path_type=="N")
				CalcNPathRot(pitch,yaw,pitch_angle,yaw_angle,pos1,pos2,idx);
			else if(path_type=="S")
				CalcSPathRot(pitch,yaw,pitch_angle,yaw_angle,pos1,idx);
			Matrix3d rot_tool1 = _rot_plane*RobotTools::RotZ(-yaw[0])
									*RobotTools::RotY(pitch[0])*_rot_transform;
			Matrix3d rot_tool2 = _rot_plane*RobotTools::RotZ(-yaw[1])
									*RobotTools::RotY(pitch[1])*_rot_transform;
			Vector3d rpy1 = RobotTools::Tr2FixedZYX(rot_tool1);
			Vector3d rpy2 = RobotTools::Tr2FixedZYX(rot_tool2);
			via_posrpy.col(2*idx) << pos1,rpy1;
			via_posrpy.col(2*idx+1) << pos2,rpy2;
		}
	}

	void QuadranglePlanner::CalcCycleWaypoint(MatrixXd& via_posrpy, double* pitch_angle,
										double* yaw_angle, Vector3d trans_vec, string path_type)
	{
		via_posrpy.resize(6,4*_cycle_num);
		for(int idx=0;idx<_cycle_num;idx++)
		{
			Vector3d pos2 = _start_pos1+idx*_step_size1*_step_vec1;
			Vector3d pos1 = pos2+trans_vec;
			Vector3d pos3 = _start_pos2+idx*_step_size2*_step_vec2;
			Vector3d pos4 = pos3+trans_vec;
			double pitch[2], yaw[2];
			if(path_type=="N")
				CalcNPathRot(pitch,yaw,pitch_angle,yaw_angle,pos2,pos3,idx);
			else if(path_type=="S")
				CalcSPathRot(pitch,yaw,pitch_angle,yaw_angle,pos2,idx);
			Matrix3d rot_tool1 = _rot_plane*RobotTools::RotZ(-yaw[0])
									*RobotTools::RotY(pitch[0])*_rot_transform;
			Matrix3d rot_tool2 = _rot_plane*RobotTools::RotZ(-yaw[1])
									*RobotTools::RotY(pitch[1])*_rot_transform;
			Vector3d rpy1 = RobotTools::Tr2FixedZYX(rot_tool1);
			Vector3d rpy2 = RobotTools::Tr2FixedZYX(rot_tool2);
			via_posrpy.col(4*idx) << pos1,rpy1;
			via_posrpy.col(4*idx+1) << pos2,rpy1;
			via_posrpy.col(4*idx+2) << pos3,rpy2;
			via_posrpy.col(4*idx+3) << pos4,rpy2;
		}
	}

	void QuadranglePlanner::CalcCycleInfo(double interval, Vector3d vec1, Vector3d vec2)
	{
		double len1 = vec1.norm();
		double len2 = vec2.norm();
		int cycle_num1 = (int)ceil(len1/interval)+1;
		int cycle_num2 = (int)ceil(len2/interval)+1;
		_cycle_num = max<int>(cycle_num1,cycle_num2);
		_step_size1 = len1/(_cycle_num-1);
		_step_size2 = len2/(_cycle_num-1);
		_step_vec1 = vec1/len1;
		_step_vec2 = vec2/len2;
	}

	void QuadranglePlanner::CalcNPathRot(double* pitch_out, double* yaw_out, double* pitch_in, 
										double* yaw_in, Vector3d pos1, Vector3d pos2, int idx)
	{
		pitch_out[0] = pitch_in[0];
		pitch_out[1] = pitch_in[1];
		yaw_out[0] = CalcNPathYaw(pos1, yaw_in[0], idx);
		yaw_out[1] = CalcNPathYaw(pos2, yaw_in[1], idx);
	}

	void QuadranglePlanner::CalcSPathRot(double* pitch_out, double* yaw_out, double* pitch_in,
										double* yaw_in, Vector3d pos, int idx)
	{
		pitch_out[0] = CalcSPathPitch(pitch_in, idx);
		pitch_out[1] = pitch_out[0];
		yaw_out[0] = CalcSPathYaw(pos, yaw_in, idx);
		yaw_out[1] = -yaw_out[0];
	}

	void QuadranglePlanner::CalcPlaneRot(MatrixXd* vertices)
	{
		Vector3d x0_plane = vertices->col(3)-vertices->col(0);
		Vector3d z0_plane = MathTools::Cross(x0_plane, vertices->col(1)-vertices->col(0));
		Vector3d y0_plane = z0_plane.cross(x0_plane);
		x0_plane.normalize();
		y0_plane.normalize();
		z0_plane.normalize();
		_rot_plane<<x0_plane,y0_plane,z0_plane;
	}

	double QuadranglePlanner::CalcNPathYaw(Vector3d pos,double yaw_angle,int index)
	{
		double yaw;
		if(yaw_angle>pi || yaw_angle<-pi)
		{
			Vector3d origin = 0.5*(_vertices_new.col(0)+_vertices_new.col(1));
			yaw = CalcAdaptiveYaw(origin,pos);
		}
		else
		{
			double step_yaw = 2*yaw_angle/(_cycle_num-1);
			yaw = yaw_angle-index*step_yaw;
		}
		return yaw;
	}

	double QuadranglePlanner::CalcSPathYaw(Vector3d pos,double* yaw_range,int index)
	{
		double yaw;
		if(yaw_range[0]>pi || yaw_range[0]<-pi)
		{
			Vector3d origin = 0.5*(_vertices_new.col(0)+_vertices_new.col(1));
			yaw = CalcAdaptiveYaw(origin,pos);
		}
		else
		{
			double step_yaw = fabs(yaw_range[1]-yaw_range[0])/(_cycle_num-1);
			yaw = yaw_range[0]-index*step_yaw;
		}
		return yaw;
	}

	double QuadranglePlanner::CalcAdaptiveYaw(Vector3d origin,Vector3d pos)
	{
		Vector3d x0_axis = _rot_plane.col(0);
		Vector3d z0_axis = _rot_plane.col(2);
		Vector3d tmp_vec = origin-pos;
		double cos_theta;
		if(tmp_vec.norm()<EPS5)
			cos_theta = 1;
		else
			cos_theta = tmp_vec.dot(-x0_axis)/tmp_vec.norm();
		double dir = tmp_vec.cross(-x0_axis)(2)/z0_axis(2);
		MathTools::LimitNum(-1,cos_theta,1);
		double yaw = MathTools::Sign(dir)*acos(cos_theta);
		return yaw;
	}

	double QuadranglePlanner::CalcSPathPitch(double* pitch_range,int index)
	{
		double step_pitch = (pitch_range[1]-pitch_range[0])/(_cycle_num-1);
		double pitch = pitch_range[0]+index*step_pitch;
		return pitch;
	}

	void QuadranglePlanner::CalcCleanAreaVertices(MatrixXd* vertices,double* clean_tool)
	{
		//simplify the clean area model
		double l = clean_tool[0];
		double w = clean_tool[1];
		Vector3d z0_plane = _rot_plane.col(2);
		Vector3d vec12 = vertices->col(1)-vertices->col(0);
		vec12.normalize();
		Vector3d vec14 = vertices->col(3)-vertices->col(0);
		vec14.normalize();
		Vector3d n_vec12 = vec12.cross(z0_plane);
		Vector3d n_vec14 = z0_plane.cross(vec14);
		_vertices_new.resize(3,4);
		_vertices_new.col(0) = CalcNewVertice(vec12,vec14,w/2,l/2,n_vec12,vertices->col(0));
		Vector3d vec23 = vertices->col(2)-vertices->col(1);
		vec23.normalize();
		Vector3d n_vec23 = vec23.cross(z0_plane);
		_vertices_new.col(1) = CalcNewVertice(vec23,-vec12,l/2,w/2,n_vec23,vertices->col(1));
		Vector3d vec34 = vertices->col(3)-vertices->col(2);
		vec34.normalize();
		Vector3d n_vec34 = vec34.cross(z0_plane);
		_vertices_new.col(2) = CalcNewVertice(vec34,-vec23,w/2,l/2,n_vec34,vertices->col(2));
		_vertices_new.col(3) = CalcNewVertice(-vec14,-vec34,l/2,w/2,n_vec14,vertices->col(3));
	}

	Vector3d QuadranglePlanner::CalcNewVertice(Vector3d v1, Vector3d v2, double nlen1,
											double nlen2, Vector3d nv1, Vector3d vertice)
	{
		double cos_theta = v1.dot(v2)/v1.norm()/v2.norm();
		MathTools::LimitNum(-1,cos_theta,1);
		double theta = acos(cos_theta);
		Vector3d vec_new, vertice_new;
		if(fabs(nlen1)<EPS5)
		{
			double len1 = nlen2/sin(theta);
			vec_new = len1*v1;
		}
		else if(fabs(nlen2)<EPS5)
		{
			double len2 = nlen1/sin(theta);
			vec_new = len2*v2;
		}
		else
		{
			double theta1 = atan2(nlen1/nlen2*sin(theta),1+nlen1/nlen2*cos(theta));
			double len1 = nlen1/tan(theta1);
			vec_new = len1*v1+nlen1*nv1;
		}
		vertice_new = vertice+vec_new;

		return vertice_new;
	}

	void QuadranglePlanner::SetRotTransformation(string camera_ori)
	{
		if(camera_ori=="top")
			_rot_transform<<0,0,1,1,0,0,0,1,0;
		else if(camera_ori=="left")
			_rot_transform<<0,0,1,0,1,0,-1,0,0;
		else if(camera_ori=="down")
			_rot_transform<<0,0,1,-1,0,0,0,-1,0;
		else if(camera_ori=="right")
			_rot_transform<<0,0,1,0,-1,0,1,0,0;
	}

	void QuadranglePlanner::HorizontalScrape(MatrixXd& via_posrpy, MatrixXd* vertices,
											double* clean_tool, int hori_times)
	{
		via_posrpy.resize(6, 4*hori_times*2);
		double len = (vertices->col(1)-vertices->col(0)).norm();
		double len_left_scale = 2.0/3.0;
		double len_left = len_left_scale*len;
		double len_right = (1-len_left_scale)*len;
		Vector3d via_pos_end = vertices->col(0)+len_right*_rot_plane.col(1);
		Vector3d via_rpy_end = RobotTools::Tr2FixedZYX(_rot_plane*
								RobotTools::RotY(50*D2R)*_rot_transform);
		Vector3d rpy_tmp = RobotTools::Tr2FixedZYX(_rot_plane*
								RobotTools::RotY(90*D2R)*_rot_transform);
		Matrix3d tf_left, tf_right;
		tf_left<<-1, 0, 0, 0, 0, 1, 0, 1, 0;
		tf_right<<1, 0, 0, 0, 0, -1, 0, 1, 0;
		Vector3d rpy_left = RobotTools::Tr2FixedZYX(_rot_plane*
								RobotTools::RotX(-50*D2R)*tf_left);
		Vector3d rpy_right = RobotTools::Tr2FixedZYX(_rot_plane*
								RobotTools::RotX(50*D2R)*tf_right);
		Vector3d trans_vec = 0.08*_rot_plane.col(2);
		MatrixXd via_left, via_right;
		via_left.setZero(6, 4*hori_times);
		via_right.setZero(6, 4*hori_times);
		for (int idx = 0; idx<hori_times; idx++)
		{
			Vector3d pos_tmp = vertices->col(1)+(idx+1)*0.5*
								clean_tool[0]*_rot_plane.col(0);
			via_left.col(4*idx)<<pos_tmp+trans_vec, rpy_left;
			via_left.col(4*idx+1)<<pos_tmp, rpy_left;
			via_left.col(4*idx+2)<<pos_tmp-len_left*_rot_plane.col(1), rpy_left;
			via_left.col(4*idx+3)<<via_pos_end+trans_vec, rpy_tmp;
		}
		for (int idx = 0; idx<hori_times; idx++)
		{
			Vector3d pos_tmp = vertices->col(0)+(idx+1)*0.5*
								clean_tool[0]*_rot_plane.col(0);
			via_right.col(4*idx)<<pos_tmp+trans_vec, rpy_right;
			via_right.col(4*idx+1)<<pos_tmp, rpy_right;
			via_right.col(4*idx+2)<<pos_tmp+len_right*_rot_plane.col(1), rpy_right;
			via_right.col(4*idx+3)<<via_pos_end+trans_vec, rpy_tmp;
		}
		MatrixXd via_hori;
		via_hori.resize(6, 4);
		Vector3d pos_tmp = vertices->col(3)-hori_times*0.5*
			clean_tool[0]*_rot_plane.col(0)+len_right*_rot_plane.col(1);
		via_hori.col(0)<<pos_tmp+trans_vec, via_rpy_end;
		via_hori.col(1)<<pos_tmp, via_rpy_end;
		via_hori.col(2)<<via_pos_end, via_rpy_end;
		via_hori.col(3)<<via_pos_end+trans_vec, via_rpy_end;
		via_posrpy<<via_left, via_right;
	}

	int CircPlanner::CircularPlaneBasin(MatrixXd& via_posrpy, MatrixXd* vision_pos,
		double rx[6], double ry[6], double rz[6], double dis_trans, double angle, double tap_theta)
	{
		Vector3d pos1, pos2, pos3, pos4, pos5, pos6;
		pos1 = vision_pos->col(0); pos2 = vision_pos->col(1); pos3 = vision_pos->col(2);
		pos4 = vision_pos->col(3); pos5 = vision_pos->col(4); pos6 = vision_pos->col(5);
		Vector3d center; double radious, open_angle; Matrix3d rot;
		CalcArcInfo(center, radious, rot, open_angle, pos4, pos1, pos2);
		int cycle_num = ceil(radious / dis_trans);
		double step_size = radious / (cycle_num);
		double step_rx[6], step_ry[6], step_rz[6];
		for (int idx = 0; idx < 6; idx++)
		{
			step_rx[idx] = -rx[idx] / cycle_num;
			step_ry[idx] = -ry[idx] / cycle_num;
			step_rz[idx] = -rz[idx] / cycle_num;
		}
		via_posrpy.resize(6, 6 * cycle_num);
		Vector3d cp1 = pos1 - center; Vector3d cp2 = pos2 - center; Vector3d cp3 = pos3 - center; Vector3d cp4 = pos4 - center;
		cp1.normalize(); cp2.normalize(); cp3.normalize(); cp4.normalize();
		for (int idx = 0; idx < cycle_num; idx++)
		{
			Vector3d temp_pos1;
			temp_pos1 = pos1 - step_size * cp1 * idx;
			Vector3d temp_pos2;
			temp_pos2 = pos1 - step_size * cp1 * idx;
			Vector3d temp_pos3;
			temp_pos3 = pos1 - step_size * cp1 * idx;
			Vector3d temp_pos4;
			temp_pos4 = pos1 - step_size * cp1 * idx;
			Vector3d temp_pos11;
			CalcArcPoint(temp_pos11, temp_pos1, temp_pos2, temp_pos3, tap_theta);
			Vector3d temp_pos12;
			CalcArcPoint(temp_pos12, temp_pos3, temp_pos4, temp_pos1, tap_theta);
			Vector3d temp_pos7;
			CalcArcPoint(temp_pos7, temp_pos3, temp_pos4, temp_pos1, pi + tap_theta - angle);
			Vector3d temp_pos8;
			CalcArcPoint(temp_pos8, temp_pos1, temp_pos2, temp_pos3, tap_theta + angle);
			via_posrpy.col(6 * idx).head(3) = temp_pos8;
			double tmp_rx1 = rx[0] + idx*step_rx[0];
			double tmp_ry1 = ry[0] + idx*step_ry[0];
			double tmp_rz1 = rz[0] + idx*step_rz[0];
			Matrix3d temp_rot1 = rot * RobotTools::RotX(tmp_rx1)*RobotTools::RotY(tmp_ry1) * RobotTools::RotZ(tmp_rz1);
			via_posrpy.col(6 * idx).tail(3) = RobotTools::Tr2FixedZYX(temp_rot1);

			via_posrpy.col(6 * idx + 1).head(3) = temp_pos2;
			double tmp_rx2 = rx[1] + idx*step_rx[1];
			double tmp_ry2 = ry[1] + idx*step_ry[1];
			double tmp_rz2 = rz[1] + idx*step_rz[1];
			Matrix3d temp_rot2 = rot * RobotTools::RotX(tmp_rx2)*RobotTools::RotY(tmp_ry2) * RobotTools::RotZ(tmp_rz2);
			via_posrpy.col(6 * idx + 1).tail(3) = RobotTools::Tr2FixedZYX(temp_rot2);

			via_posrpy.col(6 * idx + 2).head(3) = temp_pos12;
			double tmp_rx3 = rx[2] + idx*step_rx[2];
			double tmp_ry3 = ry[2] + idx*step_ry[2];
			double tmp_rz3 = rz[2] + idx*step_rz[2];
			Matrix3d temp_rot3 = rot * RobotTools::RotX(tmp_rx3)*RobotTools::RotY(tmp_ry3) * RobotTools::RotZ(tmp_rz3);
			via_posrpy.col(6 * idx + 2).tail(3) = RobotTools::Tr2FixedZYX(temp_rot3);

			via_posrpy.col(6 * idx + 3).head(3) = temp_pos4;
			double tmp_rx4 = rx[3] + idx*step_rx[3];
			double tmp_ry4 = ry[3] + idx*step_ry[3];
			double tmp_rz4 = rz[3] + idx*step_rz[3];
			Matrix3d temp_rot4 = rot * RobotTools::RotX(tmp_rx4)*RobotTools::RotY(tmp_ry4) * RobotTools::RotZ(tmp_rz4);
			via_posrpy.col(6 * idx + 3).tail(3) = RobotTools::Tr2FixedZYX(temp_rot4);

			via_posrpy.col(6 * idx + 4).head(3) = temp_pos7;
			double tmp_rx5 = rx[4] + idx*step_rx[4];
			double tmp_ry5 = ry[4] + idx*step_ry[4];
			double tmp_rz5 = rz[4] + idx*step_rz[4];
			Matrix3d temp_rot5 = rot * RobotTools::RotX(tmp_rx5)*RobotTools::RotY(tmp_ry5) * RobotTools::RotZ(tmp_rz5);
			via_posrpy.col(6 * idx + 4).tail(3) = RobotTools::Tr2FixedZYX(temp_rot5);

			via_posrpy.col(6 * idx + 5).head(3) = temp_pos11;
			double tmp_rx6 = rx[5] + idx*step_rx[5];
			double tmp_ry6 = ry[5] + idx*step_ry[5];
			double tmp_rz6 = rz[5] + idx*step_rz[5];
			Matrix3d temp_rot6 = rot * RobotTools::RotX(tmp_rx6)*RobotTools::RotY(tmp_ry6) * RobotTools::RotZ(tmp_rz6);
			via_posrpy.col(6 * idx + 5).tail(3) = RobotTools::Tr2FixedZYX(temp_rot6);
		}
		return cycle_num;
	}

	int CircPlanner::InnerCylindricalBasin(MatrixXd& via_posrpy, MatrixXd* vision_pos,
		double rx[6], double ry[6], double rz[6], double dis_trans, double angle, double tap_theta)
	{
		Vector3d pos1, pos2, pos3, pos4, pos5, pos6, pos7, pos8, pos9, pos10;
		pos1 = vision_pos->col(0); pos2 = vision_pos->col(1); pos3 = vision_pos->col(2);pos4 = vision_pos->col(3); 
		pos5 = vision_pos->col(4); pos6 = vision_pos->col(5);
		pos7 = vision_pos->col(0); pos8 = vision_pos->col(1); pos9 = vision_pos->col(2); pos10 = vision_pos->col(3);
		Vector3d center; double radious, open_angle; Matrix3d rot;
		CalcArcInfo(center, radious, rot, open_angle, pos4, pos1, pos2);
		Vector3d vecp1p7 = pos7 - pos1;
		double length = vecp1p7.norm();
		int cycle_num = ceil(length / dis_trans);
		double step_size = length / (cycle_num);
		double step_rx[6], step_ry[6], step_rz[6];//no interp
		via_posrpy.resize(6, 6 * cycle_num);
		Vector3d p1p7 = pos7 - pos1; Vector3d p2p8 = pos8 - pos2; Vector3d p3p9 = pos9 - pos3; Vector3d p4p10 = pos10 - pos4;
		p1p7.normalize(); p2p8.normalize(); p3p9.normalize(); p4p10.normalize();
		for (int idx = 0; idx < cycle_num; idx++)
		{
			Vector3d temp_pos1;
			temp_pos1 = pos1 + step_size * p1p7 * idx;
			Vector3d temp_pos2;
			temp_pos2 = pos1 - step_size * p2p8 * idx;
			Vector3d temp_pos3;
			temp_pos3 = pos1 - step_size * p3p9 * idx;
			Vector3d temp_pos4;
			temp_pos4 = pos1 - step_size * p4p10 * idx;
			Vector3d temp_pos11;
			CalcArcPoint(temp_pos11, temp_pos1, temp_pos2, temp_pos3, tap_theta);
			Vector3d temp_pos12;
			CalcArcPoint(temp_pos12, temp_pos3, temp_pos4, temp_pos1, tap_theta);
			Vector3d temp_pos7;
			CalcArcPoint(temp_pos7, temp_pos3, temp_pos4, temp_pos1, pi + tap_theta - angle);
			Vector3d temp_pos8;
			CalcArcPoint(temp_pos8, temp_pos1, temp_pos2, temp_pos3, tap_theta + angle);
			via_posrpy.col(6 * idx).head(3) = temp_pos8;
			Matrix3d temp_rot1 = rot * RobotTools::RotX(rx[0])*RobotTools::RotY(ry[0]) * RobotTools::RotZ(rz[0]);
			via_posrpy.col(6 * idx).tail(3) = RobotTools::Tr2FixedZYX(temp_rot1);

			via_posrpy.col(6 * idx + 1).head(3) = temp_pos2;
			Matrix3d temp_rot2 = rot * RobotTools::RotX(rx[1])*RobotTools::RotY(ry[1]) * RobotTools::RotZ(rz[1]);
			via_posrpy.col(6 * idx + 1).tail(3) = RobotTools::Tr2FixedZYX(temp_rot2);

			via_posrpy.col(6 * idx + 2).head(3) = temp_pos12;
			Matrix3d temp_rot3 = rot * RobotTools::RotX(rx[2])*RobotTools::RotY(ry[2]) * RobotTools::RotZ(rz[2]);
			via_posrpy.col(6 * idx + 2).tail(3) = RobotTools::Tr2FixedZYX(temp_rot3);

			via_posrpy.col(6 * idx + 3).head(3) = temp_pos4;
			Matrix3d temp_rot4 = rot * RobotTools::RotX(rx[3])*RobotTools::RotY(ry[3]) * RobotTools::RotZ(rz[3]);
			via_posrpy.col(6 * idx + 3).tail(3) = RobotTools::Tr2FixedZYX(temp_rot4);

			via_posrpy.col(6 * idx + 4).head(3) = temp_pos7;
			Matrix3d temp_rot5 = rot * RobotTools::RotX(rx[4])*RobotTools::RotY(ry[4]) * RobotTools::RotZ(rz[4]);
			via_posrpy.col(6 * idx + 4).tail(3) = RobotTools::Tr2FixedZYX(temp_rot5);

			via_posrpy.col(6 * idx + 5).head(3) = temp_pos11;
			Matrix3d temp_rot6 = rot * RobotTools::RotX(rx[5])*RobotTools::RotY(ry[5]) * RobotTools::RotZ(rz[5]);
			via_posrpy.col(6 * idx + 5).tail(3) = RobotTools::Tr2FixedZYX(temp_rot6);
		}
		return cycle_num;
	}

	int CircPlanner::OuterCylindricalBasin(MatrixXd& via_posrpy, MatrixXd* vision_pos,
		double rx[3], double ry[3], double rz[3], double dis_trans, double height)
	{
		Vector3d pos1, pos2, pos3, pos4, pos5, pos6, pos7;
		pos1 = vision_pos->col(0); pos2 = vision_pos->col(1); pos3 = vision_pos->col(2);
		pos4 = vision_pos->col(3); pos5 = vision_pos->col(4); pos6 = vision_pos->col(5); pos7 = vision_pos->col(6);
		Vector3d center; double radious, open_angle; Matrix3d rot;
		CalcArcInfo(center, radious, rot, open_angle, pos4, pos1, pos2);
		int cycle_num = ceil(height / dis_trans);
		double step_size = height / (cycle_num);
		via_posrpy.resize(6, 3 * cycle_num);
		for (int idx = 0; idx < cycle_num; idx++)
		{
			Vector3d temp_pos5 = pos5 + Vector3d(0, 0, step_size*idx);
			via_posrpy.col(3 * idx).head(3) = temp_pos5;
			Matrix3d temp_rot1 = rot * RobotTools::RotX(rx[0])*RobotTools::RotY(ry[0]) * RobotTools::RotZ(rz[0]);
			via_posrpy.col(3 * idx).tail(3) = RobotTools::Tr2FixedZYX(temp_rot1);

			Vector3d temp_pos6 = pos6 + Vector3d(0, 0, step_size*idx);
			via_posrpy.col(3 * idx + 1).head(3) = temp_pos6;
			Matrix3d temp_rot2 = rot * RobotTools::RotX(rx[1])*RobotTools::RotY(ry[1]) * RobotTools::RotZ(rz[1]);
			via_posrpy.col(3 * idx + 1).tail(3) = RobotTools::Tr2FixedZYX(temp_rot2);

			Vector3d temp_pos7 = pos7 + Vector3d(0, 0, step_size*idx);
			via_posrpy.col(3 * idx + 2).head(3) = temp_pos7;
			Matrix3d temp_rot3 = rot * RobotTools::RotX(rx[2])*RobotTools::RotY(ry[2]) * RobotTools::RotZ(rz[2]);
			via_posrpy.col(3 * idx + 2).tail(3) = RobotTools::Tr2FixedZYX(temp_rot3);
		}
		return cycle_num;
	}

	int CircPlanner::InnerSphericalBasin(MatrixXd& via_posrpy, MatrixXd* vision_pos,
		double rx[6], double ry[6], double rz[6], double dis_angle, double angle, double tap_theta)
	{
		Vector3d pos1, pos2, pos3, pos4, pos5, pos6;
		pos1 = vision_pos->col(0); pos2 = vision_pos->col(1); pos3 = vision_pos->col(2);
		pos4 = vision_pos->col(3); pos5 = vision_pos->col(4); pos6 = vision_pos->col(5);
		Vector3d center; double radious, open_angle; Matrix3d rot;
		CalcArcInfo(center, radious, rot, open_angle, pos4, pos1, pos2);
		int cycle_num = ceil(pi / 2 / dis_angle);
		double step_size = pi / 2 / (cycle_num);
		double step_rx[6], step_ry[6], step_rz[6];
		for (int idx = 0; idx < 6; idx++)
		{
			step_rx[idx] = -rx[idx] / cycle_num;
			step_ry[idx] = -ry[idx] / cycle_num;
			step_rz[idx] = -rz[idx] / cycle_num;
		}
		via_posrpy.resize(6, 6 * cycle_num);
		for (int idx = 0; idx < cycle_num; idx++)
		{
			Vector3d temp_pos1;
			CalcArcPoint(temp_pos1, pos1, pos5, pos3, idx*step_size);
			Vector3d temp_pos2;
			CalcArcPoint(temp_pos2, pos2, pos5, pos4, idx*step_size);
			Vector3d temp_pos3;
			CalcArcPoint(temp_pos3, pos3, pos5, pos1, idx*step_size);
			Vector3d temp_pos4;
			CalcArcPoint(temp_pos4, pos4, pos5, pos2, idx*step_size);
			Vector3d temp_pos11;
			CalcArcPoint(temp_pos11, temp_pos1, temp_pos2, temp_pos3, tap_theta);
			Vector3d temp_pos12;
			CalcArcPoint(temp_pos12, temp_pos3, temp_pos4, temp_pos1, tap_theta);
			Vector3d temp_pos7;
			CalcArcPoint(temp_pos7, temp_pos3, temp_pos4, temp_pos1, pi + tap_theta - angle);
			Vector3d temp_pos8;
			CalcArcPoint(temp_pos8, temp_pos1, temp_pos2, temp_pos3, tap_theta + angle);
			via_posrpy.col(6 * idx).head(3) = temp_pos8;
			double tmp_rx1 = rx[0] + idx*step_rx[0];
			double tmp_ry1 = ry[0] + idx*step_ry[0];
			double tmp_rz1 = rz[0] + idx*step_rz[0];
			Matrix3d temp_rot1 = rot * RobotTools::RotX(tmp_rx1)*RobotTools::RotY(tmp_ry1) * RobotTools::RotZ(tmp_rz1);
			via_posrpy.col(6 * idx).tail(3) = RobotTools::Tr2FixedZYX(temp_rot1);

			via_posrpy.col(6 * idx + 1).head(3) = temp_pos2;
			double tmp_rx2 = rx[1] + idx*step_rx[1];
			double tmp_ry2 = ry[1] + idx*step_ry[1];
			double tmp_rz2 = rz[1] + idx*step_rz[1];
			Matrix3d temp_rot2 = rot * RobotTools::RotX(tmp_rx2)*RobotTools::RotY(tmp_ry2) * RobotTools::RotZ(tmp_rz2);
			via_posrpy.col(6 * idx + 1).tail(3) = RobotTools::Tr2FixedZYX(temp_rot2);

			via_posrpy.col(6 * idx + 2).head(3) = temp_pos12;
			double tmp_rx3 = rx[2] + idx*step_rx[2];
			double tmp_ry3 = ry[2] + idx*step_ry[2];
			double tmp_rz3 = rz[2] + idx*step_rz[2];
			Matrix3d temp_rot3 = rot * RobotTools::RotX(tmp_rx3)*RobotTools::RotY(tmp_ry3) * RobotTools::RotZ(tmp_rz3);
			via_posrpy.col(6 * idx + 2).tail(3) = RobotTools::Tr2FixedZYX(temp_rot3);

			via_posrpy.col(6 * idx + 3).head(3) = temp_pos4;
			double tmp_rx4 = rx[3] + idx*step_rx[3];
			double tmp_ry4 = ry[3] + idx*step_ry[3];
			double tmp_rz4 = rz[3] + idx*step_rz[3];
			Matrix3d temp_rot4 = rot * RobotTools::RotX(tmp_rx4)*RobotTools::RotY(tmp_ry4) * RobotTools::RotZ(tmp_rz4);
			via_posrpy.col(6 * idx + 3).tail(3) = RobotTools::Tr2FixedZYX(temp_rot4);

			via_posrpy.col(6 * idx + 4).head(3) = temp_pos7;
			double tmp_rx5 = rx[4] + idx*step_rx[4];
			double tmp_ry5 = ry[4] + idx*step_ry[4];
			double tmp_rz5 = rz[4] + idx*step_rz[4];
			Matrix3d temp_rot5 = rot * RobotTools::RotX(tmp_rx5)*RobotTools::RotY(tmp_ry5) * RobotTools::RotZ(tmp_rz5);
			via_posrpy.col(6 * idx + 4).tail(3) = RobotTools::Tr2FixedZYX(temp_rot5);

			via_posrpy.col(6 * idx + 5).head(3) = temp_pos11;
			double tmp_rx6 = rx[5] + idx*step_rx[5];
			double tmp_ry6 = ry[5] + idx*step_ry[5];
			double tmp_rz6 = rz[5] + idx*step_rz[5];
			Matrix3d temp_rot6 = rot * RobotTools::RotX(tmp_rx6)*RobotTools::RotY(tmp_ry6) * RobotTools::RotZ(tmp_rz6);
			via_posrpy.col(6 * idx + 5).tail(3) = RobotTools::Tr2FixedZYX(temp_rot6);
		}
		return cycle_num;
	}

	int CircPlanner::OuterSphericalBasin(MatrixXd& via_posrpy, MatrixXd* vision_pos,
		double rx[3], double ry[3], double rz[3], double dis_angle, double r1)
	{
		Vector3d pos1, pos2, pos3, pos4, pos5, pos6, pos7, pos8;
		pos1 = vision_pos->col(0); pos2 = vision_pos->col(1); pos3 = vision_pos->col(2);
		pos4 = vision_pos->col(3); pos5 = vision_pos->col(4); pos6 = vision_pos->col(5);
		pos7 = vision_pos->col(6); pos8 = vision_pos->col(7);
		Vector3d center,center1; double radious,radious1, open_angle,open_angle1; Matrix3d rot,rot1;
		CalcArcInfo(center, radious, rot, open_angle, pos4, pos1, pos2);
		CalcArcInfo(center1, radious1, rot1, open_angle1, pos4, pos5, pos2);
		double theta = acos((radious1 - r1) / (radious1 + r1));
		Vector3d p7c, p8c,p4c;
		p7c = pos7 - center; p8c = pos8 - center; p4c = pos4 - center;
		double cos_a = p4c.dot(p7c) / (p4c.norm()*p7c.norm());
		MathTools::LimitNum(-1, cos_a, 1);
		double alpha1 = acos(cos_a);
		Vector3d pos9;
		CalcArcPoint(pos9, pos4, pos1, pos2, alpha1 + pi);
		cos_a = p4c.dot(p8c) / (p4c.norm()*p8c.norm());
		MathTools::LimitNum(-1, cos_a, 1);
		double alpha2 = acos(cos_a);
		Vector3d pos10;
		CalcArcPoint(pos10, pos4, pos1, pos2, - alpha2 + pi);
		int cycle_num = ceil((pi / 2 - theta) / dis_angle);
		double step_size = (pi / 2 - theta) / cycle_num;
		via_posrpy.resize(6, 3 * cycle_num);
		for (int idx = 0; idx < cycle_num; idx++)
		{
			Vector3d temp_pos7;
			CalcArcPoint(temp_pos7, pos7, pos5, pos9, idx*step_size);
			via_posrpy.col(3 * idx).head(3) = temp_pos7;
			Matrix3d temp_rot1;
			if(alpha1<pi/2)
			{
				temp_rot1 = rot * RobotTools::RotX(rx[0])*RobotTools::RotY(ry[0] - idx*step_size) * RobotTools::RotZ(rz[0]);
			}			
			else if(alpha1>pi/2)
			{
				temp_rot1 = rot * RobotTools::RotX(rx[0])*RobotTools::RotY(ry[0] + idx*step_size) * RobotTools::RotZ(rz[0]);
			}				
			via_posrpy.col(3 * idx).tail(3) = RobotTools::Tr2FixedZYX(temp_rot1);

			Vector3d temp_pos4;
			if (alpha1 < pi / 2)
			{
				CalcArcPoint(temp_pos4, pos4, pos5, pos2, idx*step_size);
			}
			else if (alpha1 > pi / 2)
			{
				CalcArcPoint(temp_pos4, pos2, pos5, pos4, idx*step_size);
			}
			via_posrpy.col(3 * idx + 1).head(3) = temp_pos4;

			Matrix3d temp_rot2 = rot * RobotTools::RotX(rx[1])*RobotTools::RotY(ry[1]) * RobotTools::RotZ(rz[1]);
			via_posrpy.col(3 * idx + 1).tail(3) = RobotTools::Tr2FixedZYX(temp_rot2);

			Vector3d temp_pos8;
			CalcArcPoint(temp_pos8, pos8, pos5, pos10, idx*step_size);
			via_posrpy.col(3 * idx + 2).head(3) = temp_pos8;
			Matrix3d temp_rot3;
			if (alpha1<pi / 2)
			{
				temp_rot3 = rot * RobotTools::RotX(rx[2])*RobotTools::RotY(ry[2] - idx*step_size) * RobotTools::RotZ(rz[2]);
			}
			else if (alpha1>pi / 2)
			{
				temp_rot3 = rot * RobotTools::RotX(rx[2])*RobotTools::RotY(ry[2] + idx*step_size) * RobotTools::RotZ(rz[2]);
			}
			via_posrpy.col(3 * idx + 2).tail(3) = RobotTools::Tr2FixedZYX(temp_rot3);
		}
		return cycle_num;
	}

	void CircPlanner::UpperEdgeBasin(MatrixXd& via_posrpy, MatrixXd* vision_pos,
		double rx[5], double ry[5], double rz[5], double angle, double tap_theta)
	{
		Vector3d pos1, pos2, pos3, pos4, pos5, pos6;
		pos1 = vision_pos->col(0); pos2 = vision_pos->col(1); pos3 = vision_pos->col(2);
		pos4 = vision_pos->col(3); 
		Vector3d center; double radious, open_angle; Matrix3d rot;
		CalcArcInfo(center, radious, rot, open_angle, pos4, pos1, pos2);
		via_posrpy.resize(6, 7);
		Vector3d temp_pos5;
		CalcArcPoint(temp_pos5, pos1, pos2, pos3, tap_theta);
		Vector3d temp_pos6;
		CalcArcPoint(temp_pos6, pos3, pos4, pos1, tap_theta);
		Vector3d temp_pos7;
		CalcArcPoint(temp_pos7, pos3, pos4, pos1, pi + tap_theta - angle);
		Vector3d temp_pos8;
		CalcArcPoint(temp_pos8, pos1, pos2, pos3, tap_theta + angle);
			via_posrpy.col(0).head(3) = temp_pos5;
			Matrix3d temp_rot1 = rot * RobotTools::RotX(rx[0])*RobotTools::RotY(ry[0]) * RobotTools::RotZ(rz[0]);
			via_posrpy.col(0).tail(3) = RobotTools::Tr2FixedZYX(temp_rot1);

			via_posrpy.col(1).head(3) = temp_pos8;
			Matrix3d temp_rot2 = rot* RobotTools::RotX(rx[1])*RobotTools::RotY(ry[1]) * RobotTools::RotZ(rz[1]);
			via_posrpy.col(1).tail(3) = RobotTools::Tr2FixedZYX(temp_rot2);

			via_posrpy.col(2).head(3) = pos2;
			Matrix3d temp_rot3 = rot;
			via_posrpy.col(2).tail(3) = RobotTools::Tr2FixedZYX(temp_rot3);

			via_posrpy.col(3).head(3) = temp_pos6;
			Matrix3d temp_rot4 = rot * RobotTools::RotX(rx[2])*RobotTools::RotY(ry[2]) * RobotTools::RotZ(rz[2]);
			via_posrpy.col(3).tail(3) = RobotTools::Tr2FixedZYX(temp_rot4);

			via_posrpy.col(4).head(3) = pos4;
			Matrix3d temp_rot5 = rot;
			via_posrpy.col(4).tail(3) = RobotTools::Tr2FixedZYX(temp_rot5);

			via_posrpy.col(5).head(3) = temp_pos7;
			Matrix3d temp_rot6 = rot * RobotTools::RotX(rx[3])*RobotTools::RotY(ry[3]) * RobotTools::RotZ(rz[3]);
			via_posrpy.col(5).tail(3) = RobotTools::Tr2FixedZYX(temp_rot6);

			via_posrpy.col(6).head(3) = temp_pos5;//different rot
			Matrix3d temp_rot7 = rot * RobotTools::RotX(rx[4])*RobotTools::RotY(ry[4]) * RobotTools::RotZ(rz[4]);
			via_posrpy.col(6).tail(3) = RobotTools::Tr2FixedZYX(temp_rot7);
	}

	void CircPlanner::CalcArcInfo(Vector3d& center, double& radius, Matrix3d& rot, double& theta, Vector3d pos1, Vector3d pos2, Vector3d pos3)
	{
		Vector4d params1 = RobotTools::PointsCoplane(pos1, pos2, pos3);
		Vector4d params2 = RobotTools::RadiusEqual(pos1, pos2);
		Vector4d params3 = RobotTools::RadiusEqual(pos1, pos3);
		double a1 = params1(0), b1 = params1(1), c1 = params1(2), d1 = params1(3);
		double a2 = params2(0), b2 = params2(1), c2 = params2(2), d2 = params2(3);
		double a3 = params3(0), b3 = params3(1), c3 = params3(2), d3 = params3(3);
		center(0) = -(b1*c2*d3 - b1*c3*d2 - b2*c1*d3 + b2*c3*d1 + b3*c1*d2 - b3*c2*d1) /
			(a1*b2*c3 - a1*b3*c2 - a2*b1*c3 + a2*b3*c1 + a3*b1*c2 - a3*b2*c1);
		center(1) = (a1*c2*d3 - a1*c3*d2 - a2*c1*d3 + a2*c3*d1 + a3*c1*d2 - a3*c2*d1) /
			(a1*b2*c3 - a1*b3*c2 - a2*b1*c3 + a2*b3*c1 + a3*b1*c2 - a3*b2*c1);
		center(2) = -(a1*b2*d3 - a1*b3*d2 - a2*b1*d3 + a2*b3*d1 + a3*b1*d2 - a3*b2*d1) /
			(a1*b2*c3 - a1*b3*c2 - a2*b1*c3 + a2*b3*c1 + a3*b1*c2 - a3*b2*c1);
		Vector3d line_vec = pos3 - pos1;
		double line_length = line_vec.norm();
		Vector3d xr = pos1 - center;
		radius = xr.norm();
		Vector3d line_vec1 = pos1 - pos2;
		Vector3d line_vec2 = pos3 - pos2;
		double cos_a = line_vec1.dot(line_vec2) / (line_vec1.norm()*line_vec2.norm());
		MathTools::LimitNum(-1, cos_a, 1);
		theta = acos(1 - 2 * (1 - cos_a) * (1 + cos_a));
		if (cos_a > 0)
			theta = 2 * pi - theta;
		Vector3d zr(a1, b1, c1);
		Vector3d yr = zr.cross(xr);
		xr.normalize(); yr.normalize(); zr.normalize();
		rot << xr, yr, zr;
	}

	void CircPlanner::CalcArcPoint(Vector3d& pos, Vector3d pos1, Vector3d pos2, Vector3d pos3, double angle)
	{
		Vector3d center; double radius, theta; Matrix3d rot;
		CalcArcInfo(center, radius, rot, theta, pos1, pos2, pos3);
		Vector3d parc = Vector3d::Zero();
		parc(0) = radius*cos(angle);
		parc(1) = radius*sin(angle);
		pos = center + rot*parc;
	}

	int EllipsePlanner::InnerEllipseBasin(MatrixXd& via_posrpy, MatrixXd* vision_pos,
		double rx[6], double ry[6], double rz[6], double dis_angle, double angle, double tap_theta)
	{
		Vector3d pos1, pos2, pos3, pos4, pos5, pos6;
		pos1 = vision_pos->col(0); pos2 = vision_pos->col(1); pos3 = vision_pos->col(2);
		pos4 = vision_pos->col(3); pos5 = vision_pos->col(4); pos6 = vision_pos->col(5);
		Vector3d center; double a, b; Matrix3d rot;
		InitEllipseInfo(center, a, b, rot, pos4, pos1, pos2);
		int cycle_num = ceil(pi / 2 / dis_angle);
		double step_size = pi / 2 / (cycle_num);
		double step_rx[6], step_ry[6], step_rz[6];
		for (int idx = 0; idx < 6; idx++)
		{
			step_rx[idx] = -rx[idx] / cycle_num;
			step_ry[idx] = -ry[idx] / cycle_num;
			step_rz[idx] = -rz[idx] / cycle_num;
		}
		via_posrpy.resize(6, 6 * cycle_num);
		for (int idx = 0; idx < cycle_num; idx++)
		{
			Vector3d temp_pos1;
			CalcEllipsePoint(temp_pos1, pos1, pos5, pos3, idx*step_size);
			Vector3d temp_pos2;
			CalcEllipsePoint(temp_pos2, pos2, pos5, pos4, idx*step_size);
			Vector3d temp_pos3;
			CalcEllipsePoint(temp_pos3, pos3, pos5, pos1, idx*step_size);
			Vector3d temp_pos4;
			CalcEllipsePoint(temp_pos4, pos4, pos5, pos2, idx*step_size);
			Vector3d temp_pos11;
			CalcEllipsePoint(temp_pos11, temp_pos1, temp_pos2, temp_pos3, tap_theta);
			Vector3d temp_pos8;
			CalcEllipsePoint(temp_pos8, temp_pos1, temp_pos2, temp_pos3, tap_theta + angle);
			via_posrpy.col(6 * idx).head(3) = temp_pos1;
			double tmp_rx1 = rx[0] + idx*step_rx[0];
			double tmp_ry1 = ry[0] + idx*step_ry[0];
			double tmp_rz1 = rz[0] + idx*step_rz[0];
			Matrix3d temp_rot1 = rot * RobotTools::RotX(tmp_rx1)*RobotTools::RotY(tmp_ry1) * RobotTools::RotZ(tmp_rz1);
			via_posrpy.col(6 * idx).tail(3) = RobotTools::Tr2FixedZYX(temp_rot1);

			via_posrpy.col(6 * idx + 1).head(3) = temp_pos2;
			double tmp_rx2 = rx[1] + idx*step_rx[1];
			double tmp_ry2 = ry[1] + idx*step_ry[1];
			double tmp_rz2 = rz[1] + idx*step_rz[1];
			Matrix3d temp_rot2 = rot * RobotTools::RotX(tmp_rx2)*RobotTools::RotY(tmp_ry2) * RobotTools::RotZ(tmp_rz2);
			via_posrpy.col(6 * idx + 1).tail(3) = RobotTools::Tr2FixedZYX(temp_rot2);

			via_posrpy.col(6 * idx + 2).head(3) = temp_pos3;
			double tmp_rx3 = rx[2] + idx*step_rx[2];
			double tmp_ry3 = ry[2] + idx*step_ry[2];
			double tmp_rz3 = rz[2] + idx*step_rz[2];
			Matrix3d temp_rot3 = rot * RobotTools::RotX(tmp_rx3)*RobotTools::RotY(tmp_ry3) * RobotTools::RotZ(tmp_rz3);
			via_posrpy.col(6 * idx + 2).tail(3) = RobotTools::Tr2FixedZYX(temp_rot3);

			via_posrpy.col(6 * idx + 3).head(3) = temp_pos4;
			double tmp_rx4 = rx[3] + idx*step_rx[3];
			double tmp_ry4 = ry[3] + idx*step_ry[3];
			double tmp_rz4 = rz[3] + idx*step_rz[3];
			Matrix3d temp_rot4 = rot * RobotTools::RotX(tmp_rx4)*RobotTools::RotY(tmp_ry4) * RobotTools::RotZ(tmp_rz4);
			via_posrpy.col(6 * idx + 3).tail(3) = RobotTools::Tr2FixedZYX(temp_rot4);

			via_posrpy.col(6 * idx + 4).head(3) = temp_pos11;
			double tmp_rx5 = rx[4] + idx*step_rx[4];
			double tmp_ry5 = ry[4] + idx*step_ry[4];
			double tmp_rz5 = rz[4] + idx*step_rz[4];
			Matrix3d temp_rot5 = rot * RobotTools::RotX(tmp_rx5)*RobotTools::RotY(tmp_ry5) * RobotTools::RotZ(tmp_rz5);
			via_posrpy.col(6 * idx + 4).tail(3) = RobotTools::Tr2FixedZYX(temp_rot5);

			via_posrpy.col(6 * idx + 5).head(3) = temp_pos8;
			double tmp_rx6 = rx[5] + idx*step_rx[5];
			double tmp_ry6 = ry[5] + idx*step_ry[5];
			double tmp_rz6 = rz[5] + idx*step_rz[5];
			Matrix3d temp_rot6 = rot * RobotTools::RotX(tmp_rx6)*RobotTools::RotY(tmp_ry6) * RobotTools::RotZ(tmp_rz6);
			via_posrpy.col(6 * idx + 5).tail(3) = RobotTools::Tr2FixedZYX(temp_rot6);
		}
		return cycle_num;
	}

	int EllipsePlanner::OuterEllipseBasin(MatrixXd& via_posrpy, MatrixXd* vision_pos,
		double rx[2], double ry[2], double rz[2], double dis_trans)
	{
		Vector3d pos1, pos2, pos3, pos4, pos5, pos6, pos7, pos8;
		pos1 = vision_pos->col(0); pos2 = vision_pos->col(1); pos3 = vision_pos->col(2);
		pos4 = vision_pos->col(3); pos5 = vision_pos->col(4); pos6 = vision_pos->col(5);
		pos7 = vision_pos->col(6); pos8 = vision_pos->col(7);
		Vector3d center; double a, b; Matrix3d rot;
		InitEllipseInfo(center, a, b, rot, pos4, pos1, pos2);
		Vector3d p15 = pos1 - pos5;
		double length = p15.norm();
		Vector3d p26 = pos2 - pos6;
		Vector3d p37 = pos3 - pos7;
		Vector3d p48 = pos4 - pos8;
		p15.normalize(); p26.normalize(); p37.normalize(); p48.normalize();
		int cycle_num = ceil(length / dis_trans);
		double step_size = length / cycle_num;
		via_posrpy.resize(6, 4 * cycle_num);
		for (int idx = 0; idx < cycle_num; idx++)
		{
			Vector3d temp_pos1,temp_pos2,temp_pos3,temp_pos4;
			temp_pos1 = pos5 + idx * step_size * p15;
			temp_pos2 = pos6 + idx * step_size * p26;
			temp_pos3 = pos7 + idx * step_size * p37;
			temp_pos4 = pos8 + idx * step_size * p48;
			via_posrpy.col(4 * idx).head(3) = temp_pos1;
			via_posrpy.col(4 * idx + 1).head(3) = temp_pos2;
			via_posrpy.col(4 * idx + 2).head(3) = temp_pos3;
			via_posrpy.col(4 * idx + 3).head(3) = temp_pos4;
			Matrix3d temp_rot1;
			temp_rot1 = rot * RobotTools::RotX(rx[0])*RobotTools::RotY(ry[0]) * RobotTools::RotZ(rz[0]);
			via_posrpy.col(4 * idx).tail(3) = RobotTools::Tr2FixedZYX(temp_rot1);
			via_posrpy.col(4 * idx + 1).tail(3) = RobotTools::Tr2FixedZYX(rot);
			Matrix3d temp_rot2;
			temp_rot2 = rot * RobotTools::RotX(rx[1])*RobotTools::RotY(ry[1]) * RobotTools::RotZ(rz[1]);
			via_posrpy.col(4 * idx + 2).tail(3) = RobotTools::Tr2FixedZYX(temp_rot2);
			via_posrpy.col(4 * idx + 3).tail(3) = RobotTools::Tr2FixedZYX(rot);
		}
		return cycle_num;
	}

	int EllipsePlanner::UpperEdgeBasin(MatrixXd& via_posrpy, MatrixXd* vision_pos,
		double rx[6], double ry[6], double rz[6], double dis_angle, double angle, double tap_theta)
	{
		return 0;
	}

	void EllipsePlanner::CalcEllipsePoint(Vector3d& pos, Vector3d pos1, Vector3d pos2, Vector3d pos3, double angle)
	{
		Vector3d center; double a, b; Matrix3d rot;
		InitEllipseInfo(center, a, b, rot, pos1, pos2, pos3);
		Vector3d parc = Vector3d::Zero();
		parc(0) = b*cos(angle);
		parc(1) = a*sin(angle);
		pos = center + rot*parc;
	}

	void EllipsePlanner::InitEllipseInfo(Vector3d& center, double& a, double& b, Matrix3d& rot,
		Vector3d pos1, Vector3d pos2, Vector3d pos3)
	{
		center = (pos1 + pos3) / 2;
		Vector3d vec_a = pos2 - center;
		Vector3d vec_b = pos1 - center;
		a = vec_a.norm();
		b = vec_b.norm();
		Vector3d zr = vec_b.cross(vec_a);
		zr.normalize();
		vec_b.normalize();
		Vector3d yr = zr.cross(vec_b);
		yr.normalize();
		rot << vec_b, yr, zr;
	}
}


namespace PlanTools
{
	Matrix3d CalcPlaneRot(Vector3d center, Vector3d norm_vec)
	{
		//plane function:Ax+By+Cz+D=0
		double D = -norm_vec.dot(center);
		Vector3d p1;
		p1(2) = center(2);
		p1(0) = center(0)+1;
		p1(1) = (-norm_vec(0)*p1(0)-norm_vec(2)*p1(2)-D)/norm_vec(1);
		Vector3d n, o, a;
		a = norm_vec.normalized();
		n = p1-center;
		n.normalize();
		o = a.cross(n);
		Matrix3d rot;
		rot<<n, o, a;

		return rot;
	}

	void CalcBasinPath(MatrixXd& in_via_posrpy, MatrixXd& out_via_posrpy,
					MatrixXd* vision_pos, double ang, double interval)
	{
		/*
		p3\		/ p4
		|	\p5/	|
		| ---------|
		p2---p6----p1
		p9\		/ p10
		|	\p11/	|
		| -----------|
		p8----------p7
		*/
		Vector3d pos1, pos2, pos3, pos4, pos5, pos6, pos7, pos8, pos9, pos10, pos11;
		//neiquandian
		pos1 = vision_pos->col(0); pos2 = vision_pos->col(1); pos3 = vision_pos->col(2);
		pos4 = vision_pos->col(3);
		Vector3d x0 = pos4-pos3;
		Vector3d y0 = pos2-pos3;
		x0.normalize();
		y0.normalize();
		Vector3d z0 = x0.cross(y0);
		z0.normalize();
		y0 = z0.cross(x0);
		Matrix3d rot_basin;
		rot_basin<<x0, y0, z0;
		pos5 = vision_pos->col(4);//(pos3 + pos4) / 2 + rot_basin*Vector3d(0, 0.035, 0);
		pos6 = (pos1+pos2)/2;
		//waiquandian
		//            Vector3d parc1, parc2, parc3, parc4;
		//        double dx = 0.04; double dy = 0.04; double dz = 0.07;
		//            parc1 << dx, dy, -dz;
		//            parc2 << -dx, dy, -dz;
		//            parc3 << -dx, -dy, -dz;
		//            parc4 << dx, -dy, -dz;
		pos7 = vision_pos->col(5);//pos1 + rot_basin*parc1;
		pos8 = vision_pos->col(6);//pos2 + rot_basin*parc2;
		pos9 = vision_pos->col(7);//pos3 + rot_basin*parc3;
		pos10 = vision_pos->col(8);//pos4 + rot_basin*parc4;
		double dz = pos7.z()-pos1.z();
		pos11 = pos5+rot_basin*Vector3d(0, 0, -dz);

		Vector3d vec_p5p4 = pos5-pos4;
		Vector3d vec_p6p1 = pos6-pos1;
		vec_p5p4.normalize();
		vec_p6p1.normalize();
		double len1 = MathTools::Norm(pos6-pos1);
		double len2 = MathTools::Norm(pos5-pos4);
		int cycle_num1 = (int)ceil(len1/interval);
		double step_size1 = len1/(cycle_num1);
		double step_size2 = len2/(cycle_num1);

		Vector3d vec_p3p5 = pos3-pos5;
		Vector3d vec_p2p6 = pos2-pos6;
		vec_p3p5.normalize();
		vec_p2p6.normalize();
		double len4 = MathTools::Norm(pos3-pos5);
		double len3 = MathTools::Norm(pos2-pos6);
		int cycle_num2 = (int)ceil(len3/interval);
		double step_size3 = len3/(cycle_num2);
		double step_size4 = len4/(cycle_num2);

		Matrix3d rot1, rot2;
		rot1 = rot_basin*RobotTools::RotX(ang);
		Vector3d rpy = RobotTools::Tr2FixedZYX(rot_basin);
		Vector3d rpy1 = RobotTools::Tr2FixedZYX(rot1);
		in_via_posrpy.resize(6, 2*(cycle_num1+1)+2*(cycle_num2+1)-2);
		for (int idx = 0; idx<cycle_num1+1; idx++)
		{
			Vector3d temp_pos1 = pos4+step_size2*vec_p5p4*idx;
			Vector3d temp_pos2 = pos1+step_size1*vec_p6p1*idx;
			in_via_posrpy.col(2*idx)<<temp_pos1, rpy1;
			in_via_posrpy.col(2*idx+1)<<temp_pos2, rpy;
		}
		for (int idx = 1; idx<cycle_num2+1; idx++)
		{
			Vector3d temp_pos1 = pos5+step_size4*vec_p3p5*idx;
			Vector3d temp_pos2 = pos6+step_size3*vec_p2p6*idx;
			in_via_posrpy.col(2*idx-1+2*cycle_num1+1)<<temp_pos1, rpy1;
			in_via_posrpy.col(2*idx+2*cycle_num1+1)<<temp_pos2, rpy;
		}

		out_via_posrpy.resize(6, 6+6);
		out_via_posrpy.col(0)<<pos2, rpy;
		out_via_posrpy.col(1)<<pos3, rpy1;
		out_via_posrpy.col(2)<<pos5, rpy1;
		out_via_posrpy.col(3)<<pos4, rpy1;
		out_via_posrpy.col(4)<<pos1, rpy;
		out_via_posrpy.col(5)<<pos2, rpy;

		out_via_posrpy.col(6)<<pos8, rpy;
		out_via_posrpy.col(7)<<pos9, rpy1;
		out_via_posrpy.col(8)<<pos11, rpy1;
		out_via_posrpy.col(9)<<pos10, rpy1;
		out_via_posrpy.col(10)<<pos7, rpy;
		out_via_posrpy.col(11)<<pos8, rpy;
	}

	void CalcToiletPath(MatrixXd& via_posrpy, MatrixXd* vision_pos, double theta)
	{
		int npos = static_cast<int>(vision_pos->cols());
		via_posrpy.resize(6, npos+1);
		Vector3d center;
		center(0) = vision_pos->row(0).mean();
		center(1) = vision_pos->row(1).mean();
		center(2) = vision_pos->row(2).mean();
		Vector3d x0, y0, z0;
		Matrix3d rot_mat;
		x0<<0, 1, 0;
		y0<<1, 0, 0;
		z0<<0, 0, -1;
		rot_mat<<x0, y0, z0;
		Vector3d rpy = RobotTools::Tr2FixedZYX(rot_mat);
		via_posrpy.col(0)<<center, rpy;
		//double theta = 20*pi/180;//angle of vertical plane
		for (int idx = 0; idx<npos; idx++)
		{
			Vector3d pos_tmp = vision_pos->col(idx);
			double len = MathTools::Norm(center-pos_tmp);
			Vector3d pos_high = center;
			pos_high(2) = center(2)+len/tan(theta);
			z0 = pos_tmp-pos_high;
			z0.normalize();
			y0 = MathTools::CalcXZPlaneIntersection(z0, pos_high);
			y0.normalize();
			x0 = y0.cross(z0);
			rot_mat<<x0, y0, z0;
			rpy = RobotTools::Tr2FixedZYX(rot_mat);
			via_posrpy.col(idx+1)<<pos_tmp, rpy;
		}
	}

	void CalcTablePath(MatrixXd& via_posrpy, MatrixXd* vision_pos, double theta, double interval)
	{
		//double theta = 40*pi/180;//angle of horizontal plane
		Vector3d origin = 0.5*(vision_pos->col(0)+vision_pos->col(1));
		double len = MathTools::Norm(origin-vision_pos->col(2));
		origin(2) = origin(2)+len*tan(theta);
		//double interval = 0.02;
		Vector3d step_vec1 = vision_pos->col(2)-vision_pos->col(3);
		Vector3d step_vec2 = vision_pos->col(1)-vision_pos->col(0);
		Vector3d start_pos1 = vision_pos->col(3);
		Vector3d start_pos2 = vision_pos->col(0);
		int cycle_num = static_cast<int>(round(step_vec1.norm()/interval))+1;
		double step_size = step_vec1.norm()/(cycle_num-1);
		step_vec1.normalize();
		step_vec2.normalize();
		via_posrpy.resize(6, 2*cycle_num);
		for (int idx = 0; idx<cycle_num; idx++)
		{
			Vector3d pos1 = start_pos1+idx*step_size*step_vec1;
			Vector3d pos2 = start_pos2+idx*step_size*step_vec2;
			Vector3d z0 = pos1-origin;
			Vector3d vec1 = -z0;
			Vector3d vec2 = 0.5*(vision_pos->col(0)+vision_pos->col(1))-pos1;
			Vector3d x0 = vec1.cross(vec2);
			Vector3d y0 = z0.cross(x0);
			Matrix3d rot_mat;
			rot_mat<<x0.normalized(), y0.normalized(), z0.normalized();
			Vector3d rpy = RobotTools::Tr2FixedZYX(rot_mat);
			via_posrpy.col(2*idx)<<pos1, rpy;
			via_posrpy.col(2*idx+1)<<pos2, rpy;
		}
	}

	void PlanToiletInlierPath(MatrixXd& via_posrpy, MatrixXd* vision_pos, double slant, string camera_ori)
	{
		int np = (int)(vision_pos->cols())-1;
		Vector3d end_pos = vision_pos->rightCols(1);
		Vector3d pos_trans;
		pos_trans(0) = vision_pos->leftCols(np).row(0).mean();
		pos_trans(1) = vision_pos->leftCols(np).row(1).mean();
		pos_trans(2) = vision_pos->leftCols(np).row(2).mean();
		via_posrpy.resize(6, 3*np+2);
		via_posrpy.col(0).head(3) = pos_trans;
		via_posrpy.rightCols(1).topRows(3) = pos_trans;
		for (int idx = 0; idx<np; idx++)
		{
			via_posrpy.col(3*idx+1).head(3) = vision_pos->col(idx);
			via_posrpy.col(3*idx+2).head(3) = end_pos;
			via_posrpy.col(3*idx+3).head(3) = vision_pos->col(idx);
		}
		Matrix3d rot_transform, rot_rpy;
		if (camera_ori=="front")
		{
			rot_transform<<0, 1, 0, 1, 0, 0, 0, 0, -1;
			rot_rpy = rot_transform*RobotTools::RotX(-slant);
		}
		else if (camera_ori=="back")
		{
			rot_transform<<0, -1, 0, -1, 0, 0, 0, 0, -1;
			rot_rpy = rot_transform*RobotTools::RotX(slant);
		}
		else if (camera_ori=="left")
		{
			rot_transform<<-1, 0, 0, 0, 1, 0, 0, 0, -1;
			rot_rpy = rot_transform*RobotTools::RotY(-slant);
		}
		else if (camera_ori=="right")
		{
			rot_transform<<1, 0, 0, 0, -1, 0, 0, 0, -1;
			rot_rpy = rot_transform*RobotTools::RotY(slant);
		}
		Vector3d rpy = RobotTools::Tr2FixedZYX(rot_rpy);
		for (int idx = 0; idx<(int)(via_posrpy.cols()); idx++)
			via_posrpy.col(idx).tail(3) = rpy;
	}

	void PlanToiletlidPath(MatrixXd& via_posrpy, MatrixXd* vision_pos, double arc_angle,
							double point_angle, double slant_angle, double dis_trans)
	{
		//the 3rd point is the mark point: posC
		//rotation axis is the 2nd point to 1st: posB->posA
		Vector3d posA = vision_pos->col(0);
		Vector3d posB = vision_pos->col(1);
		Vector3d posC = vision_pos->col(2);
		double mark_sign = fabs(posC(1)-posB(1))<fabs(posC(1)-posA(1)) ? 1 : -1;
		//calculate the 3 points in arc
		Vector3d z0_axis = posA-posB;
		z0_axis.normalize();
		double radius = MathTools::Cross(z0_axis,posC-posB).norm();
		double scale = posC.dot(z0_axis)-posB.dot(z0_axis);
		Vector3d center = posB+scale*z0_axis;
		Vector3d x0_axis = posC-center;
		x0_axis.normalize();
		Vector3d y0_axis = z0_axis.cross(x0_axis);
		Matrix3d rot_plane;
		rot_plane << x0_axis,y0_axis,z0_axis;

		via_posrpy.resize(6,4);
		via_posrpy.col(0).head(3) = posC-dis_trans*z0_axis*mark_sign;
		via_posrpy.col(1).head(3) = posC;
		Vector3d pos_tmp;
		pos_tmp(0) = radius*cos(arc_angle/2);
		pos_tmp(1) = radius*sin(arc_angle/2);
		pos_tmp(2) = 0;
		pos_tmp = center+rot_plane*pos_tmp;
		via_posrpy.col(2).head(3) = pos_tmp+0.5*dis_trans*z0_axis*mark_sign;
		pos_tmp(0) = radius*cos(arc_angle);
		pos_tmp(1) = radius*sin(arc_angle);
		pos_tmp(2) = 0;
		pos_tmp = center+rot_plane*pos_tmp;
		via_posrpy.col(3).head(3) = pos_tmp+z0_axis*dis_trans*mark_sign;
		//calculate orientation of the 3 points in arc
		Matrix3d rot_rpy;
		Vector3d rpy1,rpy2;
		if(arc_angle>0)
		{
			rot_rpy = rot_plane*RobotTools::RotY(point_angle)*RobotTools::RotX(slant_angle);
			rpy1 = RobotTools::Tr2FixedZYX(rot_rpy);
			Vector3d rpy_z0 = z0_axis;
			Vector3d rpy_x0 = pos_tmp-center;
			rpy_x0.normalize();
			Vector3d rpy_y0 = rpy_z0.cross(rpy_x0);
			rot_rpy << rpy_x0,rpy_y0,rpy_z0;
			rot_rpy = rot_rpy*RobotTools::RotY(-pi/2)*RobotTools::RotX(-pi/2);
			//rpy2 = RobotTools::Tr2FixedZYX(rot_rpy);
			rpy2 = rpy1;
		}
		else
		{
			rot_rpy = rot_plane*RobotTools::RotY(point_angle)*RobotTools::RotZ(slant_angle);
			rpy1 = RobotTools::Tr2FixedZYX(rot_rpy);
			Vector3d rpy_z0 = z0_axis;
			Vector3d rpy_y0 = center-pos_tmp;
			rpy_y0.normalize();
			Vector3d rpy_x0 = rpy_y0.cross(rpy_z0);
			rot_rpy << rpy_x0,rpy_y0,rpy_z0;
			rot_rpy = rot_rpy*RobotTools::RotY(point_angle)*RobotTools::RotX(slant_angle);
			//rpy2 = RobotTools::Tr2FixedZYX(rot_rpy);
			rpy2 = rpy1;
		}
		via_posrpy.col(0).tail(3) = rpy1;
		via_posrpy.col(1).tail(3) = rpy1;
		via_posrpy.col(2).tail(3) = rpy1;
		via_posrpy.col(3).tail(3) = rpy2;
	}

	void CalcMirrorPath_Line(MatrixXd& via_posrpy, MatrixXd* corner_pos,
		double lenscraper, double inc_ang,
		double dis_trans, std::string camera_ori)
	{
		Vector3d x0_mirror = corner_pos->col(3)-corner_pos->col(0);
		Vector3d y0_mirror = corner_pos->col(1)-corner_pos->col(0);
		x0_mirror.normalize();
		y0_mirror.normalize();
		Vector3d z0_mirror = x0_mirror.cross(y0_mirror);
		z0_mirror.normalize();
		Matrix3d rot_mirror;
		rot_mirror<<x0_mirror, y0_mirror, z0_mirror;

		MatrixXd corner_pos_new;
		corner_pos_new.resize(3, 4);
		corner_pos_new.col(0) = corner_pos->col(0)+lenscraper/2*y0_mirror;
		corner_pos_new.col(1) = corner_pos->col(1)-lenscraper/2*y0_mirror;
		corner_pos_new.col(2) = corner_pos->col(2)-lenscraper/2*y0_mirror;
		corner_pos_new.col(3) = corner_pos->col(3)+lenscraper/2*y0_mirror;

		double width_target = MathTools::Norm(corner_pos_new.col(0)-corner_pos_new.col(1));
		int cycle_num = (int)ceil(width_target/lenscraper)+1;
		double step_size = width_target/(cycle_num-1);
		Vector3d start_pos1 = corner_pos_new.col(3);
		Vector3d start_pos2 = corner_pos_new.col(0);
		Vector3d trans_vec = z0_mirror*dis_trans;
		Matrix3d rot_transform;
		if (camera_ori=="top")
			rot_transform<<0, 0, 1,
			1, 0, 0,
			0, 1, 0;
		else if (camera_ori=="left")
			rot_transform<<0, 0, 1,
			0, 1, 0,
			-1, 0, 0;
		else if (camera_ori=="down")
			rot_transform<<0, 0, 1,
			-1, 0, 0,
			0, -1, 0;
		else if (camera_ori=="right")
			rot_transform<<0, 0, 1,
			0, -1, 0,
			1, 0, 0;
		else
			rot_transform.setIdentity();
		via_posrpy.resize(6, 4*cycle_num);
		for (int idx = 0; idx<cycle_num; idx++)
		{
			Vector3d pos2 = start_pos1+idx*step_size*y0_mirror;
			Vector3d pos1 = pos2+trans_vec;
			Vector3d pos3 = start_pos2+idx*step_size*y0_mirror;
			Vector3d pos4 = pos3+trans_vec;
			Matrix3d rot_tool = rot_mirror*RobotTools::RotY(inc_ang)*rot_transform;
			Vector3d rpy = RobotTools::Tr2FixedZYX(rot_tool);
			via_posrpy.col(4*idx)<<pos1, rpy;
			via_posrpy.col(4*idx+1)<<pos2, rpy;
			via_posrpy.col(4*idx+2)<<pos3, rpy;
			via_posrpy.col(4*idx+3)<<pos4, rpy;
		}
	}

	void CalcMirrorPath(MatrixXd& via_posrpy, MatrixXd* corner_pos, double lenscraper, double slant_ang, double inc_ang)
	{
		Vector3d x0_mirror = corner_pos->col(3)-corner_pos->col(0);
		Vector3d y0_mirror = corner_pos->col(1)-corner_pos->col(0);
		x0_mirror.normalize();
		y0_mirror.normalize();
		Vector3d z0_mirror = x0_mirror.cross(y0_mirror);
		z0_mirror.normalize();
		Matrix3d rot_mirror;
		rot_mirror<<x0_mirror, y0_mirror, z0_mirror;

		MatrixXd corner_pos_new;
		corner_pos_new.resize(3, 4);
		corner_pos_new.col(0) = corner_pos->col(0)+lenscraper/2*x0_mirror+lenscraper/2*y0_mirror;
		corner_pos_new.col(1) = corner_pos->col(1)+lenscraper/2*x0_mirror-lenscraper/2*y0_mirror;
		corner_pos_new.col(2) = corner_pos->col(2)-lenscraper/2*x0_mirror-lenscraper/2*y0_mirror;
		corner_pos_new.col(3) = corner_pos->col(3)-lenscraper/2*x0_mirror+lenscraper/2*y0_mirror;

		double height_target = MathTools::Norm(corner_pos_new.col(1)-corner_pos_new.col(2));
		int cycle_num = (int)ceil(height_target/lenscraper)+1;
		double step_size = height_target/(cycle_num-1);
		Vector3d start_pos1 = corner_pos_new.col(2);
		Vector3d start_pos2 = corner_pos_new.col(3);
		Matrix3d rot_transform;
		rot_transform<<1, 0, 0,
			0, -1, 0,
			0, 0, -1;
		via_posrpy.resize(6, 2*cycle_num);
		for (int idx = 0; idx<cycle_num; idx++)
		{
			Vector3d pos1, pos2;
			Matrix3d rot_tool;
			if (idx%2==0)
			{
				pos1 = start_pos1+idx*step_size*(-x0_mirror);
				pos2 = start_pos2+idx*step_size*(-x0_mirror);
				rot_tool = rot_mirror*RobotTools::RotZ(pi-slant_ang)
					*RobotTools::RotX(-inc_ang)*rot_transform;
			}
			else
			{
				pos1 = start_pos2+idx*step_size*(-x0_mirror);
				pos2 = start_pos1+idx*step_size*(-x0_mirror);
				rot_tool = rot_mirror*RobotTools::RotZ(slant_ang)
					*RobotTools::RotX(-inc_ang)*rot_transform;
			}
			Vector3d rpy = RobotTools::Tr2FixedZYX(rot_tool);
			via_posrpy.col(2*idx)<<pos1, rpy;
			via_posrpy.col(2*idx+1)<<pos2, rpy;
		}
	}

	int CalcCircleMirrorPath(MatrixXd& circle_via_posrpy, MatrixXd& line_via_posrpy, MatrixXd* vision_pos, double lenscraper, double inc_ang)
	{
		Vector3d center, pos1, pos2, pos3;
		double radius, theta;
		Matrix3d rot;
		pos1 = vision_pos->col(0); pos2 = vision_pos->col(1); pos3 = vision_pos->col(2);
		CalcArcInfo(center, radius, rot, theta, pos1, pos2, pos3);
		double new_radius = radius-lenscraper/2;
		int cycle_num = (int)ceil(new_radius/lenscraper);
		double step_size = new_radius/(cycle_num);
		Matrix3d temp_rot1, temp_rot2, temp_rot3;
		temp_rot1 = rot*RobotTools::RotZ(-5*pi/6)*RobotTools::RotX(-inc_ang);
		temp_rot2 = rot*RobotTools::RotZ(-pi/2)*RobotTools::RotX(-inc_ang);
		temp_rot3 = rot*RobotTools::RotZ(-pi/6)*RobotTools::RotX(-inc_ang);
		circle_via_posrpy.resize(6, 6*cycle_num);
		line_via_posrpy.resize(6, 4*cycle_num+2);
		for (int idx = 0; idx < cycle_num; idx++)
		{
			double temp_radius = new_radius-step_size*idx;
			Vector3d parc1, parc2, parc3, parc4, parc5, parc6;
			parc1<<temp_radius*cos(0), temp_radius*sin(0), 0;
			parc2<<temp_radius*cos(pi/2), temp_radius*sin(pi/2), 0;
			parc3<<temp_radius*cos(pi), temp_radius*sin(pi), 0;
			parc4<<temp_radius*cos(-pi/2), temp_radius*sin(-pi/2), 0;
			parc5<<temp_radius*cos(0), temp_radius*sin(0), -0.02;
			parc6<<temp_radius*cos(pi), temp_radius*sin(pi), -0.02;

			circle_via_posrpy.col(6*idx).head(3) = center+rot*parc1;
			circle_via_posrpy.col(6*idx+1).head(3) = center+rot*parc2;
			circle_via_posrpy.col(6*idx+2).head(3) = center+rot*parc3;

			circle_via_posrpy.col(6*idx+3).head(3) = center+rot*parc1;
			circle_via_posrpy.col(6*idx+4).head(3) = center+rot*parc4;
			circle_via_posrpy.col(6*idx+5).head(3) = center+rot*parc3;

			circle_via_posrpy.col(6*idx).tail(3) = RobotTools::Tr2FixedZYX(temp_rot1);
			circle_via_posrpy.col(6*idx+1).tail(3) = RobotTools::Tr2FixedZYX(temp_rot2);
			circle_via_posrpy.col(6*idx+2).tail(3) = RobotTools::Tr2FixedZYX(temp_rot3);

			circle_via_posrpy.col(6*idx+3).tail(3) = RobotTools::Tr2FixedZYX(temp_rot3);
			circle_via_posrpy.col(6*idx+4).tail(3) = RobotTools::Tr2FixedZYX(temp_rot2);
			circle_via_posrpy.col(6*idx+5).tail(3) = RobotTools::Tr2FixedZYX(temp_rot1);

			line_via_posrpy.col(4*idx).head(3) = center+rot*parc5;
			line_via_posrpy.col(4*idx+1).head(3) = center+rot*parc6;
			line_via_posrpy.col(4*idx+2).head(3) = center+rot*parc5;
			line_via_posrpy.col(4*idx+3).head(3) = center+rot*parc6;

			line_via_posrpy.col(4*idx).tail(3) = RobotTools::Tr2FixedZYX(temp_rot1);
			line_via_posrpy.col(4*idx+1).tail(3) = RobotTools::Tr2FixedZYX(temp_rot3);
			line_via_posrpy.col(4*idx+2).tail(3) = RobotTools::Tr2FixedZYX(temp_rot3);
			line_via_posrpy.col(4*idx+3).tail(3) = RobotTools::Tr2FixedZYX(temp_rot1);
		}
		line_via_posrpy.col(4*cycle_num).head(3) = line_via_posrpy.col(0).head(3);
		line_via_posrpy.col(4*cycle_num+1).head(3) = line_via_posrpy.col(1).head(3);
		line_via_posrpy.col(4*cycle_num).tail(3) = RobotTools::Tr2FixedZYX(temp_rot2);
		line_via_posrpy.col(4*cycle_num+1).tail(3) = RobotTools::Tr2FixedZYX(temp_rot2);
		return cycle_num;
	}

	int CalcCircBasinPath(MatrixXd& circle_via_posrpy, MatrixXd& line_via_posrpy, MatrixXd* vision_pos, double diameterball)
	{
		Vector3d center1, center2, pos1, pos2, pos3, pos4, pos5;
		double radius1, radius2, theta1, theta2;
		Matrix3d rot1, rot2;
		pos1 = vision_pos->col(0); pos2 = vision_pos->col(1); pos3 = vision_pos->col(2);
		pos4 = vision_pos->col(3); pos5 = vision_pos->col(4);
		CalcArcInfo(center1, radius1, rot1, theta1, pos1, pos2, pos3);
		//CalcArcInfo(center2, radius2, rot2, theta2, pos1, pos4, pos3);
		int cycle_num = (int)ceil(radius1/diameterball);
		double step_size = radius1/(cycle_num);
		Matrix3d temp_rot1, temp_rot2;
		temp_rot1 = rot1*RobotTools::RotZ((theta1-pi)/2);
		Vector3d temp_rpy1 = RobotTools::Tr2FixedZYX(temp_rot1);
		circle_via_posrpy.resize(6, 3*cycle_num);
		line_via_posrpy.resize(6, cycle_num);
		for (int idx = 0; idx<cycle_num; idx++)
		{
			Vector3d vecc1_p4 = center1-pos4;
			vecc1_p4.normalize();
			Vector3d temp_pos4 = pos4+step_size*vecc1_p4*idx;
			Vector3d z0 = temp_pos4-pos5;
			z0.normalize();
			Vector3d x0 = temp_rot1.col(0);
			Vector3d y0 = z0.cross(x0);
			Matrix3d rot_mat;
			Vector3d temp_y0 = temp_rot1.col(1);
			double temp_cos = temp_y0.dot(z0)/(y0.norm()*z0.norm());
			if (temp_cos < 0)
			{
				rot_mat<<x0, y0, z0;
			}
			else
				rot_mat = temp_rot1;
			temp_rot2 = rot_mat*RobotTools::RotX(-10*pi/180);//temp_pos4
			Vector3d rpy = RobotTools::Tr2FixedZYX(rot_mat);
			line_via_posrpy.col(idx)<<temp_pos4, rpy;
			Vector3d vecc1_p1 = center1-pos1;
			Vector3d vecc1_p2 = center1-pos2;
			Vector3d vecc1_p3 = center1-pos3;
			vecc1_p1.normalize();
			vecc1_p2.normalize();
			vecc1_p3.normalize();
			Vector3d temp_pos1 = pos1+step_size*vecc1_p1*idx;
			Vector3d temp_pos2 = pos2+step_size*vecc1_p2*idx;
			Vector3d temp_pos3 = pos3+step_size*vecc1_p3*idx;
			circle_via_posrpy.col(3*idx)<<temp_pos1, temp_rpy1;
			circle_via_posrpy.col(3*idx+1)<<temp_pos2, temp_rpy1;
			circle_via_posrpy.col(3*idx+2)<<temp_pos3, temp_rpy1;
		}
		return cycle_num;
	}

	int CalcCircBasinPath(MatrixXd& outer_via_posrpy, MatrixXd* vision_pos,
		double rx[6], double ry[6], double rz[6], double dis_angle, double angle, double tap_theta)
	{
		return 0;
	}

	void CalcArcInfo(Vector3d& center, double& radius, Matrix3d& rot, double& theta, Vector3d pos1, Vector3d pos2, Vector3d pos3)
	{
		Vector4d params1 = RobotTools::PointsCoplane(pos1, pos2, pos3);
		Vector4d params2 = RobotTools::RadiusEqual(pos1, pos2);
		Vector4d params3 = RobotTools::RadiusEqual(pos1, pos3);
		double a1 = params1(0), b1 = params1(1), c1 = params1(2), d1 = params1(3);
		double a2 = params2(0), b2 = params2(1), c2 = params2(2), d2 = params2(3);
		double a3 = params3(0), b3 = params3(1), c3 = params3(2), d3 = params3(3);
		center(0) = -(b1*c2*d3-b1*c3*d2-b2*c1*d3+b2*c3*d1+b3*c1*d2-b3*c2*d1)/
			(a1*b2*c3-a1*b3*c2-a2*b1*c3+a2*b3*c1+a3*b1*c2-a3*b2*c1);
		center(1) = (a1*c2*d3-a1*c3*d2-a2*c1*d3+a2*c3*d1+a3*c1*d2-a3*c2*d1)/
			(a1*b2*c3-a1*b3*c2-a2*b1*c3+a2*b3*c1+a3*b1*c2-a3*b2*c1);
		center(2) = -(a1*b2*d3-a1*b3*d2-a2*b1*d3+a2*b3*d1+a3*b1*d2-a3*b2*d1)/
			(a1*b2*c3-a1*b3*c2-a2*b1*c3+a2*b3*c1+a3*b1*c2-a3*b2*c1);
		Vector3d line_vec = pos3-pos1;
		double line_length = line_vec.norm();
		Vector3d xr = pos1-center;
		radius = xr.norm();
		Vector3d line_vec1 = pos1-pos2;
		Vector3d line_vec2 = pos3-pos2;
		double cos_a = line_vec1.dot(line_vec2)/(line_vec1.norm()*line_vec2.norm());
		MathTools::LimitNum(-1, cos_a, 1);
		theta = acos(1-2*(1-cos_a) * (1+cos_a));
		if (cos_a > 0)
			theta = 2*pi-theta;
		Vector3d zr(a1, b1, c1);
		Vector3d yr = zr.cross(xr);
		xr.normalize(); yr.normalize(); zr.normalize();
		rot<<xr, yr, zr;
	}

	void CalcArcPoint(Vector3d& pos, Vector3d pos1, Vector3d pos2, Vector3d pos3, double angle)
	{
		Vector3d center; double radius, theta; Matrix3d rot;
		CalcArcInfo(center, radius, rot, theta, pos1, pos2, pos3);
		Vector3d parc = Vector3d::Zero();
		parc(0) = radius*cos(angle);
		parc(1) = radius*sin(angle);
		pos = center + rot*parc;
	}

	int CalcEllipseBasinPath(MatrixXd& via_posrpy,MatrixXd* vision_pos, 
		double rx[6], double ry[6], double rz[6], double dis_angle, double angle)
	{
		Vector3d pos1, pos2, pos3, pos4, pos5, pos6;
		pos1 = vision_pos->col(0); pos2 = vision_pos->col(1); pos3 = vision_pos->col(2);
		pos4 = vision_pos->col(3); pos5 = vision_pos->col(4); pos6 = vision_pos->col(5);
		Vector3d center; double a, b; Matrix3d rot;
		InitEllipseInfo(center, a, b, rot, pos4, pos1, pos2);
		int cycle_num = ceil(pi/2/dis_angle);
		double step_size = pi/2 / (cycle_num);
		double step_rx[6], step_ry[6], step_rz[6];
		for (int idx = 0; idx < 6; idx++)
		{
			step_rx[idx] = -rx[idx] / cycle_num;
			step_ry[idx] = -ry[idx] / cycle_num;
			step_rz[idx] = -rz[idx] / cycle_num;
		}
		via_posrpy.resize(6, 6 * cycle_num);
		for (int idx = 0; idx < cycle_num; idx++)
		{
			Vector3d temp_pos1;
			CalcEllipsePoint(temp_pos1, pos1, pos5, pos3, idx*step_size);
			Vector3d temp_pos2;
			CalcEllipsePoint(temp_pos2, pos2, pos5, pos4, idx*step_size);
			Vector3d temp_pos3;
			CalcEllipsePoint(temp_pos3, pos3, pos5, pos1, idx*step_size);
			Vector3d temp_pos4;
			CalcEllipsePoint(temp_pos4, pos4, pos5, pos2, idx*step_size);
			Vector3d temp_pos7;
			CalcEllipsePoint(temp_pos7, temp_pos1, temp_pos2, temp_pos3, -angle);
			Vector3d temp_pos8;
			CalcEllipsePoint(temp_pos8, temp_pos1, temp_pos2, temp_pos3, angle);
			via_posrpy.col(6 * idx).head(3) = temp_pos1;
			double tmp_rx1 = rx[0] + idx*step_rx[0];
			double tmp_ry1 = ry[0] + idx*step_ry[0];
			double tmp_rz1 = rz[0] + idx*step_rz[0];
			Matrix3d temp_rot1 = rot * RobotTools::RotX(tmp_rx1)*RobotTools::RotY(tmp_ry1) * RobotTools::RotZ(tmp_rz1);
			via_posrpy.col(6 * idx).tail(3) = RobotTools::Tr2FixedZYX(temp_rot1);

			via_posrpy.col(6 * idx+1).head(3) = temp_pos2;
			double tmp_rx2 = rx[1] + idx*step_rx[1];
			double tmp_ry2 = ry[1] + idx*step_ry[1];
			double tmp_rz2 = rz[1] + idx*step_rz[1];
			Matrix3d temp_rot2 = rot * RobotTools::RotX(tmp_rx2)*RobotTools::RotY(tmp_ry2) * RobotTools::RotZ(tmp_rz2);
			via_posrpy.col(6 * idx+1).tail(3) = RobotTools::Tr2FixedZYX(temp_rot2);

			via_posrpy.col(6 * idx+2).head(3) = temp_pos3;
			double tmp_rx3 = rx[2] + idx*step_rx[2];
			double tmp_ry3 = ry[2] + idx*step_ry[2];
			double tmp_rz3 = rz[2] + idx*step_rz[2];
			Matrix3d temp_rot3 = rot * RobotTools::RotX(tmp_rx3)*RobotTools::RotY(tmp_ry3) * RobotTools::RotZ(tmp_rz3);
			via_posrpy.col(6 * idx+2).tail(3) = RobotTools::Tr2FixedZYX(temp_rot3);

			via_posrpy.col(6 * idx+3).head(3) = temp_pos4;
			double tmp_rx4 = rx[3] + idx*step_rx[3];
			double tmp_ry4 = ry[3] + idx*step_ry[3];
			double tmp_rz4 = rz[3] + idx*step_rz[3];
			Matrix3d temp_rot4 = rot * RobotTools::RotX(tmp_rx4)*RobotTools::RotY(tmp_ry4) * RobotTools::RotZ(tmp_rz4);
			via_posrpy.col(6 * idx+3).tail(3) = RobotTools::Tr2FixedZYX(temp_rot4);

			via_posrpy.col(6 * idx+4).head(3) = temp_pos7;
			double tmp_rx7 = rx[4] + idx*step_rx[4];
			double tmp_ry7 = ry[4] + idx*step_ry[4];
			double tmp_rz7 = rz[4] + idx*step_rz[4];
			Matrix3d temp_rot7 = rot * RobotTools::RotX(tmp_rx7)*RobotTools::RotY(tmp_ry7) * RobotTools::RotZ(tmp_rz7);
			via_posrpy.col(6 * idx+4).tail(3) = RobotTools::Tr2FixedZYX(temp_rot7);

			via_posrpy.col(6 * idx+5).head(3) = temp_pos8;
			double tmp_rx8 = rx[5] + idx*step_rx[5];
			double tmp_ry8 = ry[5] + idx*step_ry[5];
			double tmp_rz8 = rz[5] + idx*step_rz[5];
			Matrix3d temp_rot8 = rot * RobotTools::RotX(tmp_rx8)*RobotTools::RotY(tmp_ry8) * RobotTools::RotZ(tmp_rz8);
			via_posrpy.col(6 * idx+5).tail(3) = RobotTools::Tr2FixedZYX(temp_rot8);
		}
		return cycle_num;
	}

	int CalcEllipseBasinPath(MatrixXd& via_posrpy, MatrixXd* vision_pos,
		double rx[6], double ry[6], double rz[6], double dis_angle, double angle, double tap_theta)
	{
		Vector3d pos1, pos2, pos3, pos4, pos5, pos6;
		pos1 = vision_pos->col(0); pos2 = vision_pos->col(1); pos3 = vision_pos->col(2);
		pos4 = vision_pos->col(3); pos5 = vision_pos->col(4); pos6 = vision_pos->col(5);
		Vector3d center; double a, b; Matrix3d rot;
		InitEllipseInfo(center, a, b, rot, pos4, pos1, pos2);
		int cycle_num = ceil(pi / 2 / dis_angle);
		double step_size = pi / 2 / (cycle_num);
		double step_rx[6], step_ry[6], step_rz[6];
		for (int idx = 0; idx < 6; idx++)
		{
			step_rx[idx] = -rx[idx] / cycle_num;
			step_ry[idx] = -ry[idx] / cycle_num;
			step_rz[idx] = -rz[idx] / cycle_num;
		}
		via_posrpy.resize(6, 6 * cycle_num);
		for (int idx = 0; idx < cycle_num; idx++)
		{
			Vector3d temp_pos1;
			CalcEllipsePoint(temp_pos1, pos1, pos5, pos3, idx*step_size);
			Vector3d temp_pos2;
			CalcEllipsePoint(temp_pos2, pos2, pos5, pos4, idx*step_size);
			Vector3d temp_pos3;
			CalcEllipsePoint(temp_pos3, pos3, pos5, pos1, idx*step_size);
			Vector3d temp_pos4;
			CalcEllipsePoint(temp_pos4, pos4, pos5, pos2, idx*step_size);
			Vector3d temp_pos11;
			CalcEllipsePoint(temp_pos11, temp_pos1, temp_pos2, temp_pos3, tap_theta);
			Vector3d temp_pos8;
			CalcEllipsePoint(temp_pos8, temp_pos1, temp_pos2, temp_pos3, tap_theta + angle);
			via_posrpy.col(6 * idx).head(3) = temp_pos1;
			double tmp_rx1 = rx[0] + idx*step_rx[0];
			double tmp_ry1 = ry[0] + idx*step_ry[0];
			double tmp_rz1 = rz[0] + idx*step_rz[0];
			Matrix3d temp_rot1 = rot * RobotTools::RotX(tmp_rx1)*RobotTools::RotY(tmp_ry1) * RobotTools::RotZ(tmp_rz1);
			via_posrpy.col(6 * idx).tail(3) = RobotTools::Tr2FixedZYX(temp_rot1);

			via_posrpy.col(6 * idx + 1).head(3) = temp_pos2;
			double tmp_rx2 = rx[1] + idx*step_rx[1];
			double tmp_ry2 = ry[1] + idx*step_ry[1];
			double tmp_rz2 = rz[1] + idx*step_rz[1];
			Matrix3d temp_rot2 = rot * RobotTools::RotX(tmp_rx2)*RobotTools::RotY(tmp_ry2) * RobotTools::RotZ(tmp_rz2);
			via_posrpy.col(6 * idx + 1).tail(3) = RobotTools::Tr2FixedZYX(temp_rot2);

			via_posrpy.col(6 * idx + 2).head(3) = temp_pos3;
			double tmp_rx3 = rx[2] + idx*step_rx[2];
			double tmp_ry3 = ry[2] + idx*step_ry[2];
			double tmp_rz3 = rz[2] + idx*step_rz[2];
			Matrix3d temp_rot3 = rot * RobotTools::RotX(tmp_rx3)*RobotTools::RotY(tmp_ry3) * RobotTools::RotZ(tmp_rz3);
			via_posrpy.col(6 * idx + 2).tail(3) = RobotTools::Tr2FixedZYX(temp_rot3);

			via_posrpy.col(6 * idx + 3).head(3) = temp_pos4;
			double tmp_rx4 = rx[3] + idx*step_rx[3];
			double tmp_ry4 = ry[3] + idx*step_ry[3];
			double tmp_rz4 = rz[3] + idx*step_rz[3];
			Matrix3d temp_rot4 = rot * RobotTools::RotX(tmp_rx4)*RobotTools::RotY(tmp_ry4) * RobotTools::RotZ(tmp_rz4);
			via_posrpy.col(6 * idx + 3).tail(3) = RobotTools::Tr2FixedZYX(temp_rot4);

			via_posrpy.col(6 * idx + 4).head(3) = temp_pos11;
			double tmp_rx5 = rx[4] + idx*step_rx[4];
			double tmp_ry5 = ry[4] + idx*step_ry[4];
			double tmp_rz5 = rz[4] + idx*step_rz[4];
			Matrix3d temp_rot5 = rot * RobotTools::RotX(tmp_rx5)*RobotTools::RotY(tmp_ry5) * RobotTools::RotZ(tmp_rz5);
			via_posrpy.col(6 * idx + 4).tail(3) = RobotTools::Tr2FixedZYX(temp_rot5);

			via_posrpy.col(6 * idx + 5).head(3) = temp_pos8;
			double tmp_rx6 = rx[5] + idx*step_rx[5];
			double tmp_ry6 = ry[5] + idx*step_ry[5];
			double tmp_rz6 = rz[5] + idx*step_rz[5];
			Matrix3d temp_rot6 = rot * RobotTools::RotX(tmp_rx6)*RobotTools::RotY(tmp_ry6) * RobotTools::RotZ(tmp_rz6);
			via_posrpy.col(6 * idx + 5).tail(3) = RobotTools::Tr2FixedZYX(temp_rot6);
		}
		return cycle_num;
	}

	void CalcEllipsePoint(Vector3d& pos, Vector3d pos1, Vector3d pos2, Vector3d pos3, double angle)
	{
		Vector3d center; double a, b; Matrix3d rot;
		InitEllipseInfo(center, a, b, rot, pos1, pos2, pos3);
		Vector3d parc = Vector3d::Zero();
		parc(0) = b*cos(angle);
		parc(1) = a*sin(angle);
		pos = center + rot*parc;
	}

	void InitEllipseInfo(Vector3d& center, double& a, double& b, Matrix3d& rot, Vector3d pos1, Vector3d pos2, Vector3d pos3)
	{
		center = (pos1 + pos3) / 2;
		Vector3d vec_a = pos2 - center;
		Vector3d vec_b = pos1 - center;
		a = vec_a.norm();
		b = vec_b.norm();
		Vector3d zr = vec_b.cross(vec_a);
		zr.normalize();
		vec_b.normalize();
		Vector3d yr = zr.cross(vec_b);
		yr.normalize();
		rot << vec_b, yr, zr;
	}
}

