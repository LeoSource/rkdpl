#include "RobotMath.h"
namespace MathTools
{
	int Discretize(double* arr, int len, double value)
	{
		int idx = 0;
		if (fabs(value - arr[len - 1]) < EPS)
		{
			idx = len - 2;
		}
		else
		{
			auto upper_value = upper_bound(arr, arr + len, value);
			idx = static_cast<int>(upper_value - arr - 1);
		}

		return idx;
	}

	int Discretize(VectorXd vec, int len, double value)
	{
		int idx = 0, start_idx = 0, mid_idx = 0, end_idx = len;
		if (fabs(value - vec(len - 1)) < EPS)
		{
			idx = len - 2;
		}
		else
		{
			while (end_idx - start_idx>1)
			{
				mid_idx = (start_idx + end_idx) / 2;
				if (vec(mid_idx) > value)
				{
					end_idx = mid_idx;
				}
				else
				{
					start_idx = mid_idx;
				}
			}
			idx = start_idx;
		}

		return idx;
	}

	double CalcTransEqua(double a, double b, double c, double pre_value)
	{
		//calculate the equation like a*cos(theta)+b*sin(theta) = c
		double theta = 0, u = 0;
		double deg_thre[] = { -pi / 2, pi / 2 };
		if (pow(a, 2)+pow(b, 2)<pow(c, 2))
			theta = pre_value;
		else
		{
			if (fabs(a+c) < EPS)
			{
				u = (c-a)/(2*b);
				theta = 2*atan(u);
			}
			else
			{
				double tmp_value1 = (b+sqrt(pow(b, 2)+pow(a, 2)-pow(c, 2)))/(a+c);
				double tmp_value2 = (b-sqrt(pow(b, 2)+pow(a, 2)-pow(c, 2)))/(a+c);
				tmp_value1 = 2*atan(tmp_value1);
				tmp_value2 = 2*atan(tmp_value2);
				if ((tmp_value1 > deg_thre[1])||(tmp_value1 < deg_thre[0]))
					theta = tmp_value2;
				else
					theta = tmp_value1;
			}
		}

		return theta;
	}

	int CalcSinCosEqua(double *x1, double *x2, double a, double b, double c)
	{
		double aa, discr;
		int n;

		if (a == 0. && b == 0. && c == 0.)
		{
			n = 0;
			*x1 = -100;
			*x2 = -100;
		}
		else if (a == b && b == c)
		{
			n = 2;
			*x1 = -pi * 0.5;
			*x2 = pi;
		}
		else if (b == 0. && c == 0.)
		{
			n = 2;
			*x1 = -pi;
			*x2 = pi;
		}
		else if (a == 0. && c == 0.)
		{
			n = 2;
			*x1 = -pi * 0.5;
			*x2 = pi * 0.5;
		}
		else
		{
			discr = a * a + b * b - c * c;
			if (discr >= 0.)
				aa = sqrt(discr);
			else
				aa = 0.;

			if (discr < -0.01)
			{
				n = 0;
				*x1 = -100;
				*x2 = -100;
			}
			else
			{
				*x1 = 2. * atan2((double)(-a + aa), (double)(-b + c));
				*x2 = 2. * atan2((double)(-a - aa), (double)(-b + c));
				if (*x1 > 0 && *x1 > pi)
					*x1 = *x1 - 2. * pi;
				else if (*x1 < 0 && *x1 < -pi)
					*x1 = *x1 + 2. * pi;

				if (*x2 > 0 && *x2 > pi)
					*x2 = *x2 - 2. * pi;
				else if (*x2 < 0 && *x2 < -pi)
					*x2 = *x2 + 2. * pi;
				n = 2;
			}
		}
		return (n);
	}

	int Sign(double x)
	{
		if (x < 0)
			return -1;
		else
			return 1;
	}

    int LimitNum(double min_value, double& value, double max_value)
	{
        if (value > max_value)
        {
            value = max_value;
            return -1;
        }
        else if (value < min_value)
        {
            value = min_value;
            return -1;
        }

        return 0;
	}

	void LimitMin(double min_value, double& value)
	{
		if (value < min_value)
			value = min_value;
	}

	void LimitMax(double max_value, double &value)
	{
		if (value > max_value)
			value = max_value;
	}

    int LimitVector(VectorXd min_vec, VectorXd* value, VectorXd max_vec)
	{
        int err = 0;
		for (int idx = 0; idx<value->size(); idx++)
		{
            err |= LimitNum(min_vec(idx), (*value)(idx), max_vec(idx));
		}
        return err;
	}

	bool Any(VectorXd vec)
	{
		for (int idx = 0; idx<vec.size(); idx++)
		{
			if (fabs(vec(idx))>EPS)
				return true;
		}
		return false;
	}

	double Norm(VectorXd vec)
	{
		double res = 0;
		for (int idx = 0; idx<vec.size(); idx++)
		{
			res += pow(vec(idx), 2);
		}
		res = sqrt(res);

		return res;
	}

	Vector3d Cross(Vector3d v1, Vector3d v2)
	{
		return v1.cross(v2);
	}

	int Factorial(int n)
	{
		if (n==0)
			return 1;
		else
			return n*Factorial(n-1);
	}

	Vector3d CalcXZPlaneIntersection(Vector3d norm_vec, Vector3d via_point)
	{
		double d = -norm_vec.dot(via_point);
		double y = 0;
		double x = via_point(0)+1;
		double z = -(norm_vec(0)*x+d)/norm_vec(2);
		
		return Vector3d(x, y, z)-via_point;
	}

}


namespace RobotTools
{
	Pose PoseProduct(Pose p1, Pose p2)
	{
		Pose res;
		res.rot = p1.rot*p2.rot;
		res.pos = p1.pos+p1.rot*p2.pos;

		return res;
	}

	Pose PoseInverse(Pose p1)
	{
		Pose res;
		Matrix3d temp_rot;
		temp_rot = p1.rot;
		res.rot = temp_rot.transpose();
		res.pos = -(temp_rot.transpose() * p1.pos);

		return res;
	}

	Matrix3d RotX(double angle)
	{
		Matrix3d res;
		res.setIdentity();
		res(1, 1) = cos(angle);
		res(2, 2) = cos(angle);
		res(1, 2) = -sin(angle);
		res(2, 1) = sin(angle);
		
		return res;
	}

	Matrix3d RotY(double angle)
	{
		Matrix3d res;
		res.setIdentity();
		res(0, 0) = cos(angle);
		res(2, 2) = cos(angle);
		res(0, 2) = sin(angle);
		res(2, 0) = -sin(angle);

		return res;
	}

	Matrix3d RotZ(double angle)
	{
		Matrix3d res;
		res.setIdentity();
		res(0, 0) = cos(angle);
		res(1, 1) = cos(angle);
		res(0, 1) = -sin(angle);
		res(1, 0) = sin(angle);

		return res;
	}

	CAVP LinetoSpatial(CLineAVP* line_avp)
	{
		CAVP cavp;
		cavp.pos.setZero();
		cavp.vel.setZero();
		cavp.acc.setZero();
		cavp.pos.head(3) = line_avp->pos;
		cavp.vel.head(3) = line_avp->vel;
		cavp.acc.head(3) = line_avp->acc;

		return cavp;
	}


	CAVP AngtoSpatial(CLineAVP* ang_avp)
	{
		CAVP cavp;
		cavp.pos.setZero();
		cavp.vel.setZero();
		cavp.acc.setZero();
		cavp.pos.segment(3, 3) = ang_avp->pos;
		cavp.vel.segment(3, 3) = ang_avp->vel;
		cavp.acc.segment(3, 3) = ang_avp->acc;

		return cavp;
	}

	CAVP toSpatial(CLineAVP* line_avp, CLineAVP* ang_avp)
	{
		CAVP cavp;
		cavp.pos.head(3) = line_avp->pos;
		cavp.vel.head(3) = line_avp->vel;
		cavp.acc.head(3) = line_avp->acc;
		cavp.pos.segment(3, 3) = ang_avp->pos;
		cavp.vel.segment(3, 3) = ang_avp->vel;
		cavp.acc.segment(3, 3) = ang_avp->acc;

		return cavp;
	}

	Matrix3d RPY2Jaco(Vector3d rpy)
	{
		Matrix3d res;
		res.setIdentity();
		res(0, 2) = sin(rpy(1));
		res(1, 1) = cos(rpy(0));
		res(1, 2) = -sin(rpy(0))*cos(rpy(1));
		res(2, 1) = sin(rpy(0));
		res(2, 2) = cos(rpy(0))*cos(rpy(1));

		return res;
	}

	Vector4d Tr2Quat1(Matrix3d r)
	{
		Vector4d res;
		res.setZero();
		const double trace = r(0, 0) + r(1, 1) + r(2, 2);
		double root;
		double u0, u1, u2, u3;

		if (trace > 0.0) {
			root = sqrt(trace + 1.0);
			u0 = 0.5 * root;
			root = 0.5 / root;
			u1 = (r(2, 1) - r(1, 2)) * root;
			u2 = (r(0, 2) - r(2, 0)) * root;
			u3 = (r(1, 0) - r(0, 1)) * root;
		}
		else if (r(0, 0) >= r(1, 1) && r(0, 0) >= r(2, 2))
		{
			root = sqrt(1.0 + r(0, 0) - (r(1, 1) + r(2, 2)));
			u1 = 0.5 * root;
			root = 0.5 / root;
			u2 = (r(0, 1) + r(1, 0)) * root;
			u3 = (r(2, 0) + r(0, 2)) * root;
			u0 = (r(2, 1) - r(1, 2)) * root;
		}
		else if (r(1, 1) >= r(2, 2))
		{
			root = sqrt(1.0 + r(1, 1) - (r(2, 2) + r(0, 0)));
			u2 = 0.5 * root;
			root = 0.5 / root;
			u3 = (r(1, 2) + r(2, 1)) * root;
			u1 = (r(0, 1) + r(1, 0)) * root;
			u0 = (r(0, 2) - r(2, 0)) * root;
		}
		else
		{
			root = sqrt(1.0 + r(2, 2) - (r(0, 0) + r(1, 1)));
			u3 = 0.5 * root;
			root = 0.5 / root;
			u1 = (r(2, 0) + r(0, 2)) * root;
			u2 = (r(1, 2) + r(2, 1)) * root;
			u0 = (r(1, 0) - r(0, 1)) * root;
		}

		{
			double norm = sqrt(u0 * u0 + u1 * u1 + u2 * u2 + u3 * u3);

			u0 /= norm;
			u1 /= norm;
			u2 /= norm;
			u3 /= norm;
		}
		if (u0 < 0.0)
		{
			u0 = -u0;
			u1 = -u1;
			u2 = -u2;
			u3 = -u3;
		}
		res << u0, u1, u2, u3;
		return res;
	}

	Vector4d Tr2Quat2(Matrix3d r)
	{
		Vector4d res;
		res.setZero();
		const double trace = r(0, 0) + r(1, 1) + r(2, 2);
		double root;
		double u0, u1, u2, u3;

		if (trace > 0.0) {
			root = sqrt(trace + 1.0);
			u0 = 0.5 * root;
			root = 0.5 / root;
			u1 = (r(2, 1) - r(1, 2)) * root;
			u2 = (r(0, 2) - r(2, 0)) * root;
			u3 = (r(1, 0) - r(0, 1)) * root;
		}
		else if (r(0, 0) >= r(1, 1) && r(0, 0) >= r(2, 2))
		{
			root = sqrt(1.0 + r(0, 0) - (r(1, 1) + r(2, 2)));
			u1 = 0.5 * root;
			root = 0.5 / root;
			u2 = (r(0, 1) + r(1, 0)) * root;
			u3 = (r(2, 0) + r(0, 2)) * root;
			u0 = (r(2, 1) - r(1, 2)) * root;
		}
		else if (r(1, 1) >= r(2, 2))
		{
			root = sqrt(1.0 + r(1, 1) - (r(2, 2) + r(0, 0)));
			u2 = 0.5 * root;
			root = 0.5 / root;
			u3 = (r(1, 2) + r(2, 1)) * root;
			u1 = (r(0, 1) + r(1, 0)) * root;
			u0 = (r(0, 2) - r(2, 0)) * root;
		}
		else
		{
			root = sqrt(1.0 + r(2, 2) - (r(0, 0) + r(1, 1)));
			u3 = 0.5 * root;
			root = 0.5 / root;
			u1 = (r(2, 0) + r(0, 2)) * root;
			u2 = (r(1, 2) + r(2, 1)) * root;
			u0 = (r(1, 0) - r(0, 1)) * root;
		}

		{
			double norm = sqrt(u0 * u0 + u1 * u1 + u2 * u2 + u3 * u3);

			u0 /= norm;
			u1 /= norm;
			u2 /= norm;
			u3 /= norm;
		}
		res << u0, u1, u2, u3;
		return res;
	}

	Vector4d Tr2AngleAxis(Matrix3d r)
	{
		Vector4d res;
		res.setZero();
		Vector4d q = Tr2Quat1(r);
		Vector4d q_temp;
		q_temp.setZero();
		bool offset = false;
		double theta, x, y, z;
		if (q(0) < 0)
		{
			offset = true;
			q_temp(0) = -q(0);
		}
		else
		{
			q_temp(0) = q(0);
		}
		q_temp(1) = q(1);
		q_temp(2) = q(2);
		q_temp(3) = q(3);

		theta = 2.0 * acos(q_temp(0));

		if (fabs(theta) > 2.5e-2)
		{
			double sinth = sin(theta / 2.0);
			x = q_temp(1) / sinth;
			y = q_temp(2) / sinth;
			z = q_temp(3) / sinth;
		}
		else
		{
			double vect_len;
			vect_len = q_temp(1) * q_temp(1) + q_temp(2) * q_temp(2) + q_temp(3) * q_temp(3);
			if (vect_len < EPS10)
			{
				theta = 0.0;
				x = 0.0;
				y = 0.0;
				z = 0.0;
			}
			else
			{
				double sin_theta_approx_half = sqrt(vect_len);
				theta = (sin_theta_approx_half * 2.0);
				x = q_temp(1) / sin_theta_approx_half;
				y = q_temp(2) / sin_theta_approx_half;
				z = q_temp(3) / sin_theta_approx_half;
			}
		}
		if (offset == true)
			theta = (2 * pi) - theta;
		res << x, y, z, theta;
		return res;
	}

	Matrix3d AngleAxis2Tr(Vector4d E)
	{
		Matrix3d r;
		r.setIdentity();
		double kx = E(0);
		double ky = E(1);
		double kz = E(2);
		double ct = cos(E(3));
		double st = sin(E(3));
		double vt = 1 - ct;
		r(0, 0) = kx*kx*vt + ct;     r(0, 1) = kx*ky*vt - kz*st;  r(0, 2) = kx*kz*vt + ky*st;
		r(1, 0) = kx*ky*vt + kz*st;  r(1, 1) = ky*ky*vt + ct;     r(1, 2) = ky*kz*vt - kx*st;
		r(2, 0) = kx*kz*vt - ky*st;  r(2, 1) = ky*kz*vt + kx*st;  r(2, 2) = kz*kz*vt + ct;
		return r;
	}

	Vector3d Tr2FixedZYX(Matrix3d r)
	{
		Vector3d res;
		res.setZero();
		double x, y, z;
		double sqr;
		sqr = sqrt(r(1, 2) * r(1, 2) + r(2, 2) * r(2, 2));
		y = atan2(r(0, 2), sqr);

		if (fabs(y - pi * 0.5) <= EPS4)
		{
			z = 0.;
			x = atan2(r(1, 0), r(1, 1));
		}
		else if (fabs(y + pi * 0.5) <= EPS4)
		{
			z = 0.;
			x = atan2(-r(1, 0), r(1, 1));
		}
		else
		{
			/* No degeneration */
			z = atan2(-r(0, 1), r(0, 0));
			x = atan2(-r(1, 2), r(2, 2));
		}
		res << x, y, z;
		return res;
	}

	Matrix3d FixedZYX2Tr(Vector3d rpy)
	{
		Matrix3d res;
		res.setIdentity();
		double Z = rpy(2);
		double Y = rpy(1);
		double X = rpy(0);
		res(0, 0) = cos(Y)*cos(Z);
		res(0, 1) = -cos(Y)*sin(Z);
		res(0, 2) = sin(Y);
		res(1, 0) = sin(X)*sin(Y)*cos(Z) + cos(X)*sin(Z);
		res(1, 1) = -sin(X)*sin(Y)*sin(Z) + cos(X)*cos(Z);
		res(1, 2) = -sin(X)*cos(Y);
		res(2, 0) = -cos(X)*sin(Y)*cos(Z) + sin(X)*sin(Z);
		res(2, 1) = cos(X)*sin(Y)*sin(Z) + sin(X)*cos(Z);
		res(2, 2) = cos(X)*cos(Y);
		return res;
	}

	Vector4d CalcAngleAxis(Vector3d rpy0, Vector3d rpyn)
	{
		Vector4d res;
		res.setZero();
		Matrix3d r0 = FixedZYX2Tr(rpy0);
		Matrix3d rn = FixedZYX2Tr(rpyn);
		Matrix3d r = r0.inverse() * rn;
		res = Tr2AngleAxis(r);
		return res;
	}

	Matrix6d InvAdT(Pose pose)
	{
		Matrix6d res;
		Matrix3d zeros3;
		zeros3.setZero(3,3);
		Pose temp_pose;
		temp_pose = PoseInverse(pose);
		Matrix3d skew_pos;
		skew_pos = Skew(pose.pos);
		res.block(0, 0, 3, 3) = temp_pose.rot;
		res.block(0, 3, 3, 3) = skew_pos*temp_pose.rot;
		res.block(3, 0, 3, 3) = zeros3;
		res.block(3, 3, 3, 3) = temp_pose.rot;

		return res;
	}

	Matrix3d Skew(Vector3d w)
	{
		Matrix3d res;
		res << 0, -w(2), w(1), w(2), 0, -w(0), -w(1), w(0), 0;

		return res;
	}

	Vector6d LogT(Pose pose)
	{
		Vector6d res;
		double temp_cos = (pose.rot.trace() - 1) / 2.0;
		MathTools::LimitNum(-1, temp_cos, 1);
		double a = acos(temp_cos);
		Matrix3d skew_w;
		if (fabs(a) < EPS6)
			skew_w = (pose.rot - pose.rot.transpose()) / 2.0;
		else
			skew_w = a * (pose.rot - pose.rot.transpose()) / (2.0*sin(a));
		Vector3d w;
		w << skew_w(2, 1), skew_w(0, 2), skew_w(1, 0);
		double w_norm = w.norm();
		Matrix3d A = Matrix3d::Identity(3,3)- skew_w / 2.0 + (2.0 * sin(w_norm) - w_norm*(1 + cos(w_norm)))*skew_w*skew_w / (2.0 * w_norm*w_norm*sin(w_norm));
		res << A*pose.pos, w;

		return res;
	}

	MatrixXd CalcSplineTransPos(Vector3d pos1, Vector3d pos2, Vector3d pos3, double r, string opt)
	{
		MatrixXd ctrlpos;
		if (opt=="spline")
		{
			ctrlpos.setZero(3, 5);
			ctrlpos.col(2) = pos2;
			double line1_len = MathTools::Norm(pos2-pos1);
			double line2_len = MathTools::Norm(pos3-pos2);
			ctrlpos.col(0) = pos2+r/line1_len*(pos1-pos2);
			ctrlpos.col(4) = pos2+r/line2_len*(pos3-pos2);
			double ratio = 0.5;
			ctrlpos.col(1) = ctrlpos.col(2)+ratio*(ctrlpos.col(0)-ctrlpos.col(2));
			ctrlpos.col(3) = ctrlpos.col(2)+ratio*(ctrlpos.col(4)-ctrlpos.col(2));
		}
		else if (opt=="arc")
		{
			ctrlpos.setZero(3, 2);
			double line1_len = MathTools::Norm(pos2-pos1);
			double line2_len = MathTools::Norm(pos3-pos2);
			ctrlpos.col(0) = pos2+r/line1_len*(pos1-pos2);
			ctrlpos.col(1) = pos2+r/line2_len*(pos3-pos2);
		}
		return ctrlpos;
	}

	Vector4d RadiusEqual(Vector3d pos1, Vector3d pos2)
	{
		Vector4d res;
		res(0) = 2*(pos2(0)-pos1(0));
		res(1) = 2*(pos2(1)-pos1(1));
		res(2) = 2*(pos2(2)-pos1(2));
		res(3) = pow(pos1(0), 2)+pow(pos1(1), 2)+pow(pos1(2), 2)
			-pow(pos2(0), 2)-pow(pos2(1), 2)-pow(pos2(2), 2);

		return res;
	}

	Vector4d PointsCoplane(Vector3d pos1, Vector3d pos2, Vector3d pos3)
	{
		double x1 = pos1(0), y1 = pos1(1), z1 = pos1(2);
		double x2 = pos2(0), y2 = pos2(1), z2 = pos2(2);
		double x3 = pos3(0), y3 = pos3(1), z3 = pos3(2);
		double a = y1*z2-y2*z1-y1*z3+y3*z1+y2*z3-y3*z2;
		double b = -(x1*z2-x2*z1-x1*z3+x3*z1+x2*z3-x3*z2);
		double c = x1*y2-x2*y1-x1*y3+x3*y1+x2*y3-x3*y2;
		double d = -(x1*y2*z3-x1*y3*z2-x2*y1*z3+x2*y3*z1+x3*y1*z2-x3*y2*z1);

		return Vector4d(a, b, c, d);
	}

	double CalcArcRadius(Vector3d pos1, Vector3d pos2, Vector3d pos3)
	{
		Vector3d p2p1 = pos1-pos2;
		Vector3d p2p3 = pos3-pos2;
		double inc_angle = acos(p2p1.dot(p2p3)/p2p1.norm()/p2p3.norm());
		double radius = p2p1.norm()*tan(0.5*inc_angle);

		return radius;
	}

	Matrix3d Rodrigues(Vector3d r, double angle)
	{
		Matrix3d res,r_skew;
		r_skew = Skew(r);
		res = Matrix3d::Identity()*cos(angle) + (1 - cos(angle))*r*r.transpose() + sin(angle)*r_skew;
		return res;
	}
}


