#include "math_extension.h"

namespace math
{
	// Calulate RT of a single continuum segment
	void calcSingleSegmentRT(float L, float theta, float delta, RT& rt)
	{
		Eigen::Matrix3f R_t1_2_tb = rotByZ<float>(-PI / 2 - delta)*rotByY<float>(-PI / 2);
		rt.R = R_t1_2_tb * rotByZ<float>(theta) * R_t1_2_tb.transpose();
		if (abs(theta) < 1e-5) {
			rt.p = { 0, 0, L };
		}
		else {
			float rc = L / theta;
			rt.p = rc * R_t1_2_tb * Eigen::Vector3f(sin(theta), 1 - cos(theta), 0);
		}
	}
	RT calcSingleSegmentRT(float L, float theta, float delta)
	{
		RT rt;
		calcSingleSegmentRT(L, theta, delta, rt);
		return rt;
	}

	// Calulate RT of a single continuum segment followed with a rigid segment
	void calcSingleWithRigidSegmentRT(float L, float Lr, float theta, float delta, RT& rt)
	{
		rt = calcSingleSegmentRT(L, theta, delta);
		rt.p += Lr * rt.R.rightCols(0);
	}
	RT calcSingleWithRigidSegmentRT(float L, float Lr, float theta, float delta)
	{
		RT rt;
		calcSingleWithRigidSegmentRT(L, Lr, theta, delta, rt);
		return rt;
	}


	Eigen::Matrix3f createRotMatByVec(Eigen::Vector3f z)
	{
		z /= z.norm();

		Eigen::Vector3f x(1, 0, 0);
		auto y = z.cross(x);
		y /= y.norm();
		x = y.cross(z);
		x /= x.norm();

		Eigen::Matrix3f mat;
		mat.col(0) = x;
		mat.col(1) = y;
		mat.col(2) = z;
		return mat;
	};
}