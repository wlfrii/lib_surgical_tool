#ifndef _MATH_EXTENSION_
#define _MATH_EXTENSION_
#include <cmath>
#include <math.h>
#include <vector>
#include <Eigen/Dense>
#include "rt.h"

const double PI = 3.14159265358979323846;

namespace math
{
	/**/
	template<typename T1 = double, typename T2 = double>
	inline T1 deg2rad(const T2 & degree)
	{
		return T1(degree / 180.0 * PI);
	}

	/**/
	template<typename T1 = double, typename T2 = double>
	inline T1 rad2deg(const T2 & radian)
	{
		return T1(radian / PI * 180);
	}

	/**/
	template<typename T1, typename T2, typename T3>
	std::vector<T3>& linespace(const T1 &start, const T2 &end, const int &num, std::vector<T3> &output)
	{
		if (num == 1) {
			output.push_back(T3(start));
			return output;
		}
		if (num == 2) {
			output.push_back(T3(start));
			output.push_back(end);
			return output;
		}

		float step = (T3(end) - T3(start)) / T3(num - 1);
		for (int i = 0; i < num; ++i)
			output.push_back(T3(start + i * step));

		return output;
	}

	/**/
	template<typename T1, typename T2, typename T3>
	std::vector<T3>& linespace2(const T1 &start, const float &interval, const T2 &end, std::vector<T3> &output)
	{
		if (T3(interval) > T3(end - start)) {
			output.push_back(T3(start));
			return output;
		}
		int num = ceil((end - start) / interval);
		for (int i = 0; i < num; ++i)
			output.push_back(T3(start + i * interval));
		output.push_back(end);

		return output;
	}

	template<typename T, typename T1 = double>
	inline Eigen::Matrix<T, 3, 3> rotByX(const T1 &rad)
	{
		Eigen::Matrix<T, 3, 3> res;
		res << T(1), T(0), T(0), T(0), T(cos(rad)), T(-sin(rad)), T(0), T(sin(rad)), T(cos(rad));
		return res;
	}

	template<typename T, typename T1 = double>
	inline Eigen::Matrix<T, 3, 3> rotByY(const T1 &rad)
	{
		Eigen::Matrix<T, 3, 3> res;
		res << T(cos(rad)), T(0), T(sin(rad)), T(0), T(1), T(0), T(-sin(rad)), T(0), T(cos(rad));
		return res;
	}

	template<typename T, typename T1 = double>
	inline Eigen::Matrix<T, 3, 3> rotByZ(const T1 &rad)
	{
		Eigen::Matrix<T, 3, 3> res;
		res << T(cos(rad)), T(-sin(rad)), T(0), T(sin(rad)), T(cos(rad)), T(0), T(0), T(0), T(1);
		return res;
	}

	template<typename T, int M, int N>
	Eigen::Matrix<T, M, N> absMat(const Eigen::Matrix<T, M, N> &vec)
	{
		Eigen::Matrix<T, M, N> absvec = vec;
		for (int i = 0; i < absvec.rows(); i++)
			for (int j = 0; j < absvec.cols(); j++)
				absvec(i, j) = abs(absvec(i, j));
		return absvec;
	}



	// =====================================================================
	// ---------------------- Continuum Kinematics -------------------------
	// =====================================================================
	
	// Calulate RT of a single continuum segment
	void calcSingleSegmentRT(float L, float theta, float delta, RT& rt);
	RT   calcSingleSegmentRT(float L, float theta, float delta);

	// Calulate RT of a single continuum segment followed with a rigid segment
	void calcSingleWithRigidSegmentRT(float L, float Lr, float theta, float delta, RT& rt);
	RT   calcSingleWithRigidSegmentRT(float L, float Lr, float theta, float delta);

	Eigen::Matrix3f createRotMatByVec(Eigen::Vector3f z);
}

#endif //_MATH_DEFINE_