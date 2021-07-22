#ifndef RT_H
#define RT_H
#include <../inc/Eigen/Dense>

struct RT
{
	RT(const Eigen::Matrix3f &R = Eigen::Matrix3f::Identity(), const Eigen::Vector3f &p = {0,0,0})
		: R(R)
		, p(p)
	{}

	RT(float px, float py, float pz)
		: R(Eigen::Matrix3f::Identity())
		, p(Eigen::Vector3f(px,py,pz))
	{}

	RT(float pts[16], bool is_row_fisrt = true)
	{
		if (is_row_fisrt) {
			R << pts[0], pts[1], pts[2], pts[4], pts[5], pts[6], pts[8], pts[9], pts[10];
			p << pts[3], pts[7], pts[11];
		}
		else{
			R << pts[0], pts[4], pts[8], pts[1], pts[5], pts[9], pts[2], pts[6], pts[10];
			p << pts[12], pts[13], pts[14];
		}
	}

	RT& operator= (const RT& rt)
	{
		this->R = rt.R;
		this->p = rt.p;
		return *this;
	}

	char* to_c_str() const
	{
		static char tmp_cstr[200];
		sprintf(tmp_cstr, "row-1st, R=[%f,%f,%f,%f,%f,%f,%f,%f,%f],p=[%f,%f,%f]", R(0, 0), R(0, 1), R(0, 2), R(1, 0), R(1, 1), R(1, 2), R(2, 0), R(2, 1), R(2, 2), p[0], p[1], p[2]);
		return tmp_cstr;
	}

	Eigen::Matrix3f R;
	Eigen::Vector3f p;
};


#endif // RT_H