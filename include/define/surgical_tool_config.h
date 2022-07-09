#ifndef LIB_SURGICAL_TOOL_CONFIG_H_LF
#define LIB_SURGICAL_TOOL_CONFIG_H_LF
#include <cstdio>
#include <cmath>

enum SurgicalToolConfigID{
    CONFIG_L_INSERT = 0,
    CONFIG_PHI      = 1,
    CONFIG_THETA1   = 2,
    CONFIG_DELTA1   = 3,
    CONFIG_THETA2   = 4,
    CONFIG_DELTA2   = 5
};


class SurgicalToolConfig
{
public:
    SurgicalToolConfig(float L_insert = 0, float phi = 0, float theta1 = 0,
                       float delta1 = 0, float theta2 = 0, float delta2 = 0,
                       float theta1_max = 3.1415926f / 2,
                       float theta2_max = 3.1415926f*2 / 3)
		: L_insert(L_insert)
		, phi(phi)
		, theta1(theta1)
		, delta1(delta1)
		, theta2(theta2)
		, delta2(delta2)
        , theta1_max(theta1_max)
        , theta2_max(theta2_max)
	{}


    SurgicalToolConfig(const SurgicalToolConfig& other)
	{
		L_insert = other.L_insert;
		phi = other.phi;
		theta1 = other.theta1;
		delta1 = other.delta1;
		theta2 = other.theta2;
		delta2 = other.delta2;

        theta1_max = other.theta1_max;
        theta2_max = other.theta2_max;
	}


    void set(float value, SurgicalToolConfigID idx)
	{
		switch (idx)
		{
		case 0:
			L_insert = value;
			break;
		case 1:
			phi = value;
			break;
		case 2:
			theta1 = value;
			break;
		case 3:
            if (value > M_PI)        value -= 2.f*M_PI;
            else if (value < -M_PI)  value += 2.f*M_PI;
            delta1 = value;
			break;
		case 4:
			theta2 = value;
			break;
		case 5:
            if (value > M_PI)        value -= 2.f*M_PI;
            else if (value < -M_PI)  value += 2.f*M_PI;
            delta2 = value;
			break;
		default:
			break;
		}
	}


    char* info() const {
        static char info[128];
        sprintf(info, "L_insert:%f,phi:%f,theta1:%f,delta1:%f,theta2:%f"
                      ",delta2:%f. theta1_max:%f, theta2_max:%f",
                L_insert, phi, theta1, delta1, theta2, delta2,
                theta1_max, theta2_max);
        return info;
    }


	float L_insert;
	float phi;
	float theta1;
	float delta1;
	float theta2;
	float delta2;

    float theta1_max;
    float theta2_max;
};

#endif // LIB_SURGICAL_TOOL_CONFIG_H_LF
