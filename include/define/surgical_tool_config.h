#ifndef LIB_SURGICAL_TOOL_CONFIG_H_LF
#define LIB_SURGICAL_TOOL_CONFIG_H_LF
#include <cstdio>
#include <cmath>
#include <iostream>
#include <iomanip>

enum SurgicalToolConfigID
{
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
        : theta1_max(theta1_max)
        , theta2_max(theta2_max)
    {
        safeSet(CONFIG_L_INSERT, L_insert);
        safeSet(CONFIG_PHI, phi);
        safeSet(CONFIG_THETA1, theta1);
        safeSet(CONFIG_DELTA1, delta1);
        safeSet(CONFIG_THETA2, theta2);
        safeSet(CONFIG_DELTA2, delta2);
    }


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


    void safeSet(SurgicalToolConfigID idx, float value)
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


    void safeAdd(SurgicalToolConfigID idx, float value)
    {
        switch (idx)
        {
        case 0:
            L_insert += value;
            break;
        case 1:
            phi += value;
            break;
        case 2:
            theta1 += value;
            if(theta1 < 0.f) {
                theta1 *= -1.f;
                safeAdd(CONFIG_DELTA2, M_PI);
            }
            else if(abs(theta1) < 1e-5) delta1 = 0;
            else if(theta1 > theta1_max) theta1 = theta1_max;
            break;
        case 3:
            delta1 += value;
            if (delta1 > M_PI)           delta1 -= 2.f*M_PI;
            else if (delta1 < -M_PI)     delta1 += 2.f*M_PI;
            break;
        case 4:
            theta2 += value;
            if(theta2 < 0.f) {
                theta2 *= -1.f;
                safeAdd(CONFIG_DELTA2, M_PI);
            }
            else if(abs(theta2) < 1e-5) delta2 = 0;
            else if(theta2 > theta2_max) theta2 = theta1_max;
            break;
        case 5:
            delta2 += value;
            if (delta2 > M_PI)           delta2 -= 2.f*M_PI;
            else if (delta2 < -M_PI)     delta2 += 2.f*M_PI;
            break;
        default:
            break;
        }
    }



    const char* info() const {
        static char info[128];
        sprintf(info, "L_insert:%f,phi:%f,theta1:%f,delta1:%f,theta2:%f"
                      ",delta2:%f. theta1_max:%f, theta2_max:%f",
                L_insert, phi, theta1, delta1, theta2, delta2,
                theta1_max, theta2_max);
        return info;
    }


    friend std::ostream& operator<< (std::ostream &os,
                                     const SurgicalToolConfig& config)
    {
        int w = 12;
        std::cout.setf(std::ios::fixed);
        os << std::setw(w) << std::setprecision(6) << config.L_insert << " "
           << std::setw(w) << config.phi << " "
           << std::setw(w) << config.theta1 << " "
           << std::setw(w) << config.delta1 << " "
           << std::setw(w) << config.theta2 << " "
           << std::setw(w) << config.delta2;
        std::cout.unsetf(std::ios::fixed);
        return os;
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


