#ifndef LIB_INSTRUMENT_PARAM_H_LF
#define LIB_INSTRUMENT_PARAM_H_LF
#include <cstdio>

class InstrumentParam
{
public:
    InstrumentParam(float L1 = 40, float Lr = 20, float L2 = 60, float Lg = 20, float r = 4, float gamma3 = 0)
		: L1(L1)
		, Lr(Lr)
		, L2(L2)
		, Lg(Lg)
		, radius(r)
		, gamma3(gamma3)
	{}
	InstrumentParam& operator= (const InstrumentParam &param)
	{
		L1 = param.L1;
		Lr = param.Lr;
		L2 = param.L2;
		Lg = param.Lg;
		radius = param.radius;
		gamma3 = param.gamma3;

		return *this;
	}

	float getL1()       const	{   return L1;	    }
	
    float getLr()       const	{   return Lr;	    }
	
    float getL2()       const	{   return L2;	    }

	float getLg()       const	{   return Lg;	    }
	
    float getRadius()   const	{   return radius;    }
	
    float getGamma3()   const	{   return gamma3;    }

    char* c_str() const {
        static char info[128];
        sprintf(info, "L1:%f,Lr:%f,L2:%f,Lg:%f,radius:%f,gamma3:%f",
                L1, Lr, L2, Lg, radius, gamma3);
        return info;
    }

private:
	float L1;		// would also used to represent LS1
	float Lr;
	float L2;
	float Lg;
	float radius;
	float gamma3;
};

#endif // LIB_INSTRUMENT_PARAM_H_LF
