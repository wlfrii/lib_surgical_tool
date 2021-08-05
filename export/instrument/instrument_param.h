#ifndef LIB_INSTRUMENT_PARAM_H_LF
#define LIB_INSTRUMENT_PARAM_H_LF

class InstrumentParam
{
public:
	InstrumentParam()
		: L1(40)
		, Lr(20)
		, L2(60)
		, Lg(0)
		, radius(4)
		, gamma3(0)
	{}
	InstrumentParam(float L1, float Lr, float L2, float Lg, float r, float gamma3 = 0)
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

private:
	float L1;		// would also used to represent LS1
	float Lr;
	float L2;
	float Lg;
	float radius;
	float gamma3;
};

#endif // LIB_INSTRUMENT_PARAM_H_LF