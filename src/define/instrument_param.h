#ifndef INSTRUMENT_PARAM_H
#define INSTRUMENT_PARAM_H

class InstrumentParam
{
public:
	InstrumentParam()
		: L1(40)
		, Lr(20)
		, L2(60)
		, LS2(0)
		, LSr(0)
		, radius(4)
		, gamma3(0)
	{}
	InstrumentParam(float L1, float Lr, float L2, float LS2, float LSr, float r, float gamma3 = 0)
		: L1(L1)
		, Lr(Lr)
		, L2(L2)
		, LS2(LS2)
		, LSr(LSr)
		, radius(r)
		, gamma3(gamma3)
	{}
	InstrumentParam& operator= (const InstrumentParam &param)
	{
		L1 = param.L1;
		Lr = param.Lr;
		L2 = param.L2;
		radius = param.radius;
		LS2 = param.LS2;
		LSr = param.LSr;
		gamma3 = param.gamma3;

		return *this;
	}

	float getL1()       const	{   return L1;	    }
	
    float getLr()       const	{   return Lr;	    }
	
    float getL2()       const	{   return L2;	    }
	
    float getLS2()      const	{   return LS2;	    }
	
    float getLSr()      const	{   return LSr;	    }
	
    float getRadius()   const	{   return radius;    }
	
    float getGamma3()   const	{   return gamma3;    }

private:
	float L1;		// would also used to represent LS1
	float Lr;
	float L2;	
	float LS2;		// LS2 of short bend instrument
	float LSr;		// LSr of short bend instrument
	float radius;

	float gamma3;
};

#endif // INSTRUMENT_PARAM_H