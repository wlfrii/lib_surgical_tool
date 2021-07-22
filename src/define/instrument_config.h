#ifndef INSTRUMENT_CONFIG_H
#define INSTRUMENT_CONFIG_H

class InstrumentConfig
{
public:
	InstrumentConfig() 
		: L(30)
		, phi(0)
		, theta1(0)
		, delta1(0)
		, theta2(0)
		, delta2(0) 
	{}

	InstrumentConfig(float L, float phi, float theta1, float delta1, float theta2, float delta2)
		: L(L)
		, phi(phi)
		, theta1(theta1)
		, delta1(delta1)
		, theta2(theta2)
		, delta2(delta2)
	{}

	InstrumentConfig(const InstrumentConfig& other)
	{
		L = other.L;
		phi = other.phi;
		theta1 = other.theta1;
		delta1 = other.delta1;
		theta2 = other.theta2;
		delta2 = other.delta2;
	}

	InstrumentConfig& operator= (const InstrumentConfig &other)
	{
		L = other.L;
		phi = other.phi;
		theta1 = other.theta1;
		delta1 = other.delta1;
		theta2 = other.theta2;
		delta2 = other.delta2;
		return *this;
	}

	void set(float value, unsigned char idx)
	{
		switch (idx)
		{
		case 0:
			L = value;
			break;
		case 1:
			phi = value;
			break;
		case 2:
			theta1 = value;
			break;
		case 3:
			delta1 = -value;
			break;
		case 4:
			theta2 = value;
			break;
		case 5:
			delta2 = -value;
			break;
		default:
			break;
		}
	}

	float getL()        const	{	return L;	    }

	float getPhi()      const	{	return phi;	    }

	float getTheta1()   const	{	return theta1;	}

	float getDelta1()   const	{	return delta1;	}

	float getTheta2()   const	{	return theta2;	}

	float getDelta2()   const	{	return delta2;	}

private:
	float L;
	float phi;
	float theta1;
	float delta1;
	float theta2;
	float delta2;
};

#endif // INSTRUMENT_CONFIG_H