#ifndef LIB_INSTRUMENT_CONFIG_H_LF
#define LIB_INSTRUMENT_CONFIG_H_LF

class InstrumentConfig
{
public:
	InstrumentConfig(float L_insert = 0, float phi = 0, float theta1 = 0, float delta1 = 0, float theta2 = 0, float delta2 = 0)
		: L_insert(L_insert)
		, phi(phi)
		, theta1(theta1)
		, delta1(delta1)
		, theta2(theta2)
		, delta2(delta2)
	{}

	InstrumentConfig(const InstrumentConfig& other)
	{
		L_insert = other.L_insert;
		phi = other.phi;
		theta1 = other.theta1;
		delta1 = other.delta1;
		theta2 = other.theta2;
		delta2 = other.delta2;
	}

	void set(float value, unsigned char idx)
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

	float getLinsert()  const	{	return L_insert;}

	float getPhi()      const	{	return phi;	    }

	float getTheta1()   const	{	return theta1;	}

	float getDelta1()   const	{	return delta1;	}

	float getTheta2()   const	{	return theta2;	}

	float getDelta2()   const	{	return delta2;	}

private:
	float L_insert;
	float phi;
	float theta1;
	float delta1;
	float theta2;
	float delta2;
};

#endif // LIB_INSTRUMENT_CONFIG_H_LF
