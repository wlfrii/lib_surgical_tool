#ifndef CONFIGSPC_H_
#define CONFIGSPC_H_
#include "../define/array_repo.h"

class ConfigSpc
{
public:
	ConfigSpc(float theta, float delta, float len, bool bend = false)
		: theta(theta), delta(delta), length(len), is_bend(bend)
	{}

	ConfigSpc() : theta(0), delta(0), length(0), is_bend(false)
	{}

	float theta;
	float delta;
	float length;
	bool is_bend;
	//bool is_limit;
};


const int MAX_SECTION_COUNT = 9;
class ConfigSpcs : public ArrayRepo< ConfigSpc, MAX_SECTION_COUNT >
{
public:
	int push(const ConfigSpc &v)
	{
		return add(v);
	}
};

#endif //_CONFIGSPC_3D_DRAW_