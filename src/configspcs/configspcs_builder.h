#ifndef CONFIGSPCS_BUILDER_H
#define CONFIGSPCS_BUILDER_H
#include "../instrument.h"
#include "configspc.h"

extern const float MIN_LENGTH;

class ConfigSpcsBuilder
{
public:
	ConfigSpcsBuilder(const Instrument& ins)
		: instrument(ins)
	{}
	virtual ~ConfigSpcsBuilder() {}

	void distributeInitConfigs(ConfigSpcs &);
	
	virtual void distributeConfigs(ConfigSpcs &q) {}

protected:
	ConfigSpcs& buildC1(ConfigSpcs &q, float L2);
	ConfigSpcs& buildC2(ConfigSpcs &q, float Lr);

protected:
	const Instrument &instrument;
};

#endif // CONFIGSPCS_BUILDER_H