#ifndef SINGLE_PORT_BUILDER_H
#define SINGLE_PORT_BUILDER_H
#include "configspcs_builder.h"

class SinglePortBuilder : public ConfigSpcsBuilder
{
public:
	SinglePortBuilder(const Instrument& ins)
		: ConfigSpcsBuilder(ins)
	{}
	virtual ~SinglePortBuilder() {}

	virtual void distributeConfigs(ConfigSpcs &q) override;

protected:
	ConfigSpcs& buildC3(ConfigSpcs &q, float L1);
};

#endif // SINGLE_PORT_BUILDER_H