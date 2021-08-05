#ifndef LIB_INSTRUMENT_SINGLE_PORT_BUILDER_H_LF
#define LIB_INSTRUMENT_SINGLE_PORT_BUILDER_H_LF
#include "../../export/configspcs/configspcs_builder.h"

class SinglePortBuilder : public ConfigSpcsBuilder
{
public:
	SinglePortBuilder(const Instrument* instrument)
		: ConfigSpcsBuilder(instrument)
	{}
	virtual ~SinglePortBuilder() {}

	virtual void buildConfigSpcs(ConfigSpcs &q) override;

protected:
	ConfigSpcs& buildC3(ConfigSpcs &q, float L1);
};

#endif // LIB_INSTRUMENT_SINGLE_PORT_BUILDER_H_LF