#ifndef LIB_INSTRUMENT_SINGLE_PORT_BUILDER_H_LF
#define LIB_INSTRUMENT_SINGLE_PORT_BUILDER_H_LF
#include "../../export/configspcs/configspcs_builder.h"

class SinglePortBuilder : public ConfigSpcsBuilder
{
public:
    SinglePortBuilder()
        : ConfigSpcsBuilder()
	{}
	virtual ~SinglePortBuilder() {}

    virtual void buildConfigSpcs(const InstrumentConfig&, const InstrumentParam&, ConfigSpcs&) override;

protected:
    ConfigSpcs& buildC3(const InstrumentConfig& config, const InstrumentParam& param, float L1, ConfigSpcs &q);
};

#endif // LIB_INSTRUMENT_SINGLE_PORT_BUILDER_H_LF
