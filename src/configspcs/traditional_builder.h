#ifndef LIB_INSTRUMENT_TRADITIONAL_BUILDER_H_LF
#define LIB_INSTRUMENT_TRADITIONAL_BUILDER_H_LF
#include "single_port_builder.h"

class TraditionalBuilder :	public SinglePortBuilder
{
public:
    TraditionalBuilder()
        : SinglePortBuilder()
	{}

    void buildConfigSpcs(const InstrumentConfig&, const InstrumentParam&, ConfigSpcs&) override;

protected:
    ConfigSpcs& buildC4(const InstrumentConfig& config, const InstrumentParam& param, float Lb, ConfigSpcs &q);
};

#endif // LIB_INSTRUMENT_TRADITIONAL_BUILDER_H_LF
