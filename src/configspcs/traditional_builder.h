#ifndef LIB_INSTRUMENT_TRADITIONAL_BUILDER_H_LF
#define LIB_INSTRUMENT_TRADITIONAL_BUILDER_H_LF
#include "single_port_builder.h"

class TraditionalBuilder :	public SinglePortBuilder
{
public:
	TraditionalBuilder(const Instrument* instrument)
		: SinglePortBuilder(instrument)
	{}

	void buildConfigSpcs(ConfigSpcs &q) override;

protected:
	ConfigSpcs& buildC4(ConfigSpcs &q, float Lb);
};

#endif // LIB_INSTRUMENT_TRADITIONAL_BUILDER_H_LF