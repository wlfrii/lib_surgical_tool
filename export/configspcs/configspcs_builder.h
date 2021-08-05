#ifndef LIB_INSTRUMENT_CONFIGSPCS_BUILDER_H_LF
#define LIB_INSTRUMENT_CONFIGSPCS_BUILDER_H_LF
#include "./configspcs.h"

extern const float MIN_LENGTH;

class Instrument;

class ConfigSpcsBuilder
{
public:
	ConfigSpcsBuilder(const Instrument*);
	virtual ~ConfigSpcsBuilder() {}
	
	virtual void buildConfigSpcs(ConfigSpcs&) {}

protected:
	ConfigSpcs& buildC1(ConfigSpcs &q, float L2);
	ConfigSpcs& buildC2(ConfigSpcs &q, float Lr);

	const Instrument* 		instrument;
};


#endif // LIB_INSTRUMENT_CONFIGSPCS_BUILDER_H_LF