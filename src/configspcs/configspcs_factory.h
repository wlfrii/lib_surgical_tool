#ifndef LIB_INSTRUMENT_CONFIGSPCS_FACTORY_H_LF
#define LIB_INSTRUMENT_CONFIGSPCS_FACTORY_H_LF
#include <memory>
#include "../../export/configspcs/configspcs_builder.h"

class Instrument;

class ConfigSpcsFactory
{
protected:
	ConfigSpcsFactory() {}
	~ConfigSpcsFactory() {}
public:
	static std::unique_ptr<ConfigSpcsBuilder> createConfigSpcsBuilder(const Instrument*);
};

#endif // LIB_MATH_CONTINUUM_CONFIGSPCS_FACTORY_H_LF