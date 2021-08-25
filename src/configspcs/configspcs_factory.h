#ifndef LIB_INSTRUMENT_CONFIGSPCS_FACTORY_H_LF
#define LIB_INSTRUMENT_CONFIGSPCS_FACTORY_H_LF
#include <memory>
#include "../../export/configspcs/configspcs_builder.h"
#include "../../export/instrument/instrument_type.h"

class ConfigSpcsFactory
{
protected:
	ConfigSpcsFactory() {}
	~ConfigSpcsFactory() {}
public:
    static std::unique_ptr<ConfigSpcsBuilder> createConfigSpcsBuilder(const InstrumentType&);
};

#endif // LIB_MATH_CONTINUUM_CONFIGSPCS_FACTORY_H_LF
