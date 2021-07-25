#ifndef CONFIGSPCS_FACTORY_H
#define CONFIGSPCS_FACTORY_H
#include <memory>
#include "configspcs_builder.h"

class ConfigSpcsFactory
{
protected:
	ConfigSpcsFactory() {}
	~ConfigSpcsFactory() {}
public:
	static std::unique_ptr<ConfigSpcsBuilder> buildConfigSpcs(const Instrument&);
};

#endif // CONFIGSPCS_FACTORY_H