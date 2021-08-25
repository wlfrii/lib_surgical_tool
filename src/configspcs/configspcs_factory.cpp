#include "configspcs_factory.h"
#include "traditional_builder.h"
#include "single_port_builder.h"

std::unique_ptr<ConfigSpcsBuilder> ConfigSpcsFactory::createConfigSpcsBuilder(const InstrumentType& type)
{
    switch (type)
	{
	case INSTRUMENT_TYPE_UNKNOWN:
        return std::unique_ptr<ConfigSpcsBuilder>(new ConfigSpcsBuilder());
	case INSTRUMENT_TYPE_LAPAROSCOPE:
        return std::unique_ptr<ConfigSpcsBuilder>(new TraditionalBuilder());
	case INSTRUMENT_TYPE_SP_TOOL:
        return std::unique_ptr<ConfigSpcsBuilder>(new SinglePortBuilder());
	}
	return nullptr;
}
