#include "configspcs_factory.h"
#include "traditional_builder.h"
#include "single_port_builder.h"

std::unique_ptr<ConfigSpcsBuilder> ConfigSpcsFactory::createConfigSpcsBuilder(const SurgicalToolType& type)
{
    switch (type)
	{
    case SURGICAL_TOOL_TYPE_UNKNOWN:
        return std::unique_ptr<ConfigSpcsBuilder>(new ConfigSpcsBuilder());
    case SURGICAL_TOOL_TYPE_ENDOSCOPIC:
        return std::unique_ptr<ConfigSpcsBuilder>(new TraditionalBuilder());
    case SURGICAL_TOOL_TYPE_SP_TOOL:
        return std::unique_ptr<ConfigSpcsBuilder>(new SinglePortBuilder());
	}
	return nullptr;
}
