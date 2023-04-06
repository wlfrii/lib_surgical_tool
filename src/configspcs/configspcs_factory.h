#ifndef LIB_SURGICAL_TOOL_CONFIGSPCS_FACTORY_H_LF
#define LIB_SURGICAL_TOOL_CONFIGSPCS_FACTORY_H_LF
#include <memory>
#include "../../include/define/surgical_tool_type.h"

class ConfigSpcsBuilder;

class ConfigSpcsFactory
{
protected:
	ConfigSpcsFactory() {}
	~ConfigSpcsFactory() {}
public:
    static std::unique_ptr<ConfigSpcsBuilder> createConfigSpcsBuilder(const SurgicalToolType&);
};

#endif // LIB_SURGICAL_TOOL_CONFIGSPCS_FACTORY_H_LF
