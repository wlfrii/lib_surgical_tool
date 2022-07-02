#ifndef LIB_SURGICAL_TOOL_SINGLE_PORT_BUILDER_H_LF
#define LIB_SURGICAL_TOOL_SINGLE_PORT_BUILDER_H_LF
#include "configspcs_builder.h"

class SinglePortBuilder : public ConfigSpcsBuilder
{
public:
    SinglePortBuilder()
        : ConfigSpcsBuilder()
	{}
	virtual ~SinglePortBuilder() {}

    virtual void buildConfigSpcs(const SurgicalToolConfig &, const SurgicalToolParam &, ConfigSpcs&) override;

protected:
    ConfigSpcs& buildC3(const SurgicalToolConfig& config, const SurgicalToolParam& param, float L1, ConfigSpcs &q);
};

#endif // LIB_SURGICAL_TOOL_SINGLE_PORT_BUILDER_H_LF
