#ifndef LIB_SURGICAL_TOOL_TRADITIONAL_BUILDER_H_LF
#define LIB_SURGICAL_TOOL_TRADITIONAL_BUILDER_H_LF
#include "single_port_builder.h"

class TraditionalBuilder :	public SinglePortBuilder
{
public:
    TraditionalBuilder()
        : SinglePortBuilder()
	{}

    void buildConfigSpcs(const SurgicalToolConfig&, const SurgicalToolParam &, ConfigSpcs&) override;

protected:
    ConfigSpcs& buildC4(const SurgicalToolConfig& config, const SurgicalToolParam& param, float Lb, ConfigSpcs &q);
};

#endif // LIB_SURGICAL_TOOL_TRADITIONAL_BUILDER_H_LF
