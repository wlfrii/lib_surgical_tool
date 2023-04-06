#include "traditional_builder.h"
#include "../include/lib_surgical_tool/surgical_tool_config.h"
#include "../include/lib_surgical_tool/surgical_tool_param.h"


using mmath::continuum::ConfigSpc;

void TraditionalBuilder::buildConfigSpcs(const SurgicalToolConfig& config, const SurgicalToolParam& param, ConfigSpcs &q)
{
    float L_insert = config.L_insert;
	if (L_insert <= 0)
		return;

    float L_C1_max = param.getL2();
    float L_C2_max = L_C1_max + param.getLr();
    float L_C3_max = L_C2_max + param.getL1();

	if (L_insert <= L_C1_max)
        q = buildC1(config, L_insert, q);
	else if (L_insert <= L_C2_max)
        q = buildC2(config, param, L_insert - L_C1_max, q);
	else if (L_insert <= L_C3_max)
        q = buildC3(config, param, L_insert - L_C2_max, q);
	else
        q = buildC4(config, param, L_insert - L_C3_max, q);

    // Currently, only endoscope is traditional.
    q.add(ConfigSpc(0, param.getGamma3(), param.getLg(), false));
}


ConfigSpcs& TraditionalBuilder::buildC4(const SurgicalToolConfig &config, const SurgicalToolParam &param, float Lb, ConfigSpcs &q)
{
    q.add(ConfigSpc(0, config.phi, Lb, false));
    q.add(ConfigSpc(config.theta1, config.delta1, param.getL1(), true));
    q.add(ConfigSpc(0, 0, param.getLr(), false));
    q.add(ConfigSpc(config.theta2, config.delta2, param.getL2(), true));
    q.space_type = ConfigSpaceType::C4;
	return q;
}
