#include "single_port_builder.h"
#include "../tool/surgical_tool.h"

extern const float MIN_LENGTH;

using mmath::continuum::ConfigSpc;

void SinglePortBuilder::buildConfigSpcs(const SurgicalToolConfig& config, const SurgicalToolParam& param, ConfigSpcs &q)
{
    float L_insert = config.L_insert;

	if (L_insert <= 0)
		return;

    float L_C1_max = param.getL2();
    float L_C2_max = L_C1_max + param.getLr();

	if (L_insert <= L_C1_max)
        q = buildC1(config, L_insert, q);
	else if (L_insert <= L_C2_max)
        q = buildC2(config, param, L_insert - L_C1_max, q);
	else
        q = buildC3(config, param, L_insert - L_C2_max, q);
    // Add gripper
    q.add(ConfigSpc(0, 0, param.getLg(), false));
}

ConfigSpcs& SinglePortBuilder::buildC3(const SurgicalToolConfig &config, const SurgicalToolParam &param, float L1, ConfigSpcs &q)
{
    q.add(ConfigSpc(0, config.phi, MIN_LENGTH));
    q.add(ConfigSpc(config.theta1, config.delta1, L1, true));
    q.add(ConfigSpc(0, 0, param.getLr(), false));
    q.add(ConfigSpc(config.theta2, config.delta2, param.getL2(), true));
	return q;
}
