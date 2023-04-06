#include "configspcs_builder.h"


using mmath::continuum::ConfigSpc;

// length of each stem can not be zero in GL
const float MIN_LENGTH = 0.00f;

ConfigSpcsBuilder::ConfigSpcsBuilder()
{
}


ConfigSpcs& ConfigSpcsBuilder::buildC1(const SurgicalToolConfig& config,
                                       float L2, ConfigSpcs &q)
{
    q.add(ConfigSpc(0, config.phi, MIN_LENGTH));
    q.add(ConfigSpc(config.theta2, config.delta2, L2, true));
    q.space_type = ConfigSpaceType::C1;
	return q;
}


ConfigSpcs& ConfigSpcsBuilder::buildC2(const SurgicalToolConfig& config, const SurgicalToolParam& param, float Lr, ConfigSpcs &q)
{
    q.add(ConfigSpc(0, config.phi, MIN_LENGTH));
	q.add(ConfigSpc(0, 0, Lr, false));
    q.add(ConfigSpc(config.theta2, config.delta2, param.getL2(), true));
    q.space_type = ConfigSpaceType::C2;
	return q;
}
