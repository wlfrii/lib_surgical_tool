#include "single_port_builder.h"
#include "../../export/instrument/instrument.h"

extern const float MIN_LENGTH;

using mmath::continuum::ConfigSpc;

void SinglePortBuilder::buildConfigSpcs(ConfigSpcs &q)
{
	auto config = instrument->getConfig();
	auto param = instrument->getParam();

	float L_insert = config.getLinsert();

	if (L_insert <= 0)
		return;

	float L_C1_max = param.getL2();
	float L_C2_max = L_C1_max + param.getLr();

	if (L_insert <= L_C1_max)
		q = buildC1(q, L_insert);
	else if (L_insert <= L_C2_max)
		q = buildC2(q, L_insert - L_C1_max);
	else
		q = buildC3(q, L_insert - L_C2_max);
}

ConfigSpcs& SinglePortBuilder::buildC3(ConfigSpcs &q, float L1)
{
	auto config = instrument->getConfig();
	auto param = instrument->getParam();

	q.add(ConfigSpc(0, config.getPhi(), MIN_LENGTH));
	q.add(ConfigSpc(config.getTheta1(), config.getDelta1(), L1, true));
	q.add(ConfigSpc(0, 0, param.getLr(), false));
	q.add(ConfigSpc(config.getTheta2(), config.getDelta2(), param.getL2(), true));
	return q;
}