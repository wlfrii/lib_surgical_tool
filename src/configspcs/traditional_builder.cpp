#include "traditional_builder.h"
#include "../../export/instrument/instrument.h"

// Substracting 10.0 is new requirement for vision arm. 2021.5.18
const float VISON_ARM_SINGLE_PORT_FEED = 240.f - 10.0;	// vison arm

using mmath::continuum::ConfigSpc;

void TraditionalBuilder::buildConfigSpcs(ConfigSpcs &q)
{
	auto config = instrument->getConfig();
	auto param = instrument->getParam();

	float L_insert = config.getLinsert();
	if (L_insert <= 0)
		return;

	float L_C1_max = param.getL2();
	float L_C2_max = L_C1_max + param.getLr();
	float L_C3_max = L_C2_max + param.getL1();

	if (L_insert <= L_C1_max)
		q = buildC1(q, L_insert);
	else if (L_insert <= L_C2_max)
		q = buildC2(q, L_insert - L_C1_max);
	else if (L_insert <= L_C3_max)
		q = buildC3(q, L_insert - L_C2_max);
	else
		q = buildC4(q, L_insert - L_C3_max);
}


ConfigSpcs& TraditionalBuilder::buildC4(ConfigSpcs &q, float Lb)
{
	auto config = instrument->getConfig();
	auto param = instrument->getParam();

	q.add(ConfigSpc(0, config.getPhi(), Lb, false));
	q.add(ConfigSpc(config.getTheta1(), config.getDelta1(), param.getL1(), true));
	q.add(ConfigSpc(0, 0, param.getLr(), false));
	q.add(ConfigSpc(config.getTheta2(), config.getDelta2(), param.getL2(), true));
	return q;
}
