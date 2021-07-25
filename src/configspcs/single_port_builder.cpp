#include "single_port_builder.h"

extern const float MIN_LENGTH;

void SinglePortBuilder::distributeConfigs(ConfigSpcs &q)
{
	float threshold = -(instrument.getParam().getL2() + instrument.getParam().getLr());
	float L_insert = instrument.getConfig().getL() - threshold;

	if (L_insert <= 0)
		return;

	float L_C1_max = instrument.getParam().getL2();
	float L_C2_max = L_C1_max + instrument.getParam().getLr();

	if (L_insert <= L_C1_max)
		q = buildC1(q, L_insert);
	else if (L_insert <= L_C2_max)
		q = buildC2(q, L_insert - L_C1_max);
	else
		q = buildC3(q, L_insert - L_C2_max);
}

ConfigSpcs& SinglePortBuilder::buildC3(ConfigSpcs &q, float L1)
{
	q.push(ConfigSpc(0, instrument.getConfig().getPhi(), MIN_LENGTH));
	q.push(ConfigSpc(instrument.getConfig().getTheta1(), instrument.getConfig().getDelta1(), L1, true));
	q.push(ConfigSpc(0, 0, instrument.getParam().getLr(), false));
	q.push(ConfigSpc(instrument.getConfig().getTheta2(), instrument.getConfig().getDelta2(), instrument.getParam().getL2(), true));
	return q;
}