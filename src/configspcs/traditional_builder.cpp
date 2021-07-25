#include "traditional_builder.h"

extern const float MIN_LENGTH;
// Substracting 10.0 is new requirement for vision arm. 2021.5.18
const float VISON_ARM_SINGLE_PORT_FEED = 240.f - 10.0;	// vison arm
void TraditionalBuilder::distributeConfigs(ConfigSpcs &q)
{
	float L_insert = instrument.getConfig().getL() - VISON_ARM_SINGLE_PORT_FEED;
	if (L_insert <= 0)
		return;

	float L_C1_max = instrument.getParam().getL2();
	float L_C2_max = L_C1_max + instrument.getParam().getLr();
	float L_C3_max = L_C2_max + instrument.getParam().getL1();

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
	q.push(ConfigSpc(0, instrument.getConfig().getPhi(), Lb, false));
	q.push(ConfigSpc(instrument.getConfig().getTheta1(), instrument.getConfig().getDelta1(), instrument.getParam().getL1(), true));
	q.push(ConfigSpc(0, 0, instrument.getParam().getLr(), false));
	q.push(ConfigSpc(instrument.getConfig().getTheta2(), instrument.getConfig().getDelta2(), instrument.getParam().getL2(), true));
	return q;
}
