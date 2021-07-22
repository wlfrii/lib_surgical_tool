#include "configspcs_builder.h"

const float MIN_LENGTH = 0.01f;					// length of each stem can not be zero !!!

void ConfigSpcsBuilder::distributeInitConfigs(ConfigSpcs &q)
{
	q.push(ConfigSpc(0, instrument.getConfig().getPhi(), instrument.getConfig().getL()));
	q.push(ConfigSpc(instrument.getConfig().getTheta1(), instrument.getConfig().getDelta1(), instrument.getParam().getL1()));
	q.push(ConfigSpc(0, 0, instrument.getParam().getLr()));
	q.push(ConfigSpc(instrument.getConfig().getTheta2(), instrument.getConfig().getDelta2(), instrument.getParam().getL2()));
}


ConfigSpcs& ConfigSpcsBuilder::buildC1(ConfigSpcs &q, float L2)
{
	q.push(ConfigSpc(0, instrument.getConfig().getPhi(), MIN_LENGTH));
	q.push(ConfigSpc(instrument.getConfig().getTheta2(), instrument.getConfig().getDelta2(), L2, true, instrument.getTheta2Color()));
	return q;
}

ConfigSpcs& ConfigSpcsBuilder::buildC2(ConfigSpcs &q, float Lr)
{
	q.push(ConfigSpc(0, instrument.getConfig().getPhi(), MIN_LENGTH));
	q.push(ConfigSpc(0, 0, Lr, false, instrument.getBodyColor()));
	q.push(ConfigSpc(instrument.getConfig().getTheta2(), instrument.getConfig().getDelta2(), instrument.getParam().getL2(), true, instrument.getTheta2Color()));
	return q;
}
