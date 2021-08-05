#include "../../export/configspcs/configspcs_builder.h"
#include "../../export/instrument/instrument.h"

using mmath::continuum::ConfigSpc;

// length of each stem can not be zero in GL
const float MIN_LENGTH = 0.0f;					

ConfigSpcsBuilder::ConfigSpcsBuilder(const Instrument* instrument)
	: instrument(instrument)
{
}

ConfigSpcs& ConfigSpcsBuilder::buildC1(ConfigSpcs &q, float L2)
{
	auto config = instrument->getConfig();
	auto param = instrument->getParam();

	q.add(ConfigSpc(0, config.getPhi(), MIN_LENGTH));
	q.add(ConfigSpc(config.getTheta2(), config.getDelta2(), L2, true));
	return q;
}

ConfigSpcs& ConfigSpcsBuilder::buildC2(ConfigSpcs &q, float Lr)
{
	auto config = instrument->getConfig();
	auto param = instrument->getParam();

	q.add(ConfigSpc(0, config.getPhi(), MIN_LENGTH));
	q.add(ConfigSpc(0, 0, Lr, false));
	q.add(ConfigSpc(config.getTheta2(), config.getDelta2(), param.getL2(), true));
	return q;
}
