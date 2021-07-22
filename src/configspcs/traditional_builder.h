#ifndef TRADITIONAL_BUILDER_H
#define TRADITIONAL_BUILDER_H
#include "single_port_builder.h"

class TraditionalBuilder :	public SinglePortBuilder
{
public:
	TraditionalBuilder(const Instrument& ins)
		: SinglePortBuilder(ins)
	{}

	void distributeConfigs(ConfigSpcs &q) override;

protected:
	ConfigSpcs& buildC4(ConfigSpcs &q, float Lb);
};

#endif // TRADITIONAL_BUILDER_H