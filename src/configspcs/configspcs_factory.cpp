#include "configspcs_factory.h"
#include "traditional_builder.h"
#include "single_port_builder.h"
#include "../../export/instrument/instrument.h"

std::unique_ptr<ConfigSpcsBuilder> ConfigSpcsFactory::createConfigSpcsBuilder(
	const Instrument* instrument)
{
	switch (instrument->getInstrumentType())
	{
	case INSTRUMENT_TYPE_UNKNOWN:
		return std::unique_ptr<ConfigSpcsBuilder>(new ConfigSpcsBuilder(instrument));
	case INSTRUMENT_TYPE_LAPAROSCOPE:
		return std::unique_ptr<ConfigSpcsBuilder>(new TraditionalBuilder(instrument));
	case INSTRUMENT_TYPE_SP_TOOL:
		return std::unique_ptr<ConfigSpcsBuilder>(new SinglePortBuilder(instrument));
	}
	return nullptr;
}