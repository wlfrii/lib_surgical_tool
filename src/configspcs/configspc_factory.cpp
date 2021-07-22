#include "configspcs_factory.h"
#include "traditional_builder.h"
#include "single_port_builder.h"

std::unique_ptr<ConfigSpcsBuilder> ConfigSpcsFactory::buildConfigSpcs(const Instrument&instrument)
{
	switch (instrument.getInstrumentType())
	{
	case INSTRUMENT_UNKNOWN:	// 8
		return std::unique_ptr<ConfigSpcsBuilder>(new ConfigSpcsBuilder(instrument));
	case TRADITIONAL_TOOL:		// 0
	case VISION_TRADITIONAL:	// 7
	case VISION_LAPARO:			// 5
		return std::unique_ptr<ConfigSpcsBuilder>(new TraditionalBuilder(instrument));
	case SINGLE_BLADDER:		// 4 单孔臂, 这两种臂结构上都是L2,Lr,L1,L
	case SINGLE_LAPARO:			// 3
	case VISION_BLADDER:		// 6
	case SINGLE_LXARM:			// 5
		return std::unique_ptr<ConfigSpcsBuilder>(new SinglePortBuilder(instrument));
	}
	return nullptr;
}