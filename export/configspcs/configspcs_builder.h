/**
 * @file configspcs_builder.h
 *
 * @author longfei Wang(longfei.wang@sjtu.edu.cn)
 *
 * @brief This class is designed to construct configuration space that is
 * usefull to calculate the forwad kinematics.
 *
 * @version 1.1
 *
 * @date 2020-01-01
 * 
 * @copyright Copyright (c) 2020
 * 
 * Revise:
 * 2021-08-25. The previous version use the object of class Instrument
 * as the input arguments for the constructor, like ConfigSpcsBuilder(const
 * Instrument*), however, this designation make the class more exclusive.
 * To deal with this problem, the input arguments for the constructor is
 * omitted, and two required object, 'InstrumentConfig' and 'InstrumentParam'
 * are set as input arguments to the only interface 'buildConfigSpcs()';
 */
#ifndef LIB_INSTRUMENT_CONFIGSPCS_BUILDER_H_LF
#define LIB_INSTRUMENT_CONFIGSPCS_BUILDER_H_LF
#include "./configspcs.h"
#include "../instrument/instrument_config.h"
#include "../instrument/instrument_param.h"

extern const float MIN_LENGTH;

class ConfigSpcsBuilder
{
public:
    ConfigSpcsBuilder();
	virtual ~ConfigSpcsBuilder() {}
	
    virtual void buildConfigSpcs(const InstrumentConfig&, const InstrumentParam&, ConfigSpcs&) {}

protected:
    ConfigSpcs& buildC1(const InstrumentConfig& config, float L2, ConfigSpcs &q);
    ConfigSpcs& buildC2(const InstrumentConfig& config, const InstrumentParam& param, float Lr, ConfigSpcs &q);
};


#endif // LIB_INSTRUMENT_CONFIGSPCS_BUILDER_H_LF
