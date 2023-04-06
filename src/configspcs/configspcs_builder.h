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
#ifndef LIB_SURGICAL_TOOL_CONFIGSPCS_BUILDER_H_LF
#define LIB_SURGICAL_TOOL_CONFIGSPCS_BUILDER_H_LF
#include "../../include/define/configspcs.h"
#include "../../include/define/surgical_tool_config.h"
#include "../../include/define/surgical_tool_param.h"

extern const float MIN_LENGTH;

class ConfigSpcsBuilder
{
public:
    ConfigSpcsBuilder();
	virtual ~ConfigSpcsBuilder() {}
	
    virtual void buildConfigSpcs(const SurgicalToolConfig&, const SurgicalToolParam&, ConfigSpcs&) {}

protected:
    ConfigSpcs& buildC1(const SurgicalToolConfig& config, float L2, ConfigSpcs &q);
    ConfigSpcs& buildC2(const SurgicalToolConfig& config, const SurgicalToolParam& param, float Lr, ConfigSpcs &q);
};


#endif // LIB_SURGICAL_TOOL_CONFIGSPCS_BUILDER_H_LF
