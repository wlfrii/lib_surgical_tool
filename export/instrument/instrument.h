#ifndef LIB_INSTRUMENT_H_LF
#define LIB_INSTRUMENT_H_LF
#include "../configspcs/configspcs_builder.h"
#include "./instrument_param.h"
#include "./instrument_config.h"
#include "./instrument_type.h"
#include <memory>
#include <lib_math/lib_math.h>

class ConfigSpcsBuilder;

/**
 * @brief A class for describing continuum Instrument.
 * 
 */
class Instrument
{
public:
	Instrument();
	~Instrument();

	/* Set() interfaces */
	void initialize(const InstrumentParam &, const InstrumentConfig &);
	void updateConfig(const InstrumentConfig &);
	
	void setInstrumentType(uint8_t type);				
	void setGripperType(uint8_t type);
	void setGripperAngle(float type);
	void setBaseRot(float angle);
	void setBasePose(const mmath::RT &rt);

	void reset();

	/* Get() interfaces */
	uint8_t	getInstrumentType() const;
	const	InstrumentConfig&	getConfig() const;
	const	InstrumentParam&	getParam() const;
	const	mmath::RT&			getBasePose() const;
	const	mmath::RT&			getEndPose() const;
	const 	ConfigSpcs& 		getConfigSpcs() const;
	const   TaskSpc& 			getTaskSpc() const;

private:
	void updateKinematics();

private:
	InstrumentParam  param;
	InstrumentConfig config;
	InstrumentType	 type;

	uint8_t	gripper_type;
	float   gripper_angle;

	mmath::RT      base_pose;
	mmath::RT      end_pose;
	
	std::unique_ptr<ConfigSpcsBuilder>	config_spcs_builder;

	ConfigSpcs 	config_spcs;
	TaskSpc 	task_spc;
};


#endif // LIB_INSTRUMENT_H_LF

