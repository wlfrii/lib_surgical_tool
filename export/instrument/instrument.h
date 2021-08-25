#ifndef LIB_INSTRUMENT_H_LF
#define LIB_INSTRUMENT_H_LF
#include "../configspcs/configspcs_builder.h"
#include "./instrument_param.h"
#include "./instrument_config.h"
#include "./instrument_type.h"
#include <memory>
#include <vector>
#include <lib_math/lib_math.h>
#include <Eigen/Dense>

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
	
	/**
	 * @brief Get the working space clouds of current instrument.
	 * 
	 * @return const std::vector<Eigen::Vector3f>& 
	 */
	const std::vector<Eigen::Vector3f>& getWSClouds() const;

private:
	void updateKinematics();
	void updateWorkingSpace();


    InstrumentParam     _param;
    InstrumentConfig    _config;
    InstrumentType      _type;

    uint8_t             _gripper_type;
    float               _gripper_angle;

    mmath::RT           _base_pose;
    mmath::RT           _end_pose;
	
    std::unique_ptr<ConfigSpcsBuilder>	_config_spcs_builder;

    ConfigSpcs          _config_spcs;
    TaskSpc             _task_spc;

    std::vector<Eigen::Vector3f> _ws_clouds;
};


#endif // LIB_INSTRUMENT_H_LF

