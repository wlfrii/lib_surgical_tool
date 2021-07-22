#ifndef INSTRUMENT_H
#define INSTRUMENT_H
#include "../du_builder.h"
#include "../configspcs/configspcs_builder.h"
#include "../define/instrument_param.h"
#include "../define/instrument_config.h"
#include "../define/instrument_type.h"
#include <memory>

class ConfigSpcsBuilder;

class Instrument
{
	friend DuBuilder;
public:
	Instrument();
	~Instrument();

	/* Set() interfaces */
	void initialize(const InstrumentParam &, const InstrumentConfig &);

	void updateConfig(const InstrumentConfig &);
	void updateLimitStatus(uint8_t, uint8_t, uint8_t, float, float, float, float);

	void setInstrumentType(uint8_t type);				//!< 设置 Instrument 类型，如传统臂，短弯臂等等
	void setGripperType(uint8_t type);
	void setGripperAngle(float type);
	void setArmCtrlState(uint8_t control_type);
	void setBaseRot(float angle);
	void setBasePose(const RT &rt);
	void setEndPoseMsg(float pose[16]);

	void reset();

	/* Get() interfaces */
	uint8_t	getInstrumentType() const;

	const	RT&	getBasePose() const;
	const	RT&	getEndPose() const;
	const	InstrumentConfig&	getConfig() const;
	const	InstrumentParam&	getParam() const;


	/**
	* Kinematics calculation for Simulation data creating.
	* The argument -q- did not set to const because of the theta would be revised once it is a negative value
	*/
	std::vector<Du*>& getDus();
	void updateDus(bool is_init = true);
	const ConfigSpcs& getConfigSpcs() const;

private:
	InstrumentParam  param;
	InstrumentConfig config;
	InstrumentType	 type;

	uint8_t	gripper_type;
	float   gripper_angle;

	RT      base_pose;				//!< base pose of instrument
	RT      end_pose;				//!< end pose of instrument
	
	// Revise 2020.11.27, Wanglf
	std::unique_ptr<ConfigSpcsBuilder>	config_builder;

	// Revise 2020.11.27, Wanglf
	DuBuilder*			du_builder;
	std::vector<Du*>	dus;		// store the 3D-clouds for opengl
	float				alpha;
	glm::vec3			default_color;

	// Revise 2020.11.27, Wanglf, for the status of moving and the corresponding
	// target point.
	bool	cannot_reach_target;
	RT		target;
	float	offset; // indicate the distance between current-end and target-end

	// Add, 2021.3.5, Wanglf
	ConfigSpcs config_spcs;

	// Add, 2021.7.17
	RT end_pose_msg;
};


#endif

