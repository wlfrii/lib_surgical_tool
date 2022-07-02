#ifndef LIB_INSTRUMENT_H_LF
#define LIB_INSTRUMENT_H_LF
#include "../configspcs/configspcs_builder.h"
#include "../../include/define/surgical_tool_param.h"
#include "../../include/define/surgical_tool_config.h"
#include "../../include/define/surgical_tool_type.h"
#include <memory>
#include <vector>
#include <lib_math/lib_math.h>
#include <Eigen/Dense>

class ConfigSpcsBuilder;

/**
 * @brief A class for describing continuum surgical tool.
 * 
 */
class SurgicalTool
{
public:
    SurgicalTool();
    ~SurgicalTool();

	/* Set() interfaces */
    void initialize(const SurgicalToolParam &param, uint8_t tool_type,
                    uint8_t gripper_type);
    void updateConfig(const SurgicalToolConfig &config);
	void setGripperAngle(float type);
    void setTao(float tao);
    void setBasePose(const mmath::Pose &pose);

	void reset();

	/* Get() interfaces */
    uint8_t	getSurgicalToolType() const;
    const	SurgicalToolConfig&	getConfig() const;
    const	SurgicalToolParam&	getParam() const;
    const	mmath::Pose&    	getBasePose() const;
    const	mmath::Pose&		getEndPose() const;
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

    SurgicalToolType      _type;
    SurgicalToolParam     _param;
    SurgicalToolConfig    _config;

    uint8_t             _gripper_type;
    float               _gripper_angle;

    mmath::Pose           _base_pose;
    mmath::Pose           _end_pose;
	
    std::unique_ptr<ConfigSpcsBuilder>	_config_spcs_builder;

    ConfigSpcs          _config_spcs;
    TaskSpc             _task_spc;

    std::vector<Eigen::Vector3f> _ws_clouds;
};


#endif // LIB_INSTRUMENT_H_LF

