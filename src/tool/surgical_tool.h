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

    /**
     * @brief Initialize the surgical tool object with tool structure parameters
     * and types of of the tool and its gripper.
     * 
     * @param param  The structure parameters
     * @param tool_type  The type of the surgical tool, see SurgicalToolType
     * @param gripper_type   The type of the gripper (not completed!!!)
     */
    void initialize(const SurgicalToolParam &param, uint8_t tool_type,
                    uint8_t gripper_type);

    /**
     * @brief Update the configures of the surgical tool. (forward kinematics)
     * 
     * @param config
     */
    void updateConfig(const SurgicalToolConfig &config);

    /**
     * @brief Update the target of the surgical tool. (inverse kinematics).
     * NOTE, the target should w.r.t the base frame of the surgical tool.
     * 
     * @param pose 
     */
    void updateTarget(const mmath::Pose &pose);

    /**
     * @brief Set the gripper angle, if it has a active gripper.
     * 
     * @param type 
     */
	void setGripperAngle(float type);

    /**
     * @brief Set the tao, which is a rotated angle w.r.t Endoscope's base.
     * 
     * @param tao 
     */
    void setTao(float tao);

    /**
     * @brief Set the base pose, if it is not single port mode.
     * 
     * @param pose 
     */
    void setBasePose(const mmath::Pose &pose);

    /**
     * @brief Reset this surigcal tool as unknow tool.
     * 
     */
	void reset();

	/* Get() interfaces */
    const SurgicalToolType&   getType() const;
    const SurgicalToolConfig& getConfig() const;
    const SurgicalToolParam&  getParam() const;
    const mmath::Pose&    	  getBasePose() const;
    const mmath::Pose&		  getEndPose() const;
    const ConfigSpcs& 		  getConfigSpcs() const;
    const TaskSpc& 			  getTaskSpc() const;
	
	/**
	 * @brief Get the working space clouds of current instrument.
	 * 
	 * @return const std::vector<Eigen::Vector3f>& 
	 */
	const std::vector<Eigen::Vector3f>& getWSClouds() const;

private:
    void forwardKinematics();
    void inverseKinematics();

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

