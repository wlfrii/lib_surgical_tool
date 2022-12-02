/**--------------------------------------------------------------------
 *																		
 *   			    Continuum Surgical Tool Library 					
 *																		
 * Description:													
 * This file is header file of lib surgical tool. You can redistribute it 
 * and or modify it to construct your own project. It is wellcome to use 
 * this library in your scientific research work.
 * 
 * @file 		surgical_tool_manager.h 
 * 
 * @brief 		The header file for the library
 * 
 * @author		Longfei Wang
 * 
 * @version		1.0.0
 * 
 * @date		2019/07/25
 * 
 * @license		
 * 
 * Copyright (C) 2019 Longfei Wang.
 * 
 * --------------------------------------------------------------------
 * Change History:                        
 * 
 * 2022.11.24 Complete the doxygen comments.
 * 2022.6.29 Refactor the code by adding the manager to manage single
 * port surgery mode.
 * 2021.7.30 Complete this library and create this file.
 * -------------------------------------------------------------------*/
#ifndef SURGICAL_TOOL_MANAGER_H_LF
#define SURGICAL_TOOL_MANAGER_H_LF
#include <array>
#include <memory>
#include <lib_math/lib_math.h>
#include "surgical_tool.h"

constexpr unsigned char MAX_TOOL_NUM = 4;

// Tool index
enum SurgicalToolIdx
{
    TOOL1 = 0,
    TOOL2 = 1,
    TOOL3 = 2,
    ENDO  = 2,
    TOOL4 = 3
};

/**
 * @brief The SurgicalToolManager class is designed to manage 4 surgical
 * tools which are consisted with continuum.
 * NOTE, the default setting for the 4-surgical tools is single port access.
 * Thus, the poses for these tools are all w.r.t frame {Trocar}.
 */
class SurgicalToolManager
{
public:
    SurgicalToolManager();

    /**
     * @brief Initialize the surgical tool object with tool structure parameters
     * and types of of the tool and its gripper.
     * 
     * @param tool_id The tool index
     * @param param  The structure parameters
     * @param tool_type  The type of the surgical tool, see SurgicalToolType
     * @param gripper_type   The type of the gripper
     */
    void initialize(SurgicalToolIdx tool_id, const SurgicalToolParam &param,
                    SurgicalToolType tool_type, uint8_t gripper_type);


    /**
     * @brief Update the configures of the surgical tool. (forward kinematics)
     * 
     * @param tool_id The tool index
     * @param config
     */                
    void updateConfig(SurgicalToolIdx tool_id, const SurgicalToolConfig &config);


    /**
     * @brief Update the target of the surgical tool. (inverse kinematics).
     * NOTE, the target should w.r.t the base frame of the surgical tool.
     * 
     * @param tool_id The tool index
     * @param pose 
     */
    void updateTarget(SurgicalToolIdx tool_id, const mmath::Pose &pose);


    /**
     * @brief Set the gripper angle, if it has a active gripper.
     * 
     * @param tool_id The tool index
     * @param angle 
     */
    void setGripperAngle(SurgicalToolIdx tool_id, float angle);


    /**
     * @brief Set the tao, which is a rotated angle w.r.t Endoscope's base.
     * 
     * @param tool_id The tool index
     * @param tao 
     */
    void setTao(SurgicalToolIdx tool_id, float tao);


    /**
     * @brief Reset this surigcal tool as unknow tool.
     * 
     * @param tool_id The tool index
     */
	void reset(SurgicalToolIdx tool_id);


    /**
	 * @brief Get the Surgical Tool Type
	 * 
     * @param tool_id The tool index
	 * @return const SurgicalToolType& 
	 */
    const SurgicalToolType&   getType(SurgicalToolIdx tool_id) const;


    /**
     * @brief Get the 2-segment continuum joint Configuration
     * 
     * @param tool_id The tool index
     * @return const SurgicalToolConfig& 
     */
    const SurgicalToolConfig& getConfig(SurgicalToolIdx tool_id) const;


    /**
     * @brief Get the Structure Parameters object
     * 
     * @param tool_id The tool index
     * @return const SurgicalToolParam& 
     */
    const SurgicalToolParam&  getParam(SurgicalToolIdx tool_id) const;


    /**
     * @brief Get the Base Pose of the continuum surgical tool w.r.t {Trocar}
     * 
     * @param tool_id The tool index
     * @return const mmath::Pose& 
     */
    const mmath::Pose&    	  getBasePose(SurgicalToolIdx tool_id) const;


    /**
     * @brief Get the End Pose of the continuum surgical tool w.r.t {Trocar}
     * 
     * @param tool_id The tool index
     * @return const mmath::Pose& 
     */
    const mmath::Pose&		  getEndPose(SurgicalToolIdx tool_id) const;


    /**
     * @brief Get the forward kinematics of the continuum surgical tool.
     * Thus, the returned pose is w.r.t the base frame.
     * 
     * @param tool_id The tool index
     * @return mmath::Pose& 
     */
    const mmath::Pose&        getEnd2BasePose(SurgicalToolIdx tool_id) const;


    /**
     * @brief Get current Configuration Spaces
     * 
     * @param tool_id The tool index
     * @return const ConfigSpcs& 
     */
    const ConfigSpcs& 		  getConfigSpcs(SurgicalToolIdx tool_id) const;


    /**
     * @brief Get current Task Sapce
     * 
     * @param tool_id The tool index
     * @return const TaskSpc& 
     */
    const TaskSpc& 			  getTaskSpc(SurgicalToolIdx tool_id) const;

private:
    std::array<std::shared_ptr<SurgicalTool>, MAX_TOOL_NUM> _tools;
};

#endif // SURGICAL_TOOL_MANAGER_H_LF
