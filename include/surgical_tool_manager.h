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
 * 2021.7.30 Complete this library and create this file.
 * 2022.6.29 Refactor the code by adding the manager to manage single
 * port surgery mode.
 * 
 * -------------------------------------------------------------------*/
#ifndef SURGICAL_TOOL_MANAGER_H_LF
#define SURGICAL_TOOL_MANAGER_H_LF
#include <array>
#include <memory>
#include <lib_math/lib_math.h>
#include "./define/configspcs.h"
#include "./define/surgical_tool_type.h"
#include "./define/surgical_tool_param.h"
#include "./define/surgical_tool_config.h"
#include "./define/surgical_tool_kine.h"

class SurgicalTool;

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
 * @brief The SurgicalToolManager class is designed to manage multiple surgical
 * tools which are consisted with continuum.
 */
class SurgicalToolManager
{
public:
    SurgicalToolManager();

    void initialize(SurgicalToolIdx tool_id, const SurgicalToolParam &param,
                    SurgicalToolType tool_type, uint8_t gripper_type);
    void updateConfig(SurgicalToolIdx tool_id, const SurgicalToolConfig &config);
    void updateTarget(SurgicalToolIdx tool_id, const mmath::Pose &pose);
    void setGripperAngle(SurgicalToolIdx tool_id, float type);
    void setTao(SurgicalToolIdx tool_id, float tao);

    const SurgicalToolType&   getType(SurgicalToolIdx tool_id) const;
    const SurgicalToolConfig& getConfig(SurgicalToolIdx tool_id) const;
    const SurgicalToolParam&  getParam(SurgicalToolIdx tool_id) const;
    const mmath::Pose&    	  getBasePose(SurgicalToolIdx tool_id) const;
    const mmath::Pose&		  getEndPose(SurgicalToolIdx tool_id) const;
    const ConfigSpcs& 		  getConfigSpcs(SurgicalToolIdx tool_id) const;
    const TaskSpc& 			  getTaskSpc(SurgicalToolIdx tool_id) const;

private:
    std::array<std::shared_ptr<SurgicalTool>, MAX_TOOL_NUM> _tools;
};

#endif // SURGICAL_TOOL_MANAGER_H_LF
