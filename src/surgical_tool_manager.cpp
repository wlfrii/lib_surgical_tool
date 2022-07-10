#include "../include/surgical_tool_manager.h"
#include "./tool/surgical_tool.h"

SurgicalToolManager::SurgicalToolManager()
{
    for(auto i = 0; i < MAX_TOOL_NUM; i++){
        _tools[i] = std::make_shared<SurgicalTool>();
    }
}


void SurgicalToolManager::initialize(ToolIdx tool_id, const SurgicalToolParam &param,
                SurgicalToolType tool_type, uint8_t gripper_type)
{
    _tools[tool_id]->initialize(param, tool_type, gripper_type);
}


void SurgicalToolManager::updateConfig(ToolIdx tool_id, const SurgicalToolConfig &config)
{
    _tools[tool_id]->updateConfig(config);
}


void SurgicalToolManager::updateTarget(ToolIdx tool_id, const mmath::Pose &pose)
{
    _tools[tool_id]->updateTarget(pose);
}


void SurgicalToolManager::setGripperAngle(ToolIdx tool_id, float type)
{
    _tools[tool_id]->setGripperAngle(type);
}


void SurgicalToolManager::setTao(ToolIdx tool_id, float tao)
{
    _tools[tool_id]->setTao(tao);
}


const SurgicalToolType& SurgicalToolManager::getType(ToolIdx tool_id) const
{
    return _tools[tool_id]->getType();
}


const SurgicalToolConfig& SurgicalToolManager::getConfig(ToolIdx tool_id) const
{
    return _tools[tool_id]->getConfig();
}


const SurgicalToolParam& SurgicalToolManager::getParam(ToolIdx tool_id) const
{
    return _tools[tool_id]->getParam();
}


const mmath::Pose& SurgicalToolManager::getBasePose(ToolIdx tool_id) const
{
    return _tools[tool_id]->getBasePose();
}


const mmath::Pose& SurgicalToolManager::getEndPose(ToolIdx tool_id) const
{
    return _tools[tool_id]->getEndPose();
}


const ConfigSpcs& SurgicalToolManager::getConfigSpcs(ToolIdx tool_id) const
{
    return _tools[tool_id]->getConfigSpcs();
}


const TaskSpc& SurgicalToolManager::getTaskSpc(ToolIdx tool_id) const
{
    return _tools[tool_id]->getTaskSpc();
}
