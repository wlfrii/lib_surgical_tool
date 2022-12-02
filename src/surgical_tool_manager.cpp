#include "../include/surgical_tool_manager.h"
#include "./define/marray.h"


namespace  {

// Max surgical tool number
//constexpr unsigned char MAX_TOOL_NUM = 4;
//std::array<std::shared_ptr<SurgicalTool>, MAX_TOOL_NUM> _tools;

/* Single port distribution
 *        3
 *		2	4
 *		  1
 */
Eigen::Matrix3f I3 = Eigen::Matrix3f::Identity(3, 3);
// The pose with respect to the real single port trocar
mArray<mmath::Pose, MAX_TOOL_NUM> real_pose = {
    mmath::Pose(I3, { 0.0f, -7.85f, 0.f }),
    mmath::Pose(I3, { 7.66f, -1.7f, 0.f }),
    mmath::Pose(I3, { 0.0f, 5.8f, 0.f - 15 }),
    mmath::Pose(I3, { -7.66f, -1.7f,  0.f })
};

}


SurgicalToolManager::SurgicalToolManager()
{
    for(auto i = 0; i < MAX_TOOL_NUM; i++){
        _tools[i] = std::make_shared<SurgicalTool>(::real_pose.get());
    }
}


void SurgicalToolManager::initialize(SurgicalToolIdx tool_id, const SurgicalToolParam &param,
                SurgicalToolType tool_type, uint8_t gripper_type)
{
    _tools[tool_id]->initialize(param, tool_type, gripper_type);
}


void SurgicalToolManager::updateConfig(SurgicalToolIdx tool_id, const SurgicalToolConfig &config)
{
    _tools[tool_id]->updateConfig(config);
}


void SurgicalToolManager::updateTarget(SurgicalToolIdx tool_id, const mmath::Pose &pose)
{
    _tools[tool_id]->updateTarget(pose);
}


void SurgicalToolManager::setGripperAngle(SurgicalToolIdx tool_id, float type)
{
    _tools[tool_id]->setGripperAngle(type);
}


void SurgicalToolManager::setTao(SurgicalToolIdx tool_id, float tao)
{
    _tools[tool_id]->setTao(tao);
}


void SurgicalToolManager::reset(SurgicalToolIdx tool_id)
{
    _tools[tool_id]->reset();
}


const SurgicalToolType& SurgicalToolManager::getType(SurgicalToolIdx tool_id) const
{
    return _tools[tool_id]->getType();
}


const SurgicalToolConfig& SurgicalToolManager::getConfig(SurgicalToolIdx tool_id) const
{
    return _tools[tool_id]->getConfig();
}


const SurgicalToolParam& SurgicalToolManager::getParam(SurgicalToolIdx tool_id) const
{
    return _tools[tool_id]->getParam();
}


const mmath::Pose& SurgicalToolManager::getBasePose(SurgicalToolIdx tool_id) const
{
    return _tools[tool_id]->getBasePose();
}


const mmath::Pose& SurgicalToolManager::getEndPose(SurgicalToolIdx tool_id) const
{
    return _tools[tool_id]->getEndPose();
}


const mmath::Pose& SurgicalToolManager::getEnd2BasePose(SurgicalToolIdx tool_id) const
{
    return _tools[tool_id]->getEnd2BasePose();
}


const ConfigSpcs& SurgicalToolManager::getConfigSpcs(SurgicalToolIdx tool_id) const
{
    return _tools[tool_id]->getConfigSpcs();
}


const TaskSpc& SurgicalToolManager::getTaskSpc(SurgicalToolIdx tool_id) const
{
    return _tools[tool_id]->getTaskSpc();
}
