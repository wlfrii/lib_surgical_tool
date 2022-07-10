#include "surgical_tool.h"
#include "../configspcs/configspcs_factory.h"
#include "../define/marray.h"
#include "../include/define/surgical_tool_kine.h"
#include <utility>

namespace
{
    /* Single port distribution
     *        3
     *		2	4
     *		  1
	 */
	Eigen::Matrix3f I3 = Eigen::Matrix3f::Identity(3, 3);
	// The pose with respect to the real single port trocar
    mArray<mmath::Pose, 4> real_pose = {
        mmath::Pose(I3, { 0.0f, -7.85f, 0.f }),
        mmath::Pose(I3, { 7.66f, -1.7f, 0.f }),
        mmath::Pose(I3, { 0.0f, 5.8f, 0.f - 15 }),
        mmath::Pose(I3, { -7.66f, -1.7f,  0.f })
	};
}


SurgicalTool::SurgicalTool()
    : _type(SURGICAL_TOOL_TYPE_UNKNOWN)
    , _gripper_type(0)
    , _gripper_angle(0)
    , _base_pose(::real_pose.get())
    , _end_pose(_base_pose)
    , _config_spcs_builder(nullptr)
{
}


SurgicalTool::~SurgicalTool()
{

}


void SurgicalTool::initialize(const SurgicalToolParam &param,
                              SurgicalToolType tool_type, uint8_t gripper_type)
{
    _param = param;

    if (_type != SurgicalToolType(tool_type))
    {
        _type = SurgicalToolType(tool_type);

        if (_config_spcs_builder)
            _config_spcs_builder.reset();
        _config_spcs_builder = ConfigSpcsFactory::createConfigSpcsBuilder(_type);
    }

    _gripper_type = gripper_type;
}


void SurgicalTool::updateConfig(const SurgicalToolConfig &config)
{
    _config = config;
    if(_type == SURGICAL_TOOL_TYPE_ENDOSCOPIC){
        _config.set(_config.L_insert - 225, CONFIG_L_INSERT);
    }

    forwardKinematics();
}


void SurgicalTool::updateTarget(const mmath::Pose &pose)
{
    SurgicalToolConfig config;
    /*bool flag = */calcInverseKinematicsC3(pose, _param, config);
    _config = config;

    forwardKinematics();
}


void SurgicalTool::setGripperAngle(float angle)
{
    _gripper_angle = angle;
}


void SurgicalTool::setTao(float tao)
{
    _base_pose.R = mmath::rotByZ<float>(-tao);
}


void SurgicalTool::setBasePose(const mmath::Pose &pose)
{
    _base_pose = pose;
}


void SurgicalTool::reset()
{
    initialize(SurgicalToolParam(0, 0, 0, 0, 0, 0),
               SURGICAL_TOOL_TYPE_UNKNOWN, 0);
    updateConfig(SurgicalToolConfig(-500, 0, 0, 0, 0, 0));
    _end_pose = _base_pose;
    _config_spcs.clear();
    _task_spc.clear();
}


const SurgicalToolType& SurgicalTool::getType() const
{
    return _type;
}


const SurgicalToolConfig& SurgicalTool::getConfig() const
{
    return _config;
}


const SurgicalToolParam& SurgicalTool::getParam() const
{
    return _param;
}


const mmath::Pose& SurgicalTool::getBasePose() const
{
    return _base_pose;
}


const mmath::Pose& SurgicalTool::getEndPose() const
{
    return _end_pose;
}


const ConfigSpcs& SurgicalTool::getConfigSpcs() const
{
    return _config_spcs;
}


const TaskSpc& SurgicalTool::getTaskSpc() const
{
    return _task_spc;
}


const std::vector<Eigen::Vector3f>& SurgicalTool::getWSClouds() const
{
    return _ws_clouds;
}


void SurgicalTool::forwardKinematics()
{
    _config_spcs.clear();
    _config_spcs_builder->buildConfigSpcs(_config, _param, _config_spcs);    

    _task_spc.clear();
    calcForwardKinematics(_config_spcs, _task_spc);

    _end_pose = _base_pose;
    _end_pose *= _task_spc.end2base;
}


void SurgicalTool::inverseKinematics()
{

}


void SurgicalTool::updateWorkingSpace()
{
    auto addPoint = [this](SurgicalToolConfig& config) {
        ConfigSpcs config_spcs;
        TaskSpc    task_spc;
        _config_spcs_builder->buildConfigSpcs(config, this->_param, config_spcs);
        calcForwardKinematics(config_spcs, task_spc);
        this->_ws_clouds.push_back(task_spc.end2base.t);
    };

    _ws_clouds.clear();
    switch(_type){
    case SURGICAL_TOOL_TYPE_ENDOSCOPIC:
    {
        std::vector<double> L_range, phi_range, theta1_range, theta2_range;
        mmath::linspace(0, 5.0, 150.0, L_range);
        mmath::linspace(0, mmath::deg2rad(4), 2*mmath::PI, phi_range);
        mmath::linspace(0, mmath::deg2rad(3), mmath::PI/2, theta1_range);
        mmath::linspace(0, mmath::deg2rad(6), mmath::PI*2/3, theta2_range);

        float L_insert = _param.getL1() + _param.getL2() + _param.getLr();
        for(auto& phi:phi_range){
            for(auto& theta2:theta2_range){
                SurgicalToolConfig config(L_insert, phi, mmath::PI/2, 0, theta2, 0);
                addPoint(config);
            }
            for(auto& theta1:theta1_range){
                SurgicalToolConfig config(L_insert + 150.0, phi, theta1, 0, 0, 0);
                addPoint(config);
            }
            for(auto& L:L_range){
                SurgicalToolConfig config(L_insert + L, phi, mmath::PI/2, 0, 0, 0);
                addPoint(config);
            }
        }
		break;
    }
    case SURGICAL_TOOL_TYPE_SP_TOOL:
    {

		break;
    }
	default:
		break;
	}

}
