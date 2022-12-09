#include "../include/surgical_tool.h"
#include "../configspcs/configspcs_builder.h"
#include "../configspcs/configspcs_factory.h"
#include <utility>


SurgicalTool::SurgicalTool(const mmath::Pose &base_pose)
    : _type(SURGICAL_TOOL_TYPE_UNKNOWN)
    , _gripper_type(0)
    , _gripper_angle(0)
    , _base_pose(base_pose)
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
        _config.safeSet(CONFIG_L_INSERT, _config.L_insert - 225);
    }

    forwardKinematics();
}


bool SurgicalTool::updateTarget(const mmath::Pose& pose, bool do_resolved_rates)
{
    bool flag = false;
    if(!do_resolved_rates){
        SurgicalToolConfig config;
        flag = calcInverseKinematicsC3(pose, _param, config);
        if(_type == SURGICAL_TOOL_TYPE_SP_TOOL){
            config.L_insert += _param.getL2() + _param.getLr();
        }
        _config = config;

        forwardKinematics();
    }
    else{
        flag = resolvedRates(pose);
    }

    return flag;
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


const mmath::Pose& SurgicalTool::getEnd2BasePose() const
{
    return _task_spc.end2base;
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

namespace {
void limitVelocity(float epsilon_p, float epsilon_r, float& v_limit, float& w_limit)
{
    float v_limit_max = 100;            // mm/s
    float w_limit_max = M_PI / 2.f;     // rad/s

    float dis1 = 2, dis2 = 0.5, dis3 = 1000, dis4 = 1000;
    float rd1 = 1, rd2 = 0.5, rd3 = 0.25, rd4 = 0.1, rd = 0.2;

    float agl1 = mmath::deg2radf(1);
    float agl2 = mmath::deg2radf(0.5);
    float agl3 = mmath::deg2radf(0.1);
    float agl4 = mmath::deg2radf(0.05);
    float rr1 = 1, rr2 = 0.7, rr3 = 0.5, rr4 = 0.3, rr = 0.1;

    if (epsilon_p > dis1)
        v_limit = v_limit_max * rd1;
    else if (epsilon_p > dis2)
        v_limit = v_limit_max * rd2;
    else if (epsilon_p > dis3)
        v_limit = v_limit_max * rd3;
    else if (epsilon_p > dis4)
        v_limit = v_limit_max * rd4;
    else
        v_limit = v_limit_max * rd;

    if (epsilon_r > agl1)
        w_limit = w_limit_max * rr1;
    else if (epsilon_r > agl2)
        w_limit = w_limit_max * rr2;
    else if (epsilon_r > agl3)
        w_limit = w_limit_max * rr3;
    else if (epsilon_r > agl4)
        w_limit = w_limit_max * rr4;
    else
        w_limit = w_limit_max * rr;
}
}

bool SurgicalTool::resolvedRates(const mmath::Pose& pose)
{
    bool flag = false;

    // Initial setting
    float v_limit = 1;
    float w_limit = 1;

    float dt = 0.001;                   // d time
    float epsilon_p_max = 0.01;         // mm
    float epsilon_r_max = 0.02;         // rad
    float epsilon_p, epsilon_r;

    uint16_t max_iterations = 1000;

    const mmath::Pose& tgt_pose = pose;

    uint16_t n = 0;
    while (n++ < max_iterations) {
        const mmath::Pose& curr_pose = getEnd2BasePose();

        Eigen::Vector3f vec = tgt_pose.t - curr_pose.t;
        epsilon_p = vec.norm();

        Eigen::AngleAxisf angle_axis(tgt_pose.R * curr_pose.R.transpose());
        epsilon_r = angle_axis.angle();

        if(epsilon_p <= epsilon_p_max && epsilon_r <= epsilon_r_max){
            flag = true;
            break;
        }

        limitVelocity(epsilon_p, epsilon_r, v_limit, w_limit);
        if (abs(epsilon_p) < 1e-7) epsilon_p = 1e-4;

        Eigen::Vector3f dv = v_limit * vec / epsilon_p;
        Eigen::Vector3f dw = w_limit * angle_axis.axis();
        Eigen::Vector<float, 6> dx;
        dx << dv[0], dv[1], dv[2], dw[0], dw[1], dw[2];

        Jacobian J, inv_J;
        calcJacobian(_config_spcs, J);
        inverseJacobian(J, inv_J);

        Eigen::Vector<float, 6> dq = inv_J * dx;
        dq *= dt;

        // There we just consider 'C3' for now.
//        _config.L_insert += dq[3];
//        _config.phi += dq[0];
//        _config.theta1 += dq[1];
//        _config.delta1 += dq[2];
//        _config.theta2 += dq[4];
//        _config.delta2 += dq[5];
        _config.safeAdd(CONFIG_L_INSERT, dq[3]);
//        _config.safeAdd(CONFIG_PHI, dq[0]);
        _config.safeAdd(CONFIG_THETA1, dq[1]);
        _config.safeAdd(CONFIG_DELTA1, dq[2]);
        _config.safeAdd(CONFIG_THETA2, dq[4]);
        _config.safeAdd(CONFIG_DELTA2, dq[5]);

        forwardKinematics();
    }

//    if(!flag){
//        printf("Resolved rates flag: %d, epsilon_p:[%.3f], epsilon_r:[%.3f] \n",
//               flag, epsilon_p, epsilon_r);
//    }

    return flag;
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
