#include "../../export/instrument/instrument.h"
#include "../configspcs/configspcs_factory.h"
#include "../../export/define/marray.h"

namespace
{
	/* Single port
	 *        3						  3
	 *		1	4 inverse the frame 1	4
	 *		  2						  2
	 */
	Eigen::Matrix3f I3 = Eigen::Matrix3f::Identity(3, 3);
	// The pose with respect to the real single port trocar
	mArray<mmath::RT, 4> real_pose = {
		mmath::RT(I3, { 0.0f, -7.85f, 0.f }),
		mmath::RT(I3, { 7.66f, -1.7f, 0.f }),
		mmath::RT(I3, { 0.0f, 5.8f, 0.f - 15 }),
		mmath::RT(I3, { -7.66f, -1.7f,  0.f })
	};

}

Instrument::Instrument()
    : _type(INSTRUMENT_TYPE_UNKNOWN)
    , _gripper_type(0)
    , _gripper_angle(0)
    , _base_pose(::real_pose.get())
    , _end_pose(_base_pose)
{
    _config_spcs_builder = ConfigSpcsFactory::createConfigSpcsBuilder(_type);
}

Instrument::~Instrument()
{

}

void Instrument::initialize(const InstrumentParam &param, const InstrumentConfig &config)
{
    _param = param;
    _config = config;
}

void Instrument::updateConfig(const InstrumentConfig &config)
{
    _config = config;
}

void Instrument::setInstrumentType(uint8_t type)
{
    if (_type != InstrumentType(type))
	{
        _type = InstrumentType(type);

        if (_config_spcs_builder)
            _config_spcs_builder.reset();
        _config_spcs_builder = ConfigSpcsFactory::createConfigSpcsBuilder(_type);

        updateWorkingSpace();
	}
}

void Instrument::setGripperType(uint8_t type)
{
    _gripper_type = type;
}

void Instrument::setGripperAngle(float angle)
{
    _gripper_angle = angle;
}

void Instrument::setBaseRot(float angle)
{
    _base_pose.R = mmath::rotByZ<float>(-angle);
}

void Instrument::setBasePose(const mmath::RT &rt)
{
    _base_pose = rt;
}

void Instrument::reset()
{
    setInstrumentType(INSTRUMENT_TYPE_UNKNOWN);
    setGripperType(0);
    initialize(InstrumentParam(0, 0, 0, 0, 0, 0), InstrumentConfig(-500, 0, 0, 0, 0, 0));
    _end_pose = _base_pose;
    _config_spcs.clear();
    _task_spc.clear();
}


uint8_t Instrument::getInstrumentType() const
{
    return _type;
}

const InstrumentConfig& Instrument::getConfig() const
{
    return _config;
}

const InstrumentParam& Instrument::getParam() const
{
    return _param;
}

const mmath::RT& Instrument::getBasePose() const
{
    return _base_pose;
}

const mmath::RT& Instrument::getEndPose() const
{
    return _end_pose;
}

const ConfigSpcs& Instrument::getConfigSpcs() const
{
    return _config_spcs;
}

const TaskSpc& Instrument::getTaskSpc() const
{
    return _task_spc;
}

const std::vector<Eigen::Vector3f>& Instrument::getWSClouds() const
{
    return _ws_clouds;
}

void Instrument::updateKinematics()
{
    _config_spcs.clear();
    _config_spcs_builder->buildConfigSpcs(_config, _param, _config_spcs);
    _task_spc.clear();
    calcForwardKinematics(_config_spcs, _task_spc);

    _end_pose = _base_pose;
    _end_pose *= _task_spc.base2end;
}

void Instrument::updateWorkingSpace()
{
    auto addPoint = [this](InstrumentConfig& config) {
        ConfigSpcs config_spcs;
        TaskSpc    task_spc;
        _config_spcs_builder->buildConfigSpcs(config, this->_param, config_spcs);
        calcForwardKinematics(config_spcs, task_spc);
        this->_ws_clouds.push_back(task_spc.base2end.t);
    };

    _ws_clouds.clear();
    switch(_type){
    case INSTRUMENT_TYPE_LAPAROSCOPE:
    {
        std::vector<double> L_range, phi_range, theta1_range, theta2_range;
        mmath::linespace(0, 5.0, 150.0, L_range);
        mmath::linespace(0, mmath::deg2rad(4), 2*mmath::PI, phi_range);
        mmath::linespace(0, mmath::deg2rad(3), mmath::PI / 2, theta1_range);
        mmath::linespace(0, mmath::deg2rad(6), mmath::PI*2 / 3, theta2_range);

        float L_insert = _param.getL1() + _param.getL2() + _param.getLr();
        for(auto& phi:phi_range){
            for(auto& theta2:theta2_range){
                InstrumentConfig config(L_insert, phi, mmath::PI / 2, 0, theta2, 0);
                addPoint(config);
            }
            for(auto& theta1:theta1_range){
                InstrumentConfig config(L_insert + 150.0, phi, theta1, 0, 0, 0);
                addPoint(config);
            }
            for(auto& L:L_range){
                InstrumentConfig config(L_insert + L, phi, mmath::PI / 2, 0, 0, 0);
                addPoint(config);
            }
        }
		break;
    }
	case InstrumentType::INSTRUMENT_TYPE_SP_TOOL:
    {

		break;
    }
	default:
		break;
	}

}
