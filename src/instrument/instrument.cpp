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
	: type(INSTRUMENT_TYPE_UNKNOWN)
	, gripper_type(0)
	, gripper_angle(0)
	, base_pose(::real_pose.get())
	, end_pose(base_pose)
{
	config_spcs_builder = ConfigSpcsFactory::createConfigSpcsBuilder(this);
}

Instrument::~Instrument()
{

}

void Instrument::initialize(const InstrumentParam &new_param, const InstrumentConfig &new_config)
{
	param = new_param;
	config = new_config;
}

void Instrument::updateConfig(const InstrumentConfig &new_config)
{
	config = new_config;
}

void Instrument::setInstrumentType(uint8_t new_type)
{
	if (type != InstrumentType(new_type))
	{
		type = InstrumentType(new_type);

		if (config_spcs_builder) config_spcs_builder.reset();
		config_spcs_builder = ConfigSpcsFactory::createConfigSpcsBuilder(this);
	}
}

void Instrument::setGripperType(uint8_t type)
{
	gripper_type = type;
}

void Instrument::setGripperAngle(float type)
{
	gripper_angle = type;
}

void Instrument::setBaseRot(float angle)
{
	base_pose.R = mmath::rotByZ<float>(-angle);
}

void Instrument::setBasePose(const mmath::RT &rt)
{
	base_pose = rt;
}

void Instrument::reset()
{
	this->setInstrumentType(INSTRUMENT_TYPE_UNKNOWN);
	this->setGripperType(0);
	this->initialize(InstrumentParam(0, 0, 0, 0, 0, 0), InstrumentConfig(-500, 0, 0, 0, 0, 0));
	this->end_pose = this->base_pose;
	this->config_spcs.clear();
	this->task_spc.clear();
}


uint8_t Instrument::getInstrumentType() const
{
	return type;
}

const InstrumentConfig& Instrument::getConfig() const
{
	return config;
}

const InstrumentParam& Instrument::getParam() const
{
	return param;
}

const mmath::RT& Instrument::getBasePose() const
{
	return base_pose;
}

const mmath::RT& Instrument::getEndPose() const
{
	return end_pose;
}

const ConfigSpcs& Instrument::getConfigSpcs() const
{
	return config_spcs;
}

const TaskSpc& Instrument::getTaskSpc() const
{
	return task_spc;
}

void Instrument::updateKinematics()
{
	config_spcs_builder->buildConfigSpcs(config_spcs);
	calcForwardKinematics(config_spcs, task_spc);

	end_pose = base_pose;
	for(int i = 0; i < task_spc.count(); i++){
		end_pose *= task_spc[i];
	}
}