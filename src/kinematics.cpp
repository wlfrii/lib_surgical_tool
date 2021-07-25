#include "kinematics.h"
#include "./define/nbus_msg_common_def.h"
#include "./instrument.h"

constexpr float DEFAULT_GRIPPER_LEN = 15;

Kinematics::Kinematics(Instrument *ins)
	: instrument(ins)
	, prevQ(nullptr)
	, prevRt(nullptr)
{
}

void Kinematics::addFirstStem(const RT& origin, ConfigSpc& q)
{
	self = origin;
	execute(q);
}

void Kinematics::addMoreStem(ConfigSpc& q)
{
	accRT();
	execute(q);
}

RT Kinematics::addGripper()
{
	accRT();
	std::string body0 = "";
	std::string body1 = "";
	switch (instrument->gripper_type)
	{
	case NBUS_NS::CHI_ZHEN_QI:						// 1
	case NBUS_NS::SHUANG_DONG_WAN_JIAN:				// 4, not use
	case NBUS_NS::DAN_JI_ZHI_JIAN:					// 5
	case NBUS_NS::DAN_JI_WAN_JIAN:					// 13
	case NBUS_NS::WU_CHUANG_ZHUA_QIAN:				// 2
	case NBUS_NS::FEN_LI_QIAN:						// 3, not use
	case NBUS_NS::DAN_JI_DIAN_GOU:					// 12
	case NBUS_NS::SHUANG_JI_FEN_LI_QIAN:			// 14, SHUANT_JI_WAN_FEN_LI_QIAN
	case NBUS_NS::SHUANG_JI_WU_CHUANG_ZHUA_QIAN:	// 15, SHUANG_JI_ZHUA_QIAN
	case NBUS_NS::SHAUNG_JI_MA_LI_LAN_QIAN:			// 16, SHUANG_JI_WAN_TOU_ZHUA_QIAN
		return addGripper(body0, body1);
	case NBUS_NS::FU_QIANG_JING:					// 17
		return addCamera();
	case NBUS_NS::EFFECTOR_TYPE_UNKNOWN:
		break;;
	}	
	return self;
}

void Kinematics::accRT()
{
	RT temp;
	if (prevQ->is_bend)
	{
		getRT(*prevQ, temp);
	}
	else
	{
		getRotatedRT(*prevQ, temp);
	}
	self.p = prevRt->p + prevRt->R*temp.p;
	self.R = prevRt->R * temp.R;

	//printf("Self, %s\n", self.to_c_str());
}

void Kinematics::execute(ConfigSpc& q)
{
	float delta;
	if (q.is_bend)
		delta = -q.delta;
	else
		delta = q.delta;

	prevQ = &q;
	prevRt = &self;
}

void Kinematics::getRT(ConfigSpc &q, RT& rt)
{
    if (q.theta < 0) {
		q.theta = -q.theta;
		q.delta = q.delta + math::deg2rad<float>(180.0f);
	}  
    math::calcSingleSegmentRT(q.length, q.theta, q.delta, rt);
}

void Kinematics::getRotatedRT(ConfigSpc &q, RT& rt)
{
	rt.R = math::rotByZ<float>(q.delta);
	rt.p[0] = 0; rt.p[1] = 0; rt.p[2] = q.length;
}

RT Kinematics::addCamera()
{
	// the gamma3 and the length of vision arm should be considered
	RT temp;
	temp.R = math::rotByZ<float>(instrument->param.getGamma3());
	temp.p = Eigen::Vector3f(0, 0, 18.56);
	self.R = self.R * temp.R;
	self.p = self.p + self.R*temp.p;
	return self;
}

RT Kinematics::addGripper(std::string &body0, std::string &body1)
{
	// the gamma3 and the length of tool arm should be considered
	self.p = self.p + self.R*Eigen::Vector3f(0, 0, DEFAULT_GRIPPER_LEN);
	return self;
}
