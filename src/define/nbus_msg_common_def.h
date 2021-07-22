#ifndef NBUS_MSG_COMMON_DEF_H
#define NBUS_MSG_COMMON_DEF_H

#define NBUS_NS sr::sbus
#define NBUS_NS_BEGIN namespace sr { namespace sbus{
#define NBUS_NS_END   }}
#define USING_NBUS_NS using namespace NBUS_NS;

NBUS_NS_BEGIN

typedef unsigned char EffectorType;// effector_type
const EffectorType EFFECTOR_TYPE_UNKNOWN = 0;
const EffectorType CHI_ZHEN_QI = 1;
const EffectorType WU_CHUANG_JIA_QIAN = 2;  //not use
const EffectorType WU_CHUANG_ZHUA_QIAN = 2;  // WU_CHUANG_JIA_QIAN --->WU_CHUANG_ZHUA_QIAN
const EffectorType FEN_LI_QIAN = 3;     //not use
const EffectorType SHUANG_DONG_WAN_JIAN = 4;    //not use
const EffectorType SHUANG_DONG_ZHI_JIAN = 5;  //not use
const EffectorType DAN_JI_ZHI_JIAN = 5;   //SHUANG_DONG_ZHI_JIAN-->DAN_JI_ZHI_JIAN
const EffectorType NEI_ZANG_ZHUA_QU_QIAN = 6;  //not use
const EffectorType SHU_YA_JIA_QIAN = 7;   //not use
const EffectorType CHANG_HE_ZHUA_QU_QIAN = 8;  //not use
const EffectorType CHANG_ZHUA_QIAN = 9;  //not use
const EffectorType DAN_DONG_WAN_JIAN = 10;   //not use
const EffectorType TAI_JIA_QIAN = 11;    //not use
const EffectorType DAN_JI_DIAN_GOU = 12;
const EffectorType DAN_JI_WAN_JIAN = 13;
const EffectorType SHUANG_JI_WAN_FEN_LI = 14; //not use
const EffectorType SHUANG_JI_FEN_LI_QIAN = 14;   //SHUANG_JI_WAN_FEN_LI -->SHUANG_JI_FEN_LI_QIAN
const EffectorType SHUANG_JI_WU_CHUANG_ZHUA_QIAN = 15;
const EffectorType SHAUNG_JI_MA_LI_LAN_QIAN = 16;
const EffectorType FU_QIANG_JING = 17;
const EffectorType PANG_GUANG_JING = 18;
const EffectorType MI_NIAO_ZHUA_QIAN = 19;
const EffectorType MI_NIAO_DIAN_GOU = 20;

NBUS_NS_END

#endif // NBUS_MSG_COMMON_DEF_H