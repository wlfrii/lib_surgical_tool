#include "../include/lib_surgical_tool/configspcs.h"
#include <iomanip>


ConfigSpcs::ConfigSpcs()
    : space_type(ConfigSpaceType::C0)
{
}



const mmath::continuum::ConfigSpc& ConfigSpcs::operator()(
        const ConfigSpcs::SegmentType &type)
{
    if(type == STEM_BASE) return _elements[0];

    uint8_t idx = type - MAX_STEM_COUNT + _count;
    if(idx > 0) {
        return _elements[idx];
    }
    else {
        static mmath::continuum::ConfigSpc null;
        return null;
    }
}


std::ostream& operator<<(std::ostream &os, const ConfigSpcs& qs)
{
    int w = 12;
    os << "Configuration space: C" << int(qs.space_type) << std::endl;
    for(uint8_t i = 0; i < qs._count; i++) {
        char bd[16];
        sprintf(bd, "IsBending:[%d]", qs[i].is_bend);

        os << std::setw(w) << qs[i].theta << " "
           << std::setw(w) << qs[i].delta << " "
           << std::setw(w) << qs[i].length << " "
           << std::setw(w + 4) << bd;
        if(i < qs._count - 1) os << '\n';
    }
    return os;
}


void ConfigSpcs::clear()
{
    for(int i = 0; i < _count; i++){
        _elements[i].clear();
    }
    _count = 0;
}



TaskSpc::TaskSpc()
    : space_type(ConfigSpaceType::C0)
{

}


const mmath::Pose& TaskSpc::operator()(const TaskSpc::SegmentType &type)
{
    auto getPose = [this](const TaskSpc::SegmentType &stype) -> mmath::Pose& {
        if(stype == POSE_1B_TO_BASE) return _elements[0];

        uint8_t idx = stype - MAX_STEM_COUNT + _count;
        if(idx > 0) {
            return _elements[idx];
        }
        else {
            static mmath::Pose null;
            return null;
        }
    };

    // Return the pose of each stem
    switch (type) {
    case POSE_1B_TO_BASE:
    case POSE_1E_TO_1B:
    case POSE_2B_TO_1E:
    case POSE_2E_TO_2B:
    case POSE_G_TO_2E:
        return getPose(type);
    case POSE_G_TO_BASE:
        return end2base;
    case POSE_G_TO_1B:
    {
        static mmath::Pose T_g_to_1b;
        T_g_to_1b = getPose(POSE_1B_TO_BASE).inverse() * end2base;
        return T_g_to_1b;
    }
    case POSE_G_TO_2B:
    {
        static mmath::Pose T_g_to_2b;;
        T_g_to_2b = getPose(POSE_2E_TO_2B) * getPose(POSE_G_TO_2E);
        return T_g_to_2b;
    }
    case POSE_2B_TO_BASE:
    {
        static mmath::Pose T_2b_to_b;;
        T_2b_to_b = getPose(POSE_1B_TO_BASE) * getPose(POSE_1E_TO_1B) *
                getPose(POSE_2B_TO_1E);
        return T_2b_to_b;
    }
    case POSE_2B_TO_1B:
    {
        static mmath::Pose T_2b_to_1b;;
        T_2b_to_1b = getPose(POSE_1E_TO_1B) * getPose(POSE_2B_TO_1E);
        return T_2b_to_1b;
    }
    case POSE_2E_TO_1B:
    {
        static mmath::Pose T_2e_to_1b;;
        T_2e_to_1b = getPose(POSE_1E_TO_1B) * getPose(POSE_2B_TO_1E) *
                getPose(POSE_2E_TO_2B);
        return T_2e_to_1b;
    }
    }
}


std::ostream& operator<<(std::ostream &os, const TaskSpc& Ts)
{
    os << "Configuration space: C" << int(Ts.space_type) << std::endl;
    for(int i = 0; i < Ts._count; i++) {
        os << "T" << i << ":\n" << Ts[i] << "\n";
    }
    os << "TaskEnd2Base:\n" << Ts.end2base;

    return os;
}


void TaskSpc::clear()
{
    for(int i = 0; i < _count; i++){
        _elements[i] = mmath::Pose();
    }
    end2base = mmath::Pose();
    _count = 0;
}
