#ifndef LIB_SURGICAL_TOOL_CONFIGSPCS_H_LF
#define LIB_SURGICAL_TOOL_CONFIGSPCS_H_LF
#include <lib_math/lib_math.h>
#include "array_repo.h"


/**
 * @brief The max segment of a continuum robot.
 */
const uint8_t MAX_SECTION_COUNT = 5;

/**
 * @brief Configurations of twe-segment continuum robot.
 */
class ConfigSpcs 
    : public ArrayRepo<mmath::continuum::ConfigSpc, MAX_SECTION_COUNT>
{
public:
    enum Space {
        C1 = 1, C2 = 2, C3 = 3, C4 = 4
    };

    void clear()
    {
        for(int i = 0; i < _count; i++){
            _elements[i].clear();
        }
        _count = 0;
    }

    Space space;
};


/**
 * @brief A class stores the RT of each segment based on task space of
 * continuum robot.
 */
class TaskSpc
    : public ArrayRepo<mmath::Pose, MAX_SECTION_COUNT>
{
public:
	void clear()
    {
        for(int i = 0; i < _count; i++){
            _elements[i] = mmath::Pose();
        }
        end2base = mmath::Pose();
        _count = 0;
    }

    mmath::Pose end2base;
};


#endif // LIB_SURGICAL_TOOL_CONFIGSPCS_H_LF
