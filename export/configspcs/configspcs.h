#ifndef LIB_INSTRUMENT_CONFIGSPCS_H_LF
#define LIB_INSTRUMENT_CONFIGSPCS_H_LF
#include <lib_math/lib_math.h>
#include "../define/marray.h"

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
    void clear()
    {
        for(int i = 0; i < _count; i++){
            _elements[i].clear();
        }
        _count = 0;
    }
};


/**
 * @brief A class stores the RT of each segment based on task space of
 * continuum robot.
 */
class TaskSpc
    : public ArrayRepo<mmath::RT, MAX_SECTION_COUNT>
{
public:
	void clear()
    {
        for(int i = 0; i < _count; i++){
            _elements[i] = mmath::RT();
        }
        _count = 0;
    }
};


/**
 * @brief Calculating the forward kinematics of two-segment continuum robot.
 * 
 * @param qs            The configurations of the continuum robot
 * @param task_space    The rotation and position of each segment in the task
 *                      space.
 */
inline void calcForwardKinematics(const ConfigSpcs& qs, TaskSpc& task_space)
{
	for(int i = 0; i < qs.count(); i++){
		auto& q = qs[i];
		mmath::continuum::calcSingleSegmentRT(q, task_space[i]);
	}
}
/**
 * @brief Override based on 'void calcForwardKinematics()'.
 */
inline TaskSpc calcForwardKinematics(const ConfigSpcs& qs)
{
	TaskSpc task_space;
	calcForwardKinematics(qs, task_space);
	return task_space;
}


#endif // LIB_INSTRUMENT_CONFIGSPCS_H_LF