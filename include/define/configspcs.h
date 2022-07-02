#ifndef LIB_SURGICAL_TOOL_CONFIGSPCS_H_LF
#define LIB_SURGICAL_TOOL_CONFIGSPCS_H_LF
#include <lib_math/lib_math.h>
#include "array_repo.h"
#include <iostream>

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


/**
 * @brief Calculating the forward kinematics of two-segment continuum robot.
 *
 * @param qs            The configurations of the continuum robot
 * @param task_space    The rotation and position of each segment in the task
 *                      space.
 */
inline void calcForwardKinematics(const ConfigSpcs& qs, TaskSpc& task_space)
{
    mmath::Pose pose;
    Eigen::Matrix4f mat = Eigen::Matrix4f::Identity();
    for(int i = 0; i < qs.count(); i++){
        auto& q = qs[i];
        mmath::continuum::calcSingleSegmentPose(q, pose);
        task_space.add(pose);
        task_space.end2base *= pose;
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

#endif // LIB_SURGICAL_TOOL_CONFIGSPCS_H_LF
