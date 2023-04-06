#ifndef LIB_SURGICAL_TOOL_CONFIGSPCS_H_LF
#define LIB_SURGICAL_TOOL_CONFIGSPCS_H_LF
#include <lib_math/lib_math.h>
#include "array_repo.h"


/**
 * @brief The max segment of a continuum robot.
 */
const uint8_t MAX_STEM_COUNT = 5;



/**
 * @brief Define the type of configuration space
 */
enum class ConfigSpaceType {
    C0 = 0,
    C1 = 1,
    C2 = 2,
    C3 = 3,
    C4 = 4
};



/**
 * @brief Configurations of twe-segment continuum robot.
 */
class ConfigSpcs 
    : public ArrayRepo<mmath::continuum::ConfigSpc, MAX_STEM_COUNT>
{
public:
    ConfigSpcs();

    /**
     * @brief The SegmentType enum
     */
    enum SegmentType {
        STEM_BASE    = 0,
        STEM_SEG1    = 1,
        STEM_RIGID   = 2,
        STEM_SEG2    = 3,
        STEM_GRIPPER = 4
    };


    /**
     * @brief Access the Configuration Space of each segment.
     *
     * @param type The segment type
     * @return
     */
    const mmath::continuum::ConfigSpc& operator() (const SegmentType& type);


    /**
     * @brief std::cout the information
     * 
     * @param os 
     * @param config 
     * @return std::ostream& 
     */
    friend std::ostream& operator<< (std::ostream &os, const ConfigSpcs& qs);


    /**
     * @brief clear the stored values
     */
    void clear();


    ConfigSpaceType space_type;  //!< The configuration space type
};



/**
 * @brief A class stores the Pose of each segment based on task space of
 * 2-segments continuum robot.
 */
class TaskSpc
    : public ArrayRepo<mmath::Pose, MAX_STEM_COUNT>
{
public:
    TaskSpc();


    /**
     * @brief The SegmentType enum
     */
    enum SegmentType {
        POSE_1B_TO_BASE = 0,  //!< Correspoing to ConfigSpcs:STEM_BASE
        POSE_1E_TO_1B   = 1,  //!< Correspoing to ConfigSpcs:STEM_SEG1
        POSE_2B_TO_1E   = 2,  //!< Correspoing to ConfigSpcs:STEM_RIGID
        POSE_2E_TO_2B   = 3,  //!< Correspoing to ConfigSpcs:STEM_SEG2
        POSE_G_TO_2E    = 4,  //!< Correspoing to ConfigSpcs:STEM_GRIPPER

        POSE_G_TO_BASE,
        POSE_G_TO_1B,
        POSE_G_TO_2B,
        POSE_2B_TO_BASE,
        POSE_2B_TO_1B,
        POSE_2E_TO_1B
    };


    /**
     * @brief Access the Configuration Space of each segment.
     *
     * @param type The segment type
     * @return
     */
    const mmath::Pose& operator() (const SegmentType& type);


    /**
     * @brief std::cout the information
     *
     * @param os
     * @param config
     * @return std::ostream&
     */
    friend std::ostream& operator<< (std::ostream &os, const TaskSpc& Ts);


    /**
     * @brief clear the stored values
     */
    void clear();

    ConfigSpaceType space_type;  //!< The configuration space type
    mmath::Pose end2base;
};


#endif // LIB_SURGICAL_TOOL_CONFIGSPCS_H_LF
