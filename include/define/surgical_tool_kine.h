#ifndef LIB_SURGICAL_TOOL_KINE_H_LF
#define LIB_SURGICAL_TOOL_KINE_H_LF
#include "configspcs.h"
#include "surgical_tool_param.h"
#include "surgical_tool_config.h"


using Jacobian = Eigen::Matrix<float, 6, 6>;


/**
 * @brief Calculate the forward kinematics of two-segment continuum robot.
 *
 * @param qs            The configurations of the continuum robot
 * @param task_space    The rotation and position of each segment in the task
 *                      space.
 */
void calcForwardKinematics(const ConfigSpcs& qs, TaskSpc& task_space);


/**
 * @brief Override based on 'void calcForwardKinematics()'.
 */
TaskSpc calcForwardKinematics(const ConfigSpcs& qs);


/**
 * @brief IKC3  Cacluate the inverse kinematics of two-segment continuum robot
 * that with C3 configuration.
 *
 * @param pose    The target pose of gripper w.r.t the robot base frame.
 * @param param   The structure parameters of the continuum robot.
 * @param config  The configure parameters of the continuum robot
 * @return true - The given pose locates in workspace
 *         false - The given pose is out of workspace
 */
bool calcInverseKinematicsC3(const mmath::Pose &pose,
                             const SurgicalToolParam& param,
                             SurgicalToolConfig &config);


/**
 * @brief Calculate the Jacobian of two-segment continuum robot, w.r.t Velocity
 *        and Angular-Velocity direction.
 *
 * @param qs        The configurations of the continuum robot
 * @param Jacobian  The returned Jacobian
 */
void calcJacobian(const ConfigSpcs& qs, Jacobian& jacobian);


/**
 * @brief Override based on 'void calcJacobian()'.
 */
Jacobian calcJacobian(const ConfigSpcs& qs);



#endif // LIB_SURGICAL_TOOL_KINE_H_LF
