/******************************************************************************
                        ros_fri

  Copyright (C) 2014 Johannes Liebrecht --- 8liebrec@informatik.uni-hamburg.de

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program; if not, write to the Free Software
    Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.

    You can also obtain a copy of the GNU General Public License
    at <http://www.gnu.org/licenses>.

******************************************************************************/

#ifndef __LWR_ACTION_SERVER__
#define __LWR_ACTION_SERVER__

#include <FastResearchInterface.h>
#include <ReflexxesAPI.h>
#include <RMLPositionFlags.h>
#include <RMLPositionInputParameters.h>
#include <RMLPositionOutputParameters.h>

#include <actionlib/server/action_server.h>
#include <actionlib/server/server_goal_handle.h>
#include <moveit/robot_state/robot_state.h>

#include <control_msgs/FollowJointTrajectoryAction.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/UInt32.h>
#include <std_msgs/Bool.h>
#include <ros_fri/RMLVelocityInputParameters.h>
#include <ros_fri/RMLPositionInputParameters.h>

#define PI 3.1415926535897932384626433832795
#define NUMBER_OF_FRAME_ELEMENTS 12
#define NUMBER_OF_CART_DOFS 6
#define EOK 0
#define EALREADY 114

enum statemaschine
{
    ready = 0,
    trajectory = 1,
    jntPosGoal = 2,
    jntVelGoal = 3,
};

typedef typename actionlib::ActionServer<control_msgs::FollowJointTrajectoryAction>::GoalHandle GoalHandle;
typedef boost::shared_ptr<const control_msgs::FollowJointTrajectoryGoal> Goal;

class lwr_action_server
{
protected:
    ros::NodeHandle nh_;
    // NodeHandle instance must be created before this line. Otherwise strange error may occur.

    // FastResearchInterface
    FastResearchInterface* FRI_;
    unsigned int currentControlScheme_;

    // ReflexxesAPI
    double CycleTime_;
    ReflexxesAPI* RML_;
    RMLVelocityInputParameters* VIP_;
    RMLVelocityOutputParameters* VOP_;
    RMLVelocityFlags VFlags_;
    RMLPositionInputParameters* PIP_;
    RMLPositionOutputParameters* POP_;
    RMLPositionFlags PFlags_;

    // MoveIt
    moveit::core::RobotState* kinematic_state_;

    // lwr_action_server
    actionlib::ActionServer<control_msgs::FollowJointTrajectoryAction> as_;
    std::string action_name_;

    GoalHandle agh_;
    bool goal_active_, cancel_goal_;
    int seqCounterFJTF_, seqCounterJointStates_, seqCounterFK_;
    statemaschine state_;
    ros_fri::RMLPositionInputParameters msgPOP_;
    ros_fri::RMLVelocityInputParameters msgVOP_;

    double Position_old_[LBR_MNJ], Position_new_[LBR_MNJ], velocity_new_[LBR_MNJ], velocity_old_[LBR_MNJ],
        acceleration_[LBR_MNJ];
    ros::Time time_old_, time_new_, time_delta_;

    float commandedJointPositions_[LBR_MNJ], measuredJointPositions_[LBR_MNJ], commandedJointPositionOffsets_[LBR_MNJ],
        measuredJointTorques_[LBR_MNJ], estimatedExternalJointTorques_[LBR_MNJ], commandedTorquesInNm_[LBR_MNJ],
        commandedStiffness_[LBR_MNJ], commandedDamping_[LBR_MNJ], driveTemperatues_[LBR_MNJ];

    float commandedCartForcesAndTorques_[NUMBER_OF_CART_DOFS], commandedCartStiffness_[NUMBER_OF_CART_DOFS],
        commandedCartDamping_[NUMBER_OF_CART_DOFS], estimatedExternalCartForcesAndTorques_[NUMBER_OF_CART_DOFS],
        commandedCartPose_[NUMBER_OF_FRAME_ELEMENTS], measuredCartPose_[NUMBER_OF_FRAME_ELEMENTS],
        commandedCartPoseOffsets_[NUMBER_OF_FRAME_ELEMENTS];

    ros::Publisher pub_estimatedExternalCartForcesAndTorques_, pub_driveTemperatues_,
        pub_estimatedExternalJointTorques_, pub_measuredCartPose_, pub_measuredJointPositions_,
        pub_commandedJointPositions_, pub_commandedJointPositionOffsets_, pub_measuredJointTorques_,
        pub_commandedCartPose_, pub_commandedCartPoseOffsets_, pub_joints_, pub_motionCompletion_;

    ros::Subscriber sub_setCurrentControlScheme_, sub_setJointDamping_, sub_setJointStiffness_, sub_setJointTorques_,
        sub_setCartDamping_, sub_setCartStiffness_, sub_setCartForcesAndTorques_, sub_cancelLWRTrajectory_,
        sub_setJointPosition_, sub_setCartPose_, sub_jntPosGoal_, sub_jntVelGoal_;

    ros::ServiceClient ser_fk_;

    // Publish feedback of trajectory execution.
    // param: current Goal; current trajectory point number; current duration from trajectory start
    void publishFeedback(Goal, unsigned int, ros::Duration);

    // Publish current joint states.
    void publishJointStates();

    // Publish current LWR states.
    void publishLWRStates();

    // Stops with a "soft" motion the LWR execution
    void stopLwr();

    // State JntPosGoal of statemachine: computes and executes joint position goal
    void stateJntPosGoal();

    // State JntVelGoal: computes and executes joint velocity goal
    void stateJntVelGoal();

    // State JointPositionTrajectory of statemachine: computes and executes joint position trajectory
    void executeJointPositionTrajectory();

    // State JointImpedanceTrajectory of statemachine: computes and executes joint impedance trajectory
    void executeJointImpedanceTrajectory();

    // State CartImpedanceTrajectory of statemachine: computes and executes cartesian impedance trajectory
    void executeCartImpedanceTrajectory();

public:
    // Constructor
    // param:	name of node
    lwr_action_server(std::string name);

    // Destructor
    ~lwr_action_server();

    // Set FRI control strategie. Possible are
    // FastResearchInterface::JOINT_POSITION_CONTROL (number 10)
    // FastResearchInterface::JOINT_IMPEDANCE_CONTROL (number 30)
    // FastResearchInterface::CART_IMPEDANCE_CONTROL (number 20)
    void setCurrentControlScheme(const std_msgs::UInt32::ConstPtr&);

    // Set the desired joint position vector into the data telegram to be send to the KRC unit.
    void setJointPosition(const std_msgs::Float32MultiArray::ConstPtr&);

    // Set the desired joint damping vector into the data telegram to be send to the KRC unit.
    void setJointDamping(const std_msgs::Float32MultiArray::ConstPtr&);

    // Set the desired joint stiffness vector into the data telegram to be send to the KRC unit.
    void setJointStiffness(const std_msgs::Float32MultiArray::ConstPtr&);

    // Set the desired joint torque vector into the data telegram to be send to the KRC unit.
    void setJointTorques(const std_msgs::Float32MultiArray::ConstPtr&);

    // Set the desired Cartesian pose frame into the data telegram to be send to the KRC unit.
    void setCartPose(const std_msgs::Float32MultiArray::ConstPtr&);

    // Set the desired Cartesian damping vector into the data telegram to be send to the KRC unit.
    void setCartDamping(const std_msgs::Float32MultiArray::ConstPtr&);

    // Set the desired Cartesian stiffness vector into the data telegram to be send to the KRC unit.
    void setCartStiffness(const std_msgs::Float32MultiArray::ConstPtr&);

    // Set the desired Cartesian force/torque vector into the data telegram to be send to the KRC unit.
    void setCartForcesAndTorques(const std_msgs::Float32MultiArray::ConstPtr&);

    // Cancel trajectoryexecution with a soft trajectory stop
    void setCancelLWRTrajectory(const std_msgs::Bool::ConstPtr&);

    // Set and execute the desired joint position goal with the ReflexxesType II Library.
    void setJntPosGoal(const ros_fri::RMLPositionInputParameters::ConstPtr&);

    // Set and execute the desired joint velocity goal with the ReflexxesType II Library.
    void setJntVelGoal(const ros_fri::RMLVelocityInputParameters::ConstPtr&);

    // Get the current running Fast Research Interface control strategie.
    unsigned int getCurrentControlScheme();

    // Gets current state of lwr_action_server statemachine.
    statemaschine getstate();

    // Starts the lwr_action_server node.
    int run();

private:
    // Goal callback of action server.
    // If no goal active, accept new goal else reject it.
    void goalCB(GoalHandle);

    // Cancel current goal, if cancel request is accepted by the action server.
    void cancelCB(GoalHandle);

};  // class lwr_action_server
#endif
