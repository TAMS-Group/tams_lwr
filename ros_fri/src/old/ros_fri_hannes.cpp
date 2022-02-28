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

#include "ros_fri.h"
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Pose.h>
#include <eigen_conversions/eigen_msg.h>
#include <moveit_msgs/GetPositionFK.h>
#include <tf/transform_listener.h>

#ifndef RAD
#define RAD(A) ((A)*PI / 180.0)
#endif

#ifndef DEG
#define DEG(A) ((A)*180.0 / PI)
#endif

static pthread_mutex_t mutex = PTHREAD_MUTEX_INITIALIZER;

// Constructor
// param:	name of node
lwr_action_server::lwr_action_server(std::string name)
    : as_(nh_, name, false), action_name_("lwr_action_server/joint_trajectory_action")
{
    // register the goal and cancel callbacks
    as_.registerGoalCallback(boost::bind(&lwr_action_server::goalCB, this, _1));
    as_.registerCancelCallback(boost::bind(&lwr_action_server::cancelCB, this, _1));
    as_.start();

    // publisher
    pub_estimatedExternalCartForcesAndTorques_ =
        nh_.advertise<std_msgs::Float32MultiArray>("/lwr_estimatedExternalCartForcesAndTorques", 1);
    pub_driveTemperatues_ = nh_.advertise<std_msgs::Float32MultiArray>("/lwr_driveTemperatues", 1);
    pub_estimatedExternalJointTorques_ =
        nh_.advertise<std_msgs::Float32MultiArray>("/lwr_estimatedExternalJointTorquesInNm", 1);
    pub_measuredCartPose_ = nh_.advertise<std_msgs::Float32MultiArray>("/lwr_measuredCartPose", 1);
    pub_commandedCartPose_ = nh_.advertise<std_msgs::Float32MultiArray>("/lwr_commandedCartPose", 1);
    pub_commandedCartPoseOffsets_ = nh_.advertise<std_msgs::Float32MultiArray>("/lwr_commandedCartPoseOffsets", 1);
    pub_measuredJointPositions_ = nh_.advertise<std_msgs::Float32MultiArray>("/lwr_measuredJointPositions", 1);
    pub_commandedJointPositions_ = nh_.advertise<std_msgs::Float32MultiArray>("/lwr_commandedJointPositions", 1);
    pub_commandedJointPositionOffsets_ =
        nh_.advertise<std_msgs::Float32MultiArray>("/lwr_commandedJointPositionOffsets", 1);
    pub_measuredJointTorques_ = nh_.advertise<std_msgs::Float32MultiArray>("/lwr_measuredJointTorques", 1);
    pub_joints_ = nh_.advertise<sensor_msgs::JointState>("/joint_states", 1);
    pub_motionCompletion_ = nh_.advertise<std_msgs::UInt32>("/lwr_motion_completion", 1);

    // subscriber
    sub_setCurrentControlScheme_ =
        nh_.subscribe("/lwr_setCurrentControlScheme", 1, &lwr_action_server::setCurrentControlScheme, this);
    sub_setJointDamping_ = nh_.subscribe("/lwr_setJointDamping", 1, &lwr_action_server::setJointDamping, this);
    sub_setJointStiffness_ = nh_.subscribe("/lwr_setJointStiffness", 1, &lwr_action_server::setJointStiffness, this);
    sub_setJointTorques_ = nh_.subscribe("/lwr_setJointTorques", 1, &lwr_action_server::setJointStiffness, this);
    sub_setCartDamping_ = nh_.subscribe("/lwr_setCartDamping", 1, &lwr_action_server::setCartDamping, this);
    sub_setCartStiffness_ = nh_.subscribe("/lwr_setCartStiffness", 1, &lwr_action_server::setCartStiffness, this);
    sub_setCartForcesAndTorques_ =
        nh_.subscribe("/lwr_setCartForcesAndTorques", 1, &lwr_action_server::setCartForcesAndTorques, this);
    sub_cancelLWRTrajectory_ =
        nh_.subscribe("/lwr_setCancelLWRTrajectory", 1, &lwr_action_server::setCancelLWRTrajectory, this);
    sub_setJointPosition_ = nh_.subscribe("/lwr_setJointPosition", 1, &lwr_action_server::setJointPosition, this);
    sub_setCartPose_ = nh_.subscribe("/lwr_setCartPose", 1, &lwr_action_server::setCartPose, this);
    sub_jntPosGoal_ = nh_.subscribe("/lwr_jntPosGoal", 0, &lwr_action_server::setJntPosGoal, this);
    sub_jntVelGoal_ = nh_.subscribe("/lwr_jntVelGoal", 0, &lwr_action_server::setJntVelGoal, this);
    ser_fk_ = nh_.serviceClient<moveit_msgs::GetPositionFK>("/compute_fk");

    goal_active_ = false;
    cancel_goal_ = false;
    // seq_counterFJTF_ = 0;
    seqCounterJointStates_ = 0;
    seqCounterFK_ = 0;
    // setCancelOnForceAtTCP_ = false;
    currentControlScheme_ = FastResearchInterface::JOINT_POSITION_CONTROL;

    memset(commandedJointPositions_, 0x0, LBR_MNJ * sizeof(float));
    memset(commandedJointPositionOffsets_, 0x0, LBR_MNJ * sizeof(float));
    memset(measuredJointPositions_, 0x0, LBR_MNJ * sizeof(float));
    memset(measuredJointTorques_, 0x0, LBR_MNJ * sizeof(float));
    memset(estimatedExternalJointTorques_, 0x0, LBR_MNJ * sizeof(float));
    memset(driveTemperatues_, 0x0, LBR_MNJ * sizeof(float));

    memset(commandedCartForcesAndTorques_, 0x0, NUMBER_OF_CART_DOFS * sizeof(float));
    memset(commandedCartStiffness_, 0x0, NUMBER_OF_CART_DOFS * sizeof(float));
    memset(commandedCartDamping_, 0x0, NUMBER_OF_CART_DOFS * sizeof(float));
    memset(estimatedExternalCartForcesAndTorques_, 0x0, NUMBER_OF_CART_DOFS * sizeof(float));
    memset(commandedCartPose_, 0x0, NUMBER_OF_FRAME_ELEMENTS * sizeof(float));
    memset(measuredCartPose_, 0x0, NUMBER_OF_FRAME_ELEMENTS * sizeof(float));
    memset(commandedCartPoseOffsets_, 0x0, NUMBER_OF_FRAME_ELEMENTS * sizeof(float));

    // FRI
    // Modify HERE---------------------------------------------------------------------------------------
    FRI_ = new FastResearchInterface("/homeL/8liebrec/TAMS/FRILibrary/etc/980039-FRI-Driver.init");
    //---------------------------------------------------------------------------------------------------

    // set lwr_action_server to default FastResearchInterface::JOINT_POSITION_CONTROL
    int resultValue = 0;
    resultValue = FRI_->StartRobot(currentControlScheme_);
    if ((resultValue != EOK) && (resultValue != EALREADY))
    {
        ROS_INFO("lwr_action_server: An error occurred during starting up the robot...\n");
        exit(EXIT_FAILURE);
    }
    // initialize position and timers for velocity calculation
    FRI_->GetMeasuredJointPositions(measuredJointPositions_);
    for (unsigned int j = 0; j < LBR_MNJ; j++)
    {
        Position_old_[j] = measuredJointPositions_[j];
    }
    time_old_ = ros::Time::now();
    FRI_->WaitForKRCTick();
    FRI_->GetMeasuredJointPositions(measuredJointPositions_);
    for (unsigned int j = 0; j < LBR_MNJ; j++)
    {
        Position_new_[j] = measuredJointPositions_[j];
    }
    time_new_ = ros::Time::now();
    time_delta_.sec = time_new_.sec - time_old_.sec;
    time_delta_.nsec = time_new_.nsec - time_old_.nsec;

    // joint impedance mode
    for (unsigned int i = 0; i < LBR_MNJ; i++)
    {
        commandedStiffness_[i] = (float)200.0;
        commandedDamping_[i] = (float)0.7;
        commandedTorquesInNm_[i] = (float)0.0;
    }
    FRI_->SetCommandedJointStiffness(commandedStiffness_);
    FRI_->SetCommandedJointDamping(commandedDamping_);
    FRI_->SetCommandedJointTorques(commandedTorquesInNm_);

    // cart impedance mode
    for (unsigned int i = 0; i < NUMBER_OF_CART_DOFS; i++)
    {
        commandedCartStiffness_[i] = (float)200.0;
        commandedCartDamping_[i] = (float)0.7;
        commandedCartForcesAndTorques_[i] = (float)0.0;
    }
    FRI_->SetCommandedCartStiffness(commandedCartStiffness_);
    FRI_->SetCommandedCartDamping(commandedCartDamping_);
    FRI_->SetCommandedCartForcesAndTorques(commandedCartForcesAndTorques_);

    // ReflexxesAPI
    CycleTime_ = 0.01;
    RML_ = new ReflexxesAPI(LBR_MNJ, CycleTime_);
    VIP_ = new RMLVelocityInputParameters(LBR_MNJ);
    VOP_ = new RMLVelocityOutputParameters(LBR_MNJ);
    PIP_ = new RMLPositionInputParameters(LBR_MNJ);
    POP_ = new RMLPositionOutputParameters(LBR_MNJ);

    // lwr_action_server statemaschine
    state_ = ready;
}

// Destructor
lwr_action_server::~lwr_action_server()
{
    FRI_->StopRobot();
    delete FRI_;
    delete RML_;
    delete VIP_;
    delete VOP_;
    delete PIP_;
    delete POP_;
    delete kinematic_state_;
}

// lwr_action_server accepts only one goal at time and rejects other incoming goals.
void lwr_action_server::goalCB(GoalHandle gh)
{
    if (!goal_active_)
    {
        // TODO check if order and joint names are correct else gh.setRejected();
        gh.setAccepted();
        agh_ = gh;
        goal_active_ = true;
        state_ = trajectory;
    }
    else
    {
        ROS_INFO("lwr_action_server: gh.setRejected, because trajectory is active, please call cancel first!\n");
        gh.setRejected();
    }
}

// State JointPositionTrajectory of statemachine: computes and executes joint position trajectory
void lwr_action_server::executeJointPositionTrajectory()
{
    if (goal_active_)
    {  // Msg motion completion
        std_msgs::UInt32 mc;
        mc.data = 0;
        // send each trajectory point to FRI
        Goal g = agh_.getGoal();
        ros::Duration time_from_start;
        time_from_start.sec = 0;
        time_from_start.nsec = 0;
        ros::Time time_start = ros::Time::now();

        for (unsigned int i = 0; i < g->trajectory.points.size(); i++)
        {
            FRI_->WaitForKRCTick();
            if (!FRI_->IsMachineOK())
            {
                ROS_INFO("lwr_action_server: ERROR the machine is not ready anymore.");
                agh_.setAborted();
                exit(EXIT_FAILURE);
            }
            for (unsigned int j = 0; j < LBR_MNJ; j++)
            {
                commandedJointPositions_[j] = g->trajectory.points[i].positions[j];
            }
            FRI_->SetCommandedJointPositions(commandedJointPositions_);
            ROS_INFO("Sent joint values at %i . %i JPT  %1.4f %1.4f %1.4f %1.4f %1.4f %1.4f %1.4f\n",
                     ros::Time::now().sec, ros::Time::now().nsec, commandedJointPositions_[0],
                     commandedJointPositions_[1], commandedJointPositions_[2], commandedJointPositions_[3],
                     commandedJointPositions_[4], commandedJointPositions_[5], commandedJointPositions_[6]);
            publishJointStates();

            time_from_start = (ros::Time::now() - time_start);
            publishFeedback(g, i, time_from_start);

            mc.data = (i + 1) * 100 / g->trajectory.points.size();
            pub_motionCompletion_.publish(mc);

            publishLWRStates();
            // check cancel request
            if (cancel_goal_)
            {
                ROS_INFO("lwr_action_server: Goal canceled\n");
                agh_.setCanceled();
                cancel_goal_ = false;
                stopLwr();
                return;
            }
        }
        agh_.setSucceeded();
        goal_active_ = false;
        state_ = ready;
    }
}

// State JointImpedanceTrajectory of statemachine: computes and executes joint impedance trajectory
void lwr_action_server::executeJointImpedanceTrajectory()
{
    if (goal_active_)
    {
        // Msg motion completion
        std_msgs::UInt32 mc;
        mc.data = 0;
        // send each trajectory point to FRI
        Goal g = agh_.getGoal();
        ros::Duration time_from_start;
        time_from_start.sec = 0;
        time_from_start.nsec = 0;
        ros::Time time_start = ros::Time::now();

        for (unsigned int i = 0; i < g->trajectory.points.size(); i++)
        {
            FRI_->WaitForKRCTick();
            if (!FRI_->IsMachineOK())
            {
                ROS_INFO("lwr_action_server: ERROR the machine is not ready anymore.");
                agh_.setAborted();
                exit(EXIT_FAILURE);
            }
            for (unsigned int j = 0; j < LBR_MNJ; j++)
            {
                commandedJointPositions_[j] = g->trajectory.points[i].positions[j];
            }
            FRI_->SetCommandedJointPositions(commandedJointPositions_);
            FRI_->SetCommandedJointStiffness(commandedStiffness_);
            FRI_->SetCommandedJointDamping(commandedDamping_);
            FRI_->SetCommandedJointTorques(commandedTorquesInNm_);

            publishJointStates();

            time_from_start = (ros::Time::now() - time_start);
            publishFeedback(g, i, time_from_start);

            mc.data = (i + 1) * 100 / g->trajectory.points.size();
            pub_motionCompletion_.publish(mc);

            publishLWRStates();
            // check cancel request
            if (cancel_goal_)
            {
                ROS_INFO("lwr_action_server: Goal canceled\n");
                agh_.setCanceled();
                cancel_goal_ = false;
                stopLwr();
                return;
            }
        }
        agh_.setSucceeded();
        goal_active_ = false;
        state_ = ready;
    }
}

// State CartImpedanceTrajectory of statemachine: computes and executes cartesian impedance trajectory
void lwr_action_server::executeCartImpedanceTrajectory()
{
    if (goal_active_)
    {
        // Msg motion completion
        std_msgs::UInt32 mc;
        mc.data = 0;
        // send each trajectory point to FRI
        Goal g = agh_.getGoal();
        ros::Duration time_from_start;
        time_from_start.sec = 0;
        time_from_start.nsec = 0;
        ros::Time time_start = ros::Time::now();
        std::string names[] = { "lwr_arm_0_joint", "lwr_arm_1_joint", "lwr_arm_2_joint", "lwr_arm_3_joint",
                                "lwr_arm_4_joint", "lwr_arm_5_joint", "lwr_arm_6_joint" };
        sensor_msgs::JointState joint_values;

        for (unsigned int i = 0; i < g->trajectory.points.size(); i++)
        {
            FRI_->WaitForKRCTick();
            if (!FRI_->IsMachineOK())
            {
                ROS_INFO("lwr_action_server: ERROR the machine is not ready anymore.");
                agh_.setAborted();
                exit(EXIT_FAILURE);
            }
            // ROS_INFO("Time1 %i . %i \n", ros::Time::now().sec, ros::Time::now().nsec);
            for (int j = 0; j < LBR_MNJ; j++)
            {
                joint_values.name.push_back(names[j]);
                joint_values.position.push_back(g->trajectory.points[i].positions[j]);
            }

            kinematic_state_->setVariableValues(joint_values);
            // name of your specific TCP (tool frame, or thatever you call it), that you have defined in your URDF file.
            // Check also the selected TOOL on your KRC.
            const Eigen::Isometry3d& end_effector_state_temp = kinematic_state_->getGlobalLinkTransform("tip");

            geometry_msgs::Pose pose;
            tf::poseEigenToMsg(end_effector_state_temp, pose);
            /*  ROS_INFO("position_x %f", pose.position.x);
                ROS_INFO("position_y %f", pose.position.y);
                ROS_INFO("position_z %f", pose.position.z);
                ROS_INFO("orientation_x %f", pose.orientation.x);
                ROS_INFO("orientation_y %f", pose.orientation.y);
                ROS_INFO("orientation_z %f", pose.orientation.z);
                ROS_INFO("orientation_w %f", pose.orientation.w);
            */
            ROS_INFO("Time2 %i . %i \n", ros::Time::now().sec, ros::Time::now().nsec);

            // This settings depending on your LWR setup. For example the LWR of the University Hamburg(TAMS) is placed
            // 0.804m above the world frame and is turned 180 Degree. URDF specific changes <origin rpy="0 0 3.1415"
            // xyz="0 0 0.804"/> for TAMS - University Hamburg.
            pose.position.z = pose.position.z - 0.804;
            pose.orientation.z = pose.orientation.z - 0.70711;
            pose.orientation.w = pose.orientation.w + 0.70711;

            Eigen::Isometry3d end_effector_state;
            tf::poseMsgToEigen(pose, end_effector_state);

            const double* pdata = end_effector_state.matrix().data();
            /*
               ROS_INFO("*(pdata+0) %f\n",*(pdata+0));
               ROS_INFO("*(pdata+4) %f\n",pdata[4]);
               ROS_INFO("*(pdata+8) %f\n",pdata[8]);
               ROS_INFO("*(pdata+12) %f\n",pdata[12]);

                ROS_INFO("*(pdata+1) %f\n",*(pdata+1));
               ROS_INFO("*(pdata+5) %f\n",pdata[5]);
               ROS_INFO("*(pdata+9) %f\n",pdata[9]);
               ROS_INFO("*(pdata+13) %f\n",pdata[13]);

               ROS_INFO("*(pdata+2) %f\n",*(pdata+2));
               ROS_INFO("*(pdata+6) %f\n",pdata[6]);
               ROS_INFO("*(pdata+10) %f\n",pdata[10]);
               ROS_INFO("*(pdata+14) %f\n",pdata[14]);

               float measuredCartPose[NUMBER_OF_FRAME_ELEMENTS];
               FRI_->GetMeasuredCartPose(measuredCartPose);

               for (unsigned int j = 0; j < NUMBER_OF_FRAME_ELEMENTS; j++)
               {
                   ROS_INFO("measuredCartPose[ %d ]  %f", j ,measuredCartPose[j] );
               }
           */
            commandedCartPose_[0] = pdata[0];
            commandedCartPose_[4] = pdata[1];
            commandedCartPose_[8] = pdata[2];

            commandedCartPose_[1] = pdata[4];
            commandedCartPose_[5] = pdata[5];
            commandedCartPose_[9] = pdata[6];

            commandedCartPose_[2] = pdata[8];
            commandedCartPose_[6] = pdata[9];
            commandedCartPose_[10] = pdata[10];

            commandedCartPose_[3] = pdata[12];
            commandedCartPose_[7] = pdata[13];
            commandedCartPose_[11] = pdata[14];

            // now send data to robot
            FRI_->SetCommandedCartStiffness(commandedCartStiffness_);
            FRI_->SetCommandedCartDamping(commandedCartDamping_);
            FRI_->SetCommandedCartForcesAndTorques(commandedCartForcesAndTorques_);
            FRI_->SetCommandedCartPose(commandedCartPose_);

            // ROS_INFO("RUN %i\n", i);
            // ROS_INFO("Time3 %i . %i \n", ros::Time::now().sec, ros::Time::now().nsec);
            publishJointStates();

            time_from_start = (ros::Time::now() - time_start);
            publishFeedback(g, i, time_from_start);

            mc.data = (i + 1) * 100 / g->trajectory.points.size();
            pub_motionCompletion_.publish(mc);

            publishLWRStates();
            // check cancel request
            if (cancel_goal_)
            {
                ROS_INFO("lwr_action_server: Goal canceled\n");
                agh_.setCanceled();
                cancel_goal_ = false;
                stopLwr();
                return;
            }
        }
        agh_.setSucceeded();
        goal_active_ = false;
    }
}

// Cancel Goal
void lwr_action_server::cancelCB(GoalHandle gh)
{
    // cancel FRI
    cancel_goal_ = true;
    // cancel Goalhandle
    goal_active_ = false;
}

// Stops with a "soft" motion the LWR execution
void lwr_action_server::stopLwr()
{
    // load first position
    // FRI_->GetMeasuredJointPositions(measuredJointPositions_);
    for (unsigned int j = 0; j < LBR_MNJ; j++)
    {
        // VIP_->CurrentPositionVector->VecData[j] = (double) DEG(measuredJointPositions_[j]);
        VIP_->CurrentPositionVector->VecData[j] = (double)DEG(commandedJointPositions_[j]);
        VIP_->CurrentVelocityVector->VecData[j] = (double)DEG(velocity_new_[j]);
        VIP_->TargetVelocityVector->VecData[j] = (double)DEG(0.0);
        VIP_->MaxAccelerationVector->VecData[j] = (double)DEG(0.5);
        VIP_->SelectionVector->VecData[j] = true;
    }
    // execute Trajectory
    int resultValue = 0;
    // target position-based trajectory generation.
    while ((FRI_->IsMachineOK()) && (resultValue != ReflexxesAPI::RML_FINAL_STATE_REACHED))
    {
        resultValue = RML_->RMLVelocity(*VIP_, VOP_, VFlags_);
        FRI_->WaitForKRCTick();
        if (resultValue < 0)
        {
            printf("lwr_action_server: ERROR during trajectory generation  (%d).\n", resultValue);
            break;
        }
        for (unsigned int j = 0; j < LBR_MNJ; j++)
        {
            commandedJointPositions_[j] = RAD((double)(VOP_->NewPositionVector->VecData[j]));
        }
        FRI_->SetCommandedJointPositions(commandedJointPositions_);
        ROS_INFO("Sent joint values at %i . %i STP %1.4f %1.4f %1.4f %1.4f %1.4f %1.4f %1.4f\n", ros::Time::now().sec,
                 ros::Time::now().nsec, commandedJointPositions_[0], commandedJointPositions_[1],
                 commandedJointPositions_[2], commandedJointPositions_[3], commandedJointPositions_[4],
                 commandedJointPositions_[5], commandedJointPositions_[6]);

        // prepare for next iteration
        *VIP_->CurrentPositionVector = *VOP_->NewPositionVector;
        *VIP_->CurrentVelocityVector = *VOP_->NewVelocityVector;
        *VIP_->CurrentAccelerationVector = *VOP_->NewAccelerationVector;
        ROS_INFO("Velocity Vector %1.4f %1.4f %1.4f %1.4f %1.4f %1.4f %1.4f\n", VOP_->NewVelocityVector->VecData[0],
                 VOP_->NewVelocityVector->VecData[1], VOP_->NewVelocityVector->VecData[2],
                 VOP_->NewVelocityVector->VecData[3], VOP_->NewVelocityVector->VecData[4],
                 VOP_->NewVelocityVector->VecData[5], VOP_->NewVelocityVector->VecData[6]);
        ROS_INFO("Acceleration Vector %1.4f %1.4f %1.4f %1.4f %1.4f %1.4f %1.4f\n",
                 VOP_->NewAccelerationVector->VecData[0], VOP_->NewAccelerationVector->VecData[1],
                 VOP_->NewAccelerationVector->VecData[2], VOP_->NewAccelerationVector->VecData[3],
                 VOP_->NewAccelerationVector->VecData[4], VOP_->NewAccelerationVector->VecData[5],
                 VOP_->NewAccelerationVector->VecData[6]);
        // publish states
        publishJointStates();
        publishLWRStates();
    }
}

// Publish feedback of trajectory execution.
// param: current Goal; current trajectory point number; current duration from trajectory start
void lwr_action_server::publishFeedback(Goal g, unsigned int counter, ros::Duration time_from_start)
{
    FRI_->GetMeasuredJointPositions(measuredJointPositions_);
    FRI_->GetMeasuredJointTorques(commandedTorquesInNm_);
    for (unsigned int j = 0; j < LBR_MNJ; j++)
    {
        acceleration_[j] = (velocity_new_[j] - velocity_old_[j]) / (time_new_.toSec() - time_old_.toSec());
    }
    // fill trajectory_msgs/JointTrajectoryPoints
    trajectory_msgs::JointTrajectoryPoint actual;
    trajectory_msgs::JointTrajectoryPoint desired;
    trajectory_msgs::JointTrajectoryPoint error;
    for (unsigned int j = 0; j < LBR_MNJ; j++)
    {
        actual.positions.push_back((double)measuredJointPositions_[j]);
        desired.positions.push_back(g->trajectory.points[counter].positions[j]);
        error.positions.push_back(fabs(g->trajectory.points[counter].positions[j]) - fabs(measuredJointPositions_[j]));

        actual.velocities.push_back(velocity_new_[j]);
        desired.velocities.push_back(g->trajectory.points[counter].velocities[j]);
        error.velocities.push_back(fabs(g->trajectory.points[counter].velocities[j]) - fabs(velocity_new_[j]));

        actual.accelerations.push_back(acceleration_[j]);
        desired.accelerations.push_back(g->trajectory.points[counter].accelerations[j]);
        error.accelerations.push_back(fabs(g->trajectory.points[counter].accelerations[j]) - fabs(acceleration_[j]));

        actual.effort.push_back(commandedTorquesInNm_[j]);
        desired.effort.push_back(NAN);
        error.effort.push_back(NAN);

        actual.time_from_start = time_from_start;
        desired.time_from_start = g->trajectory.points[counter].time_from_start;
        error.time_from_start = g->trajectory.points[counter].time_from_start - time_from_start;
    }

    // fill control_msgs/FollowJointTrajectoryFeedback
    control_msgs::FollowJointTrajectoryFeedback fjtf;
    fjtf.header.seq = seqCounterFJTF_;
    fjtf.header.stamp = ros::Time::now();
    fjtf.header.frame_id = g->trajectory.header.frame_id;
    fjtf.joint_names = g->trajectory.joint_names;
    fjtf.actual = actual;
    fjtf.desired = desired;
    fjtf.error = error;
    agh_.publishFeedback(fjtf);
    seqCounterFJTF_++;
}

// Publish current joint states.
void lwr_action_server::publishJointStates()
{
    pthread_mutex_lock(&mutex);
    std::string names[] = { "lwr_arm_0_joint", "lwr_arm_1_joint", "lwr_arm_2_joint", "lwr_arm_3_joint",
                            "lwr_arm_4_joint", "lwr_arm_5_joint", "lwr_arm_6_joint" };

    FRI_->GetMeasuredJointPositions(measuredJointPositions_);
    FRI_->GetMeasuredJointTorques(commandedTorquesInNm_);

    time_old_ = time_new_;
    time_new_ = ros::Time::now();

    for (unsigned int j = 0; j < LBR_MNJ; j++)
    {
        Position_old_[j] = Position_new_[j];
        Position_new_[j] = measuredJointPositions_[j];
        velocity_old_[j] = velocity_new_[j];
        velocity_new_[j] = (Position_new_[j] - Position_old_[j]) / (time_new_.toSec() - time_old_.toSec());
    }

    sensor_msgs::JointState js;
    js.header.seq = seqCounterJointStates_;
    js.header.stamp = ros::Time::now();
    js.header.frame_id = "calib_lwr_arm_base_link";

    for (int i = 0; i < LBR_MNJ; i++)
    {  // ROS_INFO("names %f value \n",JointValuesInRad_[i]);
        js.name.push_back(names[i]);
        js.position.push_back(measuredJointPositions_[i]);
        js.velocity.push_back(velocity_new_[i]);
        js.effort.push_back(commandedTorquesInNm_[i]);
    }

    pub_joints_.publish(js);
    seqCounterJointStates_++;
    pthread_mutex_unlock(&mutex);
}

/*
//new untested Version
// Publish current LWR states.
void publishLWRStates()
{
    FRI_->GetDriveTemperatures(driveTemperatues_);
   FRI_->GetMeasuredJointPositions(measuredJointPositions_);
   FRI_->GetCommandedJointPositions(commandedJointPositions_);
   FRI_->GetCommandedJointPositionOffsets(commandedJointPositionOffsets_);
   FRI_->GetMeasuredJointTorques(measuredJointTorques_);
   FRI_->GetEstimatedExternalJointTorques(estimatedExternalJointTorques_);
    FRI_->GetEstimatedExternalCartForcesAndTorques(estimatedExternalCartForcesAndTorques_);

    std_msgs::Float32MultiArray driveTemperatues, measuredJointPositions, commandedJointPositions,
                               commandedJointPositionOffsets, measuredJointTorques,
                               estimatedExternalJointTorques, estimatedExternalCartForcesAndTorques;

   for (unsigned int j = 0; j < LBR_MNJ; j++)
   {
       driveTemperatues.data.push_back(driveTemperatues_[j]);
       measuredJointPositions.data.push_back(measuredJointPositions_[j]);
       commandedJointPositions.data.push_back(commandedJointPositions_[j]);
       commandedJointPositionOffsets.data.push_back(commandedJointPositionOffsets_[j]);
       measuredJointTorques.data.push_back(measuredJointTorques_[j]);
       estimatedExternalJointTorques.data.push_back(estimatedExternalJointTorques_[j]);
   }

   for (unsigned int j = 0; j < NUMBER_OF_CART_DOFS; j++)
   {
       estimatedExternalCartForcesAndTorques.data.push_back(estimatedExternalCartForcesAndTorques_[j]);
   }

   pub_estimatedExternalCartForcesAndTorques_.publish(estimatedExternalCartForcesAndTorques);
   pub_driveTemperatues_.publish(driveTemperatues);
   pub_measuredJointPositions_.publish(measuredJointPositions);
   pub_commandedJointPositions_.publish(commandedJointPositions);
   pub_commandedJointPositionOffsets_.publish(commandedJointPositionOffsets);
   pub_measuredJointTorques_.publish(measuredJointTorques);
   pub_estimatedExternalJointTorques_.publish(estimatedExternalJointTorques);

   //only if cart impedance mode
   if (currentControlScheme_ == FastResearchInterface::CART_IMPEDANCE_CONTROL)
   {
       FRI_->GetMeasuredCartPose(measuredCartPose_);
       FRI_->GetCommandedCartPose(commandedCartPose_);
       FRI_->GetCommandedCartPoseOffsets(commandedCartPoseOffsets_);
       std_msgs::Float32MultiArray measuredCartPose, commandedCartPose, commandedCartPoseOffsets;

       for (unsigned int j = 0; j < NUMBER_OF_FRAME_ELEMENTS; j++)
       {
           measuredCartPose.data.push_back(measuredCartPose_[j]);
           commandedCartPose.data.push_back(commandedCartPose_[j]);
           measuredCartPose.data.push_back(commandedCartPoseOffsets_[j]);
       }
       pub_measuredCartPose_.publish(measuredCartPose);
       pub_commandedCartPose_.publish(commandedCartPose);
       pub_commandedCartPoseOffsets_.publish(commandedCartPoseOffsets);
   }
}
*/

// Publish current LWR states.
void lwr_action_server::publishLWRStates()
{
    FRI_->GetEstimatedExternalJointTorques(estimatedExternalCartForcesAndTorques_);
    FRI_->GetEstimatedExternalCartForcesAndTorques(estimatedExternalCartForcesAndTorques_);
    FRI_->GetDriveTemperatures(driveTemperatues_);

    std_msgs::Float32MultiArray DriveTemperatues;
    std_msgs::Float32MultiArray EstimatedExternalJointTorquesInNm;
    std_msgs::Float32MultiArray EstimatedExternalCartForcesAndTorques;

    for (unsigned int j = 0; j < LBR_MNJ; j++)
    {
        DriveTemperatues.data.push_back(driveTemperatues_[j]);
        EstimatedExternalJointTorquesInNm.data.push_back(estimatedExternalCartForcesAndTorques_[j]);
    }

    for (unsigned int j = 0; j < NUMBER_OF_FRAME_ELEMENTS; j++)
    {
        EstimatedExternalCartForcesAndTorques.data.push_back(estimatedExternalCartForcesAndTorques_[j]);
    }

    pub_estimatedExternalCartForcesAndTorques_.publish(EstimatedExternalCartForcesAndTorques);
    pub_driveTemperatues_.publish(DriveTemperatues);
    pub_estimatedExternalCartForcesAndTorques_.publish(EstimatedExternalJointTorquesInNm);
}

// Get the current running Fast Research Interface control strategie.
unsigned int lwr_action_server::getCurrentControlScheme()
{
    return currentControlScheme_;
}

// Set FRI control strategie. Possible are
// FastResearchInterface::JOINT_POSITION_CONTROL (number 10)
// FastResearchInterface::JOINT_IMPEDANCE_CONTROL (number 30)
// FastResearchInterface::CART_IMPEDANCE_CONTROL (number 20)
void lwr_action_server::setCurrentControlScheme(const std_msgs::UInt32::ConstPtr& msg)
{
    int resultValue = 0;
    currentControlScheme_ = msg->data;
    resultValue = FRI_->StartRobot(currentControlScheme_);
    if ((resultValue != EOK) && (resultValue != EALREADY))
    {
        ROS_INFO("lwr_action_server: An error occurred during starting up the robot...\n");
        exit(EXIT_FAILURE);
    }
}

// Set the desired joint position vector into the data telegram to be send to the KRC unit.
void lwr_action_server::setJointPosition(const std_msgs::Float32MultiArray::ConstPtr& msg)
{
    if ((currentControlScheme_ == FastResearchInterface::JOINT_POSITION_CONTROL) or
        (currentControlScheme_ == FastResearchInterface::JOINT_IMPEDANCE_CONTROL))
    {
        for (int i = 0; i < LBR_MNJ; i++)
        {
            commandedJointPositions_[i] = msg->data[i];
        }
        FRI_->SetCommandedJointTorques(commandedJointPositions_);
    }
}

// Set the desired joint damping vector into the data telegram to be send to the KRC unit.
void lwr_action_server::setJointDamping(const std_msgs::Float32MultiArray::ConstPtr& msg)
{
    if (currentControlScheme_ == FastResearchInterface::JOINT_IMPEDANCE_CONTROL)
    {
        for (int i = 0; i < LBR_MNJ; i++)
        {
            commandedDamping_[i] = msg->data[i];
        }
        FRI_->SetCommandedJointDamping(commandedDamping_);
    }
}

// Set the desired joint stiffness vector into the data telegram to be send to the KRC unit.
void lwr_action_server::setJointStiffness(const std_msgs::Float32MultiArray::ConstPtr& msg)
{
    if (currentControlScheme_ == FastResearchInterface::JOINT_IMPEDANCE_CONTROL)
    {
        for (int i = 0; i < LBR_MNJ; i++)
        {
            commandedStiffness_[i] = msg->data[i];
        }
        FRI_->SetCommandedJointStiffness(commandedStiffness_);
    }
}

// Set the desired joint torque vector into the data telegram to be send to the KRC unit.
void lwr_action_server::setJointTorques(const std_msgs::Float32MultiArray::ConstPtr& msg)
{
    if ((currentControlScheme_ == FastResearchInterface::JOINT_IMPEDANCE_CONTROL) or
        (currentControlScheme_ == FastResearchInterface::JOINT_TORQUE_CONTROL))
    {
        for (int i = 0; i < LBR_MNJ; i++)
        {
            commandedTorquesInNm_[i] = msg->data[i];
        }
        FRI_->SetCommandedJointTorques(commandedTorquesInNm_);
    }
}

// Set the desired Cartesian pose frame into the data telegram to be send to the KRC unit.
void lwr_action_server::setCartPose(const std_msgs::Float32MultiArray::ConstPtr& msg)
{
    if (currentControlScheme_ == FastResearchInterface::CART_IMPEDANCE_CONTROL)
    {
        for (int i = 0; i < LBR_MNJ; i++)
        {
            commandedCartPose_[i] = msg->data[i];
        }
        FRI_->SetCommandedCartPose(commandedCartPose_);
    }
}

// Set the desired Cartesian damping vector into the data telegram to be send to the KRC unit.
void lwr_action_server::setCartDamping(const std_msgs::Float32MultiArray::ConstPtr& msg)
{
    if (currentControlScheme_ == FastResearchInterface::CART_IMPEDANCE_CONTROL)
    {
        for (int i = 0; i < NUMBER_OF_CART_DOFS; i++)
        {
            commandedCartDamping_[i] = msg->data[i];
        }
        FRI_->SetCommandedCartDamping(commandedCartDamping_);
    }
}

// Set the desired Cartesian stiffness vector into the data telegram to be send to the KRC unit.
void lwr_action_server::setCartStiffness(const std_msgs::Float32MultiArray::ConstPtr& msg)
{
    if (currentControlScheme_ == FastResearchInterface::CART_IMPEDANCE_CONTROL)
    {
        for (int i = 0; i < NUMBER_OF_CART_DOFS; i++)
        {
            commandedCartStiffness_[i] = msg->data[i];
        }
        FRI_->SetCommandedCartStiffness(commandedCartStiffness_);
    }
}

// Set the desired Cartesian force/torque vector into the data telegram to be send to the KRC unit.
void lwr_action_server::setCartForcesAndTorques(const std_msgs::Float32MultiArray::ConstPtr& msg)
{
    if (currentControlScheme_ == FastResearchInterface::CART_IMPEDANCE_CONTROL)
    {
        for (int i = 0; i < NUMBER_OF_CART_DOFS; i++)
        {
            commandedCartForcesAndTorques_[i] = msg->data[i];
        }
        FRI_->SetCommandedCartForcesAndTorques(commandedCartForcesAndTorques_);
    }
}

// Cancel trajectoryexecution with a soft trajectory stop
void lwr_action_server::setCancelLWRTrajectory(const std_msgs::Bool::ConstPtr& msg)
{
    if (msg->data == true)
    {
        // cancel FRI
        cancel_goal_ = true;
        // cancel Goalhandle
        goal_active_ = false;
    }
}

// Set and executes the desired joint position goal with the ReflexxesType II Library.
void lwr_action_server::setJntPosGoal(const ros_fri::RMLPositionInputParameters::ConstPtr& msg)
{
    pthread_mutex_lock(&mutex);
    for (unsigned int j = 0; j < LBR_MNJ; j++)
    {
        msgPOP_.TargetPositionVector.push_back(msg->TargetPositionVector[j]);
        msgPOP_.TargetVelocityVector.push_back(msg->TargetVelocityVector[j]);
        msgPOP_.MaxAccelerationVector.push_back(msg->MaxAccelerationVector[j]);
        msgPOP_.MaxVelocityVector.push_back(msg->MaxVelocityVector[j]);
    }
    if (state_ == trajectory)
    {
        // cancel FRI
        cancel_goal_ = true;
    }
    state_ = jntPosGoal;
    pthread_mutex_unlock(&mutex);
}

// State JntPosGoal of statemachine: computes and executes joint position goal
void lwr_action_server::stateJntPosGoal()
{
    pthread_mutex_lock(&mutex);
    // load first position
    FRI_->GetMeasuredJointPositions(measuredJointPositions_);
    for (unsigned int j = 0; j < LBR_MNJ; j++)
    {
        PIP_->CurrentPositionVector->VecData[j] = (double)DEG(measuredJointPositions_[j]);
        PIP_->CurrentVelocityVector->VecData[j] = (double)DEG(velocity_new_[j]);
        PIP_->SelectionVector->VecData[j] = true;
        PIP_->TargetPositionVector->VecData[j] = (double)DEG(msgPOP_.TargetPositionVector[j]);
        PIP_->TargetVelocityVector->VecData[j] = (double)DEG(msgPOP_.TargetVelocityVector[j]);
        PIP_->MaxAccelerationVector->VecData[j] = (double)DEG(msgPOP_.MaxAccelerationVector[j]);
        PIP_->MaxVelocityVector->VecData[j] = (double)DEG(msgPOP_.MaxVelocityVector[j]);
    }
    // clean up
    ros_fri::RMLPositionInputParameters empty;
    msgPOP_ = empty;

    ROS_INFO("CheckForValidity  %d \n", PIP_->CheckForValidity());
    // execute Trajectory
    int resultValue = 0;

    // target position-based trajectory generation.
    while ((FRI_->IsMachineOK()) && (resultValue != ReflexxesAPI::RML_FINAL_STATE_REACHED))
    {
        FRI_->WaitForKRCTick();
        resultValue = RML_->RMLPosition(*PIP_, POP_, PFlags_);
        if (resultValue < 0)
        {
            printf("lwr_action_server: ERROR during trajectory generation  (%d).\n", resultValue);
            break;
        }
        for (unsigned int j = 0; j < LBR_MNJ; j++)
        {
            commandedJointPositions_[j] = RAD((double)(POP_->NewPositionVector->VecData[j]));
        }
        FRI_->SetCommandedJointPositions(commandedJointPositions_);

        // prepare for next iteration
        *PIP_->CurrentPositionVector = *POP_->NewPositionVector;
        *PIP_->CurrentVelocityVector = *POP_->NewVelocityVector;
        // publish states
        publishJointStates();
        publishLWRStates();

        /* uncomment for replannin if publisher is only publishing useful topics(not the same)
         //cancel and prepare replanning
        if (state_ == jntPosGoal)
        {
             stopLwr();
        }
        */
    }
    // allow accept new Goalhandle in trajectory state
    goal_active_ = false;
    state_ = ready;
    pthread_mutex_unlock(&mutex);
}

// Set and execute the desired joint velocity goal with the ReflexxesType II Library.
void lwr_action_server::setJntVelGoal(const ros_fri::RMLVelocityInputParameters::ConstPtr& msg)
{
    for (unsigned int j = 0; j < LBR_MNJ; j++)
    {
        msgVOP_.TargetVelocityVector.push_back(msg->TargetVelocityVector[j]);
        msgVOP_.MaxAccelerationVector.push_back(msg->MaxAccelerationVector[j]);
    }
    if (state_ == trajectory)
    {
        // cancel FRI
        cancel_goal_ = true;
    }
    state_ = jntVelGoal;
}

// State JntVelGoal: computes and executes joint velocity goal
void lwr_action_server::stateJntVelGoal()
{
    // load first position
    FRI_->GetMeasuredJointPositions(measuredJointPositions_);
    for (unsigned int j = 0; j < LBR_MNJ; j++)
    {
        VIP_->CurrentPositionVector->VecData[j] = (double)DEG(measuredJointPositions_[j]);
        VIP_->CurrentVelocityVector->VecData[j] = (double)DEG(velocity_new_[j]);
        VIP_->TargetVelocityVector->VecData[j] = (double)DEG(msgVOP_.TargetVelocityVector[j]);
        VIP_->MaxAccelerationVector->VecData[j] = (double)DEG(msgVOP_.MaxAccelerationVector[j]);
        VIP_->SelectionVector->VecData[j] = true;
    }

    // clean up
    ros_fri::RMLVelocityInputParameters empty;
    msgVOP_ = empty;
    // execute Trajectory
    int resultValue = 0;
    // target position-based trajectory generation.
    while ((FRI_->IsMachineOK()) && (resultValue != ReflexxesAPI::RML_FINAL_STATE_REACHED))
    {
        FRI_->WaitForKRCTick();
        resultValue = RML_->RMLVelocity(*VIP_, VOP_, VFlags_);
        if (resultValue < 0)
        {
            printf("lwr_action_server: ERROR during trajectory generation  (%d).\n", resultValue);
            break;
        }
        for (unsigned int j = 0; j < LBR_MNJ; j++)
        {
            commandedJointPositions_[j] = RAD((double)(VOP_->NewPositionVector->VecData[j]));
        }
        FRI_->SetCommandedJointPositions(commandedJointPositions_);
        // prepare for next iteration
        *VIP_->CurrentPositionVector = *VOP_->NewPositionVector;
        *VIP_->CurrentVelocityVector = *VOP_->NewVelocityVector;
        *VIP_->CurrentAccelerationVector = *VOP_->NewAccelerationVector;
        // publish states
        publishJointStates();
        publishLWRStates();
    }
    // allow to accept new Goalhandle in trajectory state
    goal_active_ = false;
    state_ = ready;
}

// Gets current state of lwr_action_server statemachine.
statemaschine lwr_action_server::getstate()
{
    return state_;
}

// Starts the lwr_action_server node.
int lwr_action_server::run()
{
    while (ros::ok())
    {
        switch (getstate())
        {
            case ready:
                FRI_->WaitForKRCTick();
                break;
            case trajectory:
                switch (getCurrentControlScheme())
                {
                    case FastResearchInterface::JOINT_POSITION_CONTROL:
                        executeJointPositionTrajectory();
                        break;

                    case FastResearchInterface::JOINT_IMPEDANCE_CONTROL:
                        executeJointImpedanceTrajectory();
                        break;

                    case FastResearchInterface::CART_IMPEDANCE_CONTROL:
                        executeCartImpedanceTrajectory();
                        break;

                    default:
                        ROS_INFO("lwr_action_server ERROR: Not supported FRIMODE! Select JOINT_POSITION_CONTROL, "
                                 "JOINT_IMPEDANCE_CONTROL or CART_IMPEDANCE_CONTROL \n");
                        return -1;
                }
                break;
            case jntPosGoal:
                stateJntPosGoal();
                break;
            case jntVelGoal:
                stateJntVelGoal();
                break;
        }
        publishJointStates();
        publishLWRStates();
        ros::spinOnce();
    }
    return 0;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "lwr_action_server");
    lwr_action_server lwr_action_server(ros::this_node::getName());
    ros::AsyncSpinner spinner(1);
    spinner.start();
    ros::Rate loop_rate(100);  // 100Hz
    lwr_action_server.run();
}
