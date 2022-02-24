// Johannes Liebrecht --- 8liebrec@informatik.uni-hamburg.de
#include <string>
#include <ros/ros.h>
#include <actionlib/server/action_server.h>
#include <actionlib/server/server_goal_handle.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Pose.h>

#include <TypeIRML.h>
#include <FastResearchInterface.h>
#include <LinuxAbstraction.h>

#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/UInt32.h>
#include <std_msgs/Bool.h>

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_model/robot_model.h>
#include <eigen_conversions/eigen_msg.h>

#include <moveit_msgs/GetPositionFK.h>
#include <tf/transform_listener.h>

#ifndef PI
#define PI 3.1415926535897932384626433832795
#endif

#ifndef RAD
#define RAD(A) ((A)*PI / 180.0)
#endif

#ifndef DEG
#define DEG(A) ((A)*180.0 / PI)
#endif

#define NUMBER_OF_FRAME_ELEMENTS 12
#define NUMBER_OF_CART_DOFS 6

static pthread_mutex_t mutex = PTHREAD_MUTEX_INITIALIZER;
typedef typename actionlib::ActionServer<control_msgs::FollowJointTrajectoryAction>::GoalHandle GoalHandle;
typedef boost::shared_ptr<const control_msgs::FollowJointTrajectoryGoal> Goal;

class lwr_action_server
{
protected:
    ros::NodeHandle nh_;
    // NodeHandle instance must be created before this line. Otherwise strange error may occur.
    actionlib::ActionServer<control_msgs::FollowJointTrajectoryAction> as_;
    std::string action_name_;
    bool goal_active_, cancel_goal_;
    int seq_counterFJTF_, seqCounterJointStates_, seqCounterFK_;
    GoalHandle agh_;

    // Fri
    FastResearchInterface* FRI_;
    unsigned int FRIMode_;

    double Position_old_[LBR_MNJ], Position_new_[LBR_MNJ];
    ros::Time time_old_, time_new_, time_delta_;
    double velocity_new_[LBR_MNJ], velocity_old_[LBR_MNJ];
    double acceleration_[LBR_MNJ];

    float commandedJointPosition_[LBR_MNJ];
    float measuredJointValuesInRad_[LBR_MNJ];
    // float TorqueValuesInNm_[LBR_MNJ];

    float estimatedExternalJointTorquesInNm_[LBR_MNJ];
    // float EstimatedExternalCartForcesAndTorques_[NUMBER_OF_CART_DOFS]; //TODO check
    float driveTemperatues_[LBR_MNJ];
    bool setCancelOnForceAtTCP_;

    float commandedTorquesInNm_[LBR_MNJ], commandedStiffness_[LBR_MNJ], commandedDamping_[LBR_MNJ];

    float commandedCartForcesAndTorques_[NUMBER_OF_CART_DOFS], commandedCartStiffness_[NUMBER_OF_CART_DOFS],
        commandedCartDamping_[NUMBER_OF_CART_DOFS], estimatedExternalCartForcesAndTorques_[NUMBER_OF_CART_DOFS],
        commandedPose_[NUMBER_OF_FRAME_ELEMENTS], measuredPose_[NUMBER_OF_FRAME_ELEMENTS],
        measuredJointTorques_[LBR_MNJ];

    // Msg
    std_msgs::UInt32 mc_;

    ros::Publisher pub_joints_;
    ros::Publisher pub_motionCompletion_;
    ros::Publisher pub_estimatedExternalCartForcesAndTorques_;
    ros::Publisher pub_driveTemperatues_;
    ros::Publisher pub_estimatedExternalJointTorquesInNm_;
    ros::Publisher pub_measuredPose_;

    ros::Subscriber sub_setFRIMode_;
    ros::Subscriber sub_setJointDamping_;
    ros::Subscriber sub_setJointStiffness_;
    ros::Subscriber sub_setJointTorques_;
    ros::Subscriber sub_setCartDamping_;
    ros::Subscriber sub_setCartStiffness_;
    ros::Subscriber sub_setCartForcesAndTorques_;
    ros::Subscriber sub_setCancelOnForceAtTCP_;
    ros::Subscriber sub_cancelLWRTrajectory_;
    ros::Subscriber sub_setJointPosition_;
    ros::Subscriber sub_setCartPose_;

    ros::ServiceClient ser_fk_;

    double CycleTime_;
    TypeIRML* RML_;
    TypeIRMLInputParameters* IP_;
    TypeIRMLOutputParameters* OP_;

public:
    lwr_action_server(std::string name)
        : as_(nh_, name, false), action_name_("lwr_action_server/joint_trajectory_action")
    {
        // register the goal and cancel callbacks
        as_.registerGoalCallback(boost::bind(&lwr_action_server::goalCB, this, _1));
        as_.registerCancelCallback(boost::bind(&lwr_action_server::cancelCB, this, _1));
        as_.start();

        // publisher
        pub_joints_ = nh_.advertise<sensor_msgs::JointState>("/joint_states", 1);
        pub_motionCompletion_ = nh_.advertise<std_msgs::UInt32>("/lwr_motion_completion", 1);

        pub_estimatedExternalCartForcesAndTorques_ =
            nh_.advertise<std_msgs::Float32MultiArray>("/lwr_estimatedExternalCartForcesAndTorques", 1);
        pub_driveTemperatues_ = nh_.advertise<std_msgs::Float32MultiArray>("/lwr_driveTemperatues", 1);
        pub_estimatedExternalJointTorquesInNm_ =
            nh_.advertise<std_msgs::Float32MultiArray>("/lwr_estimatedExternalJointTorquesInNm", 1);
        pub_measuredPose_ = nh_.advertise<std_msgs::Float32MultiArray>("/lwr_measuredPose", 1);

        // subscriber
        sub_setFRIMode_ = nh_.subscribe("/lwr_setFRIMode", 1, &lwr_action_server::setFRIModeSubscriber, this);
        sub_setJointDamping_ = nh_.subscribe("/lwr_setJointDamping", 1, &lwr_action_server::setJointDamping, this);
        sub_setJointStiffness_ =
            nh_.subscribe("/lwr_setJointStiffness", 1, &lwr_action_server::setJointStiffness, this);
        sub_setJointTorques_ = nh_.subscribe("/lwr_setJointTorques", 1, &lwr_action_server::setJointStiffness, this);
        sub_setCartDamping_ = nh_.subscribe("/lwr_setCartDamping", 1, &lwr_action_server::setCartDamping, this);
        sub_setCartStiffness_ = nh_.subscribe("/lwr_setCartStiffness", 1, &lwr_action_server::setCartStiffness, this);
        sub_setCartForcesAndTorques_ =
            nh_.subscribe("/lwr_setCartForcesAndTorques", 1, &lwr_action_server::setCartForcesAndTorques, this);
        sub_setCancelOnForceAtTCP_ =
            nh_.subscribe("/lwr_setCancelOnForceAtTCP", 1, &lwr_action_server::setCancelOnForceAtTCP, this);
        sub_cancelLWRTrajectory_ =
            nh_.subscribe("/lwr_setCancelLWRTrajectory", 1, &lwr_action_server::setCancelLWRTrajectory, this);
        sub_setJointPosition_ = nh_.subscribe("/lwr_setJointPosition", 1, &lwr_action_server::setJointPosition, this);
        sub_setCartPose_ = nh_.subscribe("/lwr_setCartPose", 1, &lwr_action_server::setCartPose, this);

        // TODO check
        ser_fk_ = nh_.serviceClient<moveit_msgs::GetPositionFK>("/compute_fk");

        goal_active_ = false;
        cancel_goal_ = false;
        seq_counterFJTF_ = 0;
        seqCounterJointStates_ = 0;
        seqCounterFK_ = 0;
        setCancelOnForceAtTCP_ = false;

        memset(commandedJointPosition_, 0x0, LBR_MNJ * sizeof(float));
        memset(measuredJointValuesInRad_, 0x0, LBR_MNJ * sizeof(float));
        memset(measuredJointTorques_, 0x0, LBR_MNJ * sizeof(float));
        memset(estimatedExternalJointTorquesInNm_, 0x0, LBR_MNJ * sizeof(float));
        memset(estimatedExternalCartForcesAndTorques_, 0x0, NUMBER_OF_FRAME_ELEMENTS * sizeof(float));
        memset(driveTemperatues_, 0x0, LBR_MNJ * sizeof(float));

        memset(commandedCartForcesAndTorques_, 0x0, NUMBER_OF_CART_DOFS * sizeof(float));
        memset(commandedCartStiffness_, 0x0, NUMBER_OF_CART_DOFS * sizeof(float));
        memset(commandedCartDamping_, 0x0, NUMBER_OF_CART_DOFS * sizeof(float));
        memset(estimatedExternalCartForcesAndTorques_, 0x0, NUMBER_OF_CART_DOFS * sizeof(float));
        memset(commandedPose_, 0x0, NUMBER_OF_CART_DOFS * sizeof(float));
        memset(measuredPose_, 0x0, NUMBER_OF_CART_DOFS * sizeof(float));

        // FRI
        FRI_ = new FastResearchInterface("/homeL/8liebrec/TAMS/FRILibrary/etc/980039-FRI-Driver.init");
        // FRI_ = new FastResearchInterface("/home/user/ros_workspace/FRILibrary/etc/980039-FRI-Driver.init");
        setFRIMode(FastResearchInterface::JOINT_POSITION_CONTROL);
        // init position and timers for velocity
        FRI_->GetMeasuredJointPositions(measuredJointValuesInRad_);
        for (unsigned int j = 0; j < LBR_MNJ; j++)
        {
            Position_old_[j] = measuredJointValuesInRad_[j];
        }
        time_old_ = ros::Time::now();
        FRI_->WaitForKRCTick();
        FRI_->GetMeasuredJointPositions(measuredJointValuesInRad_);
        for (unsigned int j = 0; j < LBR_MNJ; j++)
        {
            Position_new_[j] = measuredJointValuesInRad_[j];
        }
        time_new_ = ros::Time::now();
        time_delta_.sec = time_new_.sec - time_old_.sec;
        time_delta_.nsec = time_new_.nsec - time_old_.nsec;

        // RML
        CycleTime_ = 0.01;
        RML_ = new TypeIRML(LBR_MNJ, CycleTime_);
        IP_ = new TypeIRMLInputParameters(LBR_MNJ);
        OP_ = new TypeIRMLOutputParameters(LBR_MNJ);

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

        // TODO check cart impedance mode
        for (unsigned int i = 0; i < NUMBER_OF_CART_DOFS; i++)
        {
            commandedCartStiffness_[i] = (float)200.0;
            commandedCartDamping_[i] = (float)0.7;
            commandedCartForcesAndTorques_[i] = (float)0.0;
        }
        FRI_->SetCommandedCartStiffness(commandedCartStiffness_);
        FRI_->SetCommandedCartDamping(commandedCartDamping_);
        FRI_->SetCommandedCartForcesAndTorques(commandedCartForcesAndTorques_);
    }

    ~lwr_action_server()
    {
        FRI_->StopRobot();
        delete FRI_;
    }

    void goalCB(GoalHandle gh)
    {
        if (!goal_active_)
        {
            // TODO check if order and joint names are correct else gh.setRejected();
            gh.setAccepted();
            agh_ = gh;
            goal_active_ = true;
        }
        else
        {
            gh.setRejected();
        }
    }

    void executeJointPositionTrajectory()
    {
        if (goal_active_)
        {
            // helper variables
            float remember = estimatedExternalCartForcesAndTorques_[1];
            float forceThreshold = 1.0;

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
                    commandedJointPosition_[j] = g->trajectory.points[i].positions[j];
                }
                FRI_->SetCommandedJointPositions(commandedJointPosition_);

                publishJointStates();

                time_from_start = (ros::Time::now() - time_start);
                publishFeedback(g, i, time_from_start);

                mc_.data = (i + 1) * 100 / g->trajectory.points.size();
                pub_motionCompletion_.publish(mc_);

                publishLWRStates();

                // check cancel request
                if (cancel_goal_)
                {
                    ROS_INFO("lwr_action_server: Goal canceled\n");
                    agh_.setCanceled();
                    cancel_goal_ = false;
                    stopLwr(g);
                    return;
                }

                /*  			//check measured and commanded JointValues and stop motion if necessary
                        if (i > 0)
                        {
                          for (unsigned int j = 0; j < LBR_MNJ; j++)
                          {
                          //	ROS_INFO("Don't touch me )) -- %f", fabs(commandedJointPosition_[j] -
                   measuredJointValuesInRad_[j])); if (fabs(commandedJointPosition_[j] - measuredJointValuesInRad_[j]) >
                   0.02)
                            {
                              ROS_INFO("lwr_action_server ERROR: The differenz between measured and commanded
                   jointvalues is higher then threshold!");
                              //cancel Goalhandle
                                goal_active_ = false;
                              stopLwr(g);
                              agh_.setAborted();
                              return;
                            }
                          }
                        }
                 */
                // cancel trajectory if force is exerted at the y-axis
                if (setCancelOnForceAtTCP_)
                {
                    ROS_INFO("CancelOnForceAtTCP!%f\n", fabs(estimatedExternalCartForcesAndTorques_[1] - remember));

                    if (fabs(estimatedExternalCartForcesAndTorques_[1] - remember) > forceThreshold)
                    {
                        ROS_INFO("lwr_action_server INFO: CancelOnForceAtTCP was activated. \n");
                        // cancel Goalhandle
                        goal_active_ = false;
                        stopLwr(g);
                        agh_.setAborted();
                        setCancelOnForceAtTCP_ = false;
                        return;
                    }
                }
                remember = estimatedExternalCartForcesAndTorques_[1];
            }
            agh_.setSucceeded();
            goal_active_ = false;
        }
    }

    void executeJointImpedanceTrajectory()
    {
        if (goal_active_)
        {
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
                    commandedJointPosition_[j] = g->trajectory.points[i].positions[j];
                }
                FRI_->SetCommandedJointPositions(commandedJointPosition_);

                FRI_->SetCommandedJointStiffness(commandedStiffness_);
                FRI_->SetCommandedJointDamping(commandedDamping_);
                FRI_->SetCommandedJointTorques(commandedTorquesInNm_);

                publishJointStates();

                time_from_start = (ros::Time::now() - time_start);
                publishFeedback(g, i, time_from_start);

                mc_.data = (i + 1) * 100 / g->trajectory.points.size();
                pub_motionCompletion_.publish(mc_);

                publishLWRStates();

                // check cancel request
                if (cancel_goal_)
                {
                    ROS_INFO("lwr_action_server: Goal canceled\n");
                    agh_.setCanceled();
                    cancel_goal_ = false;
                    stopLwr(g);
                    return;
                }
            }
            agh_.setSucceeded();
            goal_active_ = false;
        }
    }

    void executeCartImpedanceTrajectory()
    {
        if (goal_active_)
        {
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

                // calculate FK
                moveit_msgs::GetPositionFKRequest request;
                request.header.seq = seqCounterFK_;
                seqCounterFK_++;
                request.header.stamp = ros::Time::now();
                request.header.frame_id = g->trajectory.header.frame_id;

                std::string jointNames[] = { "lwr_arm_0_joint", "lwr_arm_1_joint", "lwr_arm_2_joint", "lwr_arm_3_joint",
                                             "lwr_arm_4_joint", "lwr_arm_5_joint", "lwr_arm_6_joint" };

                for (int j = 0; j < LBR_MNJ; j++)
                {
                    request.fk_link_names.push_back("lwr_arm_7_link");
                    request.robot_state.joint_state.name.push_back(jointNames[j]);
                    request.robot_state.joint_state.position.push_back(g->trajectory.points[i].positions[j]);
                }

                moveit_msgs::GetPositionFKResponse response;
                ser_fk_.call(request, response);

                ROS_INFO("TEST service called ) \n");

                if (response.error_code.val == moveit_msgs::MoveItErrorCodes::SUCCESS)
                {
                    std::cerr << "Response: " << response.pose_stamped[0] << std::endl;
                }
                else
                {
                    ROS_ERROR_STREAM("Failed to solve FK: " << response.error_code.val);
                }

                ROS_INFO("position_x %f\n", response.pose_stamped[0].pose.position.x);
                ROS_INFO("position_y %f\n", response.pose_stamped[0].pose.position.y);
                ROS_INFO("position_z %f\n", response.pose_stamped[0].pose.position.z);
                ROS_INFO("orientation_x %f\n", response.pose_stamped[0].pose.orientation.x);
                ROS_INFO("orientation_y %f\n", response.pose_stamped[0].pose.orientation.y);
                ROS_INFO("orientation_z %f\n", response.pose_stamped[0].pose.orientation.z);
                ROS_INFO("orientation_w %f\n", response.pose_stamped[0].pose.orientation.w);

                // TODO check
                // frame transformation from /world to /world_lwr_arm_base_join or calib_lwr_arm_base_link?

                geometry_msgs::PoseStamped pose_stamped_lwr;
                tf::TransformListener listener(ros::Duration(10));
                try
                {
                    listener.waitForTransform("/world_lwr_arm_base_joint", response.pose_stamped[0].header.frame_id,
                                              ros::Time(0), ros::Duration(10.0));
                    listener.transformPose("/world_lwr_arm_base_joint", response.pose_stamped[0], pose_stamped_lwr);
                }
                catch (tf::TransformException ex)
                {
                    ROS_ERROR("lwr_action_server: TransformListener ERROR %s", ex.what());
                }

                ROS_INFO("lwr_position_x %f\n", pose_stamped_lwr.pose.position.x);
                ROS_INFO("lwr_position_y %f\n", pose_stamped_lwr.pose.position.y);
                ROS_INFO("lwr_position_z %f\n", pose_stamped_lwr.pose.position.z);
                ROS_INFO("lwr_orientation_x %f\n", pose_stamped_lwr.pose.orientation.x);
                ROS_INFO("lwr_orientation_y %f\n", pose_stamped_lwr.pose.orientation.y);
                ROS_INFO("lwr_orientation_z %f\n", pose_stamped_lwr.pose.orientation.z);
                ROS_INFO("lwr_orientation_w %f\n", pose_stamped_lwr.pose.orientation.w);

                Eigen::Isometry3d end_effector_state;
                // tf::poseMsgToEigen(response.pose_stamped[0].pose,end_effector_state);
                tf::poseMsgToEigen(pose_stamped_lwr.pose, end_effector_state);

                const double* pdata = end_effector_state.matrix().data();
                ROS_INFO("*(pdata+0) %f\n", *(pdata + 0));
                ROS_INFO("*(pdata+1) %f\n", *(pdata + 1));
                ROS_INFO("*(pdata+2) %f\n", *(pdata + 2));

                ROS_INFO("*(pdata+4) %f\n", pdata[4]);
                ROS_INFO("*(pdata+5) %f\n", pdata[5]);
                ROS_INFO("*(pdata+6) %f\n", pdata[6]);

                ROS_INFO("*(pdata+8) %f\n", pdata[8]);
                ROS_INFO("*(pdata+9) %f\n", pdata[9]);
                ROS_INFO("*(pdata+10) %f\n", pdata[10]);

                ROS_INFO("*(pdata+12) %f\n", pdata[12]);
                ROS_INFO("*(pdata+13) %f\n", pdata[13]);
                ROS_INFO("*(pdata+14) %f\n", pdata[14]);

                ROS_INFO("*(pdata+3) %f\n", *(pdata + 3));
                ROS_INFO("*(pdata+7) %f\n", *(pdata + 7));
                ROS_INFO("*(pdata+11) %f\n", *(pdata + 11));
                ROS_INFO("*(pdata+15) %f\n", *(pdata + 15));

                float measuredCartPose[NUMBER_OF_FRAME_ELEMENTS];
                FRI_->GetMeasuredCartPose(measuredCartPose);

                for (unsigned int j = 0; j < NUMBER_OF_FRAME_ELEMENTS; j++)
                {
                    ROS_INFO("measuredCartPose[ %d ]  %f", j, measuredCartPose[j]);
                }
                // float commandedCartPose[NUMBER_OF_FRAME_ELEMENTS];
                commandedPose_[0] = pdata[0];
                commandedPose_[4] = pdata[1];
                commandedPose_[8] = pdata[2];

                commandedPose_[1] = pdata[4];
                commandedPose_[5] = pdata[5];
                commandedPose_[9] = pdata[6];

                commandedPose_[2] = pdata[8];
                commandedPose_[6] = pdata[9];
                commandedPose_[10] = pdata[10];

                commandedPose_[3] = pdata[12];
                commandedPose_[7] = pdata[13];
                commandedPose_[11] = pdata[14];
                // URDF specific changes <origin rpy="0 0 3.1415" xyz="0 0 0.804"/>	//TODO hopefully unnecessary

                // now send data to robot
                FRI_->SetCommandedCartStiffness(commandedCartStiffness_);
                FRI_->SetCommandedCartDamping(commandedCartDamping_);
                FRI_->SetCommandedCartForcesAndTorques(commandedCartForcesAndTorques_);
                FRI_->SetCommandedCartPose(commandedPose_);

                publishJointStates();

                time_from_start = (ros::Time::now() - time_start);
                publishFeedback(g, i, time_from_start);

                mc_.data = (i + 1) * 100 / g->trajectory.points.size();
                pub_motionCompletion_.publish(mc_);

                publishLWRStates();

                // check cancel request
                if (cancel_goal_)
                {
                    ROS_INFO("lwr_action_server: Goal canceled\n");
                    agh_.setCanceled();
                    cancel_goal_ = false;
                    stopLwr(g);
                    return;
                }
            }
            agh_.setSucceeded();
            goal_active_ = false;
        }
    }

    void cancelCB(GoalHandle gh)
    {
        // cancel FRI
        cancel_goal_ = true;
        // cancel Goalhandle
        goal_active_ = false;
    }

    void stopLwr(Goal g)
    {
        // load first position
        FRI_->GetMeasuredJointPositions(measuredJointValuesInRad_);
        for (unsigned int j = 0; j < LBR_MNJ; j++)
        {
            IP_->CurrentPosition->VecData[j] = (double)DEG(measuredJointValuesInRad_[j]);
            IP_->CurrentVelocity->VecData[j] = (double)DEG(velocity_new_[j]);
            IP_->TargetVelocity->VecData[j] = (double)0.0;
            IP_->MaxAcceleration->VecData[j] = (double)DEG(0.2);
            IP_->SelectionVector->VecData[j] = true;
        }

        // execute Trajectory
        int ResultValue = TypeIRML::RML_WORKING;

        // TODO teste punkte weglassen.....
        for (unsigned int j = 0; j < 1; j++)
        {
            FRI_->WaitForKRCTick();
            ResultValue = RML_->GetNextMotionState_Velocity(*IP_, OP_);

            if ((ResultValue != TypeIRML::RML_WORKING) && (ResultValue != TypeIRML::RML_FINAL_STATE_REACHED))
            {
                ROS_INFO("lwr_action_server: ERROR during trajectory generation (%d).", ResultValue);
            }

            for (unsigned int j = 0; j < LBR_MNJ; j++)
            {
                commandedJointPosition_[j] = RAD((double)(OP_->NewPosition->VecData[j]));
            }
            FRI_->SetCommandedJointPositions(commandedJointPosition_);

            // prepare for next iteration
            *(IP_->CurrentPosition) = *(OP_->NewPosition);
            *(IP_->CurrentVelocity) = *(OP_->NewVelocity);
        }
        //----------------------------------------------

        // target position-based trajectory generation.
        while ((FRI_->IsMachineOK()) && (ResultValue != TypeIRML::RML_FINAL_STATE_REACHED))
        {
            FRI_->WaitForKRCTick();
            ResultValue = RML_->GetNextMotionState_Velocity(*IP_, OP_);

            if ((ResultValue != TypeIRML::RML_WORKING) && (ResultValue != TypeIRML::RML_FINAL_STATE_REACHED))
            {
                ROS_INFO("lwr_action_server: ERROR during trajectory generation (%d).", ResultValue);
            }

            for (unsigned int j = 0; j < LBR_MNJ; j++)
            {
                commandedJointPosition_[j] = RAD((double)(OP_->NewPosition->VecData[j]));
            }
            FRI_->SetCommandedJointPositions(commandedJointPosition_);

            // prepare for next iteration
            *(IP_->CurrentPosition) = *(OP_->NewPosition);
            *(IP_->CurrentVelocity) = *(OP_->NewVelocity);
        }
    }

    void publishFeedback(Goal g, unsigned int counter, ros::Duration time_from_start)
    {
        FRI_->GetMeasuredJointPositions(measuredJointValuesInRad_);
        FRI_->GetMeasuredJointTorques(measuredJointTorques_);
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
            actual.positions.push_back((double)measuredJointValuesInRad_[j]);
            desired.positions.push_back(g->trajectory.points[counter].positions[j]);
            error.positions.push_back(fabs(g->trajectory.points[counter].positions[j]) -
                                      fabs(measuredJointValuesInRad_[j]));

            actual.velocities.push_back(velocity_new_[j]);
            desired.velocities.push_back(g->trajectory.points[counter].velocities[j]);
            error.velocities.push_back(fabs(g->trajectory.points[counter].velocities[j]) - fabs(velocity_new_[j]));

            actual.accelerations.push_back(acceleration_[j]);
            desired.accelerations.push_back(g->trajectory.points[counter].accelerations[j]);
            error.accelerations.push_back(fabs(g->trajectory.points[counter].accelerations[j]) - fabs(acceleration_[j]));

            actual.effort.push_back(measuredJointTorques_[j]);
            desired.effort.push_back(NAN);
            error.effort.push_back(NAN);

            actual.time_from_start = time_from_start;
            desired.time_from_start = g->trajectory.points[counter].time_from_start;
            error.time_from_start = g->trajectory.points[counter].time_from_start - time_from_start;
        }

        // fill control_msgs/FollowJointTrajectoryFeedback
        control_msgs::FollowJointTrajectoryFeedback fjtf;
        fjtf.header.seq = seq_counterFJTF_;
        fjtf.header.stamp = ros::Time::now();
        fjtf.header.frame_id = g->trajectory.header.frame_id;
        fjtf.joint_names = g->trajectory.joint_names;
        fjtf.actual = actual;
        fjtf.desired = desired;
        fjtf.error = error;
        agh_.publishFeedback(fjtf);
        seq_counterFJTF_++;
    }

    void publishJointStates()
    {
        pthread_mutex_lock(&mutex);
        std::string names[] = { "lwr_arm_0_joint", "lwr_arm_1_joint", "lwr_arm_2_joint", "lwr_arm_3_joint",
                                "lwr_arm_4_joint", "lwr_arm_5_joint", "lwr_arm_6_joint" };

        FRI_->GetMeasuredJointPositions(measuredJointValuesInRad_);
        FRI_->GetMeasuredJointTorques(measuredJointTorques_);

        time_old_ = time_new_;
        time_new_ = ros::Time::now();

        for (unsigned int j = 0; j < LBR_MNJ; j++)
        {
            Position_old_[j] = Position_new_[j];
            Position_new_[j] = measuredJointValuesInRad_[j];
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
            js.position.push_back(measuredJointValuesInRad_[i]);
            js.velocity.push_back(velocity_new_[i]);
            js.effort.push_back(measuredJointTorques_[i]);
        }

        pub_joints_.publish(js);
        seqCounterJointStates_++;
        pthread_mutex_unlock(&mutex);
    }

    void publishLWRStates()
    {
        FRI_->GetEstimatedExternalJointTorques(estimatedExternalJointTorquesInNm_);
        FRI_->GetEstimatedExternalCartForcesAndTorques(estimatedExternalCartForcesAndTorques_);
        FRI_->GetDriveTemperatures(driveTemperatues_);

        std_msgs::Float32MultiArray DriveTemperatues;
        std_msgs::Float32MultiArray EstimatedExternalJointTorquesInNm;
        std_msgs::Float32MultiArray EstimatedExternalCartForcesAndTorques;

        for (unsigned int j = 0; j < LBR_MNJ; j++)
        {
            DriveTemperatues.data.push_back(driveTemperatues_[j]);
            EstimatedExternalJointTorquesInNm.data.push_back(estimatedExternalJointTorquesInNm_[j]);
        }

        for (unsigned int j = 0; j < NUMBER_OF_FRAME_ELEMENTS; j++)
        {
            EstimatedExternalCartForcesAndTorques.data.push_back(estimatedExternalCartForcesAndTorques_[j]);
        }

        pub_estimatedExternalCartForcesAndTorques_.publish(EstimatedExternalCartForcesAndTorques);
        pub_driveTemperatues_.publish(DriveTemperatues);
        pub_estimatedExternalJointTorquesInNm_.publish(EstimatedExternalJointTorquesInNm);

        // TODO//onyl if cart impedance mode
        if (FRIMode_ == FastResearchInterface::CART_IMPEDANCE_CONTROL)
        {
            FRI_->GetMeasuredCartPose(measuredPose_);
            std_msgs::Float32MultiArray measuredPose;

            for (unsigned int j = 0; j < NUMBER_OF_FRAME_ELEMENTS; j++)
            {
                measuredPose.data.push_back(measuredPose_[j]);
            }
            pub_measuredPose_.publish(measuredPose);
        }
    }

    void setFRIMode(unsigned int FRImode)
    {
        int ResultValue = 0;
        ResultValue = FRI_->StartRobot(FRImode);
        FRIMode_ = FRImode;

        if ((ResultValue != EOK) && (ResultValue != EALREADY))
        {
            ROS_INFO("lwr_action_server: An error occurred during starting up the robot...\n");
            exit(EXIT_FAILURE);
        }
    }

    unsigned int getFRIMode()
    {
        return FRIMode_;
    }

    void setFRIModeSubscriber(const std_msgs::UInt32::ConstPtr& msg)
    {
        setFRIMode(msg->data);
    }

    void setJointPosition(const std_msgs::Float32MultiArray::ConstPtr& msg)
    {
        if ((FRIMode_ == FastResearchInterface::JOINT_POSITION_CONTROL) or
            (FRIMode_ == FastResearchInterface::JOINT_IMPEDANCE_CONTROL))
        {
            for (int i = 0; i < LBR_MNJ; i++)
            {
                commandedJointPosition_[i] = msg->data[i];
            }
            FRI_->SetCommandedJointTorques(commandedJointPosition_);
        }
    }

    void setJointDamping(const std_msgs::Float32MultiArray::ConstPtr& msg)
    {
        if (FRIMode_ == FastResearchInterface::JOINT_IMPEDANCE_CONTROL)
        {
            for (int i = 0; i < LBR_MNJ; i++)
            {
                commandedDamping_[i] = msg->data[i];
            }
            FRI_->SetCommandedJointDamping(commandedDamping_);
        }
    }

    void setJointStiffness(const std_msgs::Float32MultiArray::ConstPtr& msg)
    {
        if (FRIMode_ == FastResearchInterface::JOINT_IMPEDANCE_CONTROL)
        {
            for (int i = 0; i < LBR_MNJ; i++)
            {
                commandedStiffness_[i] = msg->data[i];
            }
            FRI_->SetCommandedJointStiffness(commandedStiffness_);
        }
    }

    void setJointTorques(const std_msgs::Float32MultiArray::ConstPtr& msg)
    {
        if ((FRIMode_ == FastResearchInterface::JOINT_IMPEDANCE_CONTROL) or
            (FRIMode_ == FastResearchInterface::JOINT_TORQUE_CONTROL))
        {
            for (int i = 0; i < LBR_MNJ; i++)
            {
                commandedTorquesInNm_[i] = msg->data[i];
            }
            FRI_->SetCommandedJointTorques(commandedTorquesInNm_);
        }
    }

    void setCartPose(const std_msgs::Float32MultiArray::ConstPtr& msg)
    {
        if (FRIMode_ == FastResearchInterface::CART_IMPEDANCE_CONTROL)
        {
            for (int i = 0; i < LBR_MNJ; i++)
            {
                commandedPose_[i] = msg->data[i];
            }
            FRI_->SetCommandedCartPose(commandedPose_);
        }
    }

    void setCartDamping(const std_msgs::Float32MultiArray::ConstPtr& msg)
    {
        if (FRIMode_ == FastResearchInterface::CART_IMPEDANCE_CONTROL)
        {
            for (int i = 0; i < NUMBER_OF_CART_DOFS; i++)
            {
                commandedCartDamping_[i] = msg->data[i];
            }
            FRI_->SetCommandedCartDamping(commandedCartDamping_);
        }
    }

    void setCartStiffness(const std_msgs::Float32MultiArray::ConstPtr& msg)
    {
        if (FRIMode_ == FastResearchInterface::CART_IMPEDANCE_CONTROL)
        {
            for (int i = 0; i < NUMBER_OF_CART_DOFS; i++)
            {
                commandedCartStiffness_[i] = msg->data[i];
            }
            FRI_->SetCommandedCartStiffness(commandedCartStiffness_);
        }
    }

    void setCartForcesAndTorques(const std_msgs::Float32MultiArray::ConstPtr& msg)
    {
        if (FRIMode_ == FastResearchInterface::CART_IMPEDANCE_CONTROL)
        {
            for (int i = 0; i < NUMBER_OF_CART_DOFS; i++)
            {
                commandedCartForcesAndTorques_[i] = msg->data[i];
            }
            FRI_->SetCommandedCartForcesAndTorques(commandedCartForcesAndTorques_);
        }
    }

    void setCancelOnForceAtTCP(const std_msgs::Bool::ConstPtr& msg)
    {
        setCancelOnForceAtTCP_ = msg->data;
    }

    void setCancelLWRTrajectory(const std_msgs::Bool::ConstPtr& msg)
    {
        if (msg->data == true)
        {
            // cancel FRI
            cancel_goal_ = true;
            // cancel Goalhandle
            goal_active_ = false;
        }
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "lwr_action_server");
    lwr_action_server lwr_action_server(ros::this_node::getName());
    ros::AsyncSpinner spinner(1);
    spinner.start();
    ros::Rate loop_rate(100);  // 100Hz

    while (ros::ok())
    {
        lwr_action_server.publishJointStates();
        lwr_action_server.publishLWRStates();

        switch (lwr_action_server.getFRIMode())
        {
            case FastResearchInterface::JOINT_POSITION_CONTROL:
                lwr_action_server.executeJointPositionTrajectory();
                break;

            case FastResearchInterface::JOINT_IMPEDANCE_CONTROL:
                lwr_action_server.executeJointImpedanceTrajectory();
                break;

            case FastResearchInterface::CART_IMPEDANCE_CONTROL:
                lwr_action_server.executeCartImpedanceTrajectory();
                break;

            default:
                ROS_INFO("lwr_action_server ERROR: Not supported FRIMODE! Select JOINT_POSITION_CONTROL, "
                         "JOINT_IMPEDANCE_CONTROL or CART_IMPEDANCE_CONTROL \n");
                return -1;
        }

        ros::spinOnce();
        loop_rate.sleep();
    }
}

// rostopic pub -1 /lwr_setFRIMode std_msgs/UInt32 "data: 10"
