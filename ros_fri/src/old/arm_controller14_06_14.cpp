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

    float commandedJointValuesInRad_[LBR_MNJ];
    float measuredJointValuesInRad_[LBR_MNJ];
    float TorqueValuesInNm_[LBR_MNJ];
    float EstimatedExternalJointTorquesInNm_[LBR_MNJ];

    float EstimatedExternalCartForcesAndTorques_[NUMBER_OF_FRAME_ELEMENTS];
    float DriveTemperatues_[LBR_MNJ];
    bool setCancelOnForceAtTCP_;

    float commandedTorquesInNm_[LBR_MNJ], commandedStiffness_[LBR_MNJ], commandedDamping_[LBR_MNJ];

    // Msg
    std_msgs::UInt32 mc_;

    ros::Publisher pub_joints_;
    ros::Publisher pub_motionCompletion_;
    ros::Publisher pub_EstimatedExternalCartForcesAndTorques_;
    ros::Publisher pub_DriveTemperatues_;
    ros::Publisher pub_EstimatedExternalJointTorquesInNm_;

    ros::Subscriber sub_setFRIMode_;
    ros::Subscriber sub_setJointDamping_;
    ros::Subscriber sub_setJointStiffness_;
    ros::Subscriber sub_setJointTorques_;
    ros::Subscriber sub_setCartDamping_;
    ros::Subscriber sub_setCartStiffness_;
    ros::Subscriber sub_setCartForcesAndTorques_;
    ros::Subscriber sub_setCancelOnForceAtTCP_;
    ros::Subscriber sub_cancelLWRTrajectory_;

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

        pub_EstimatedExternalCartForcesAndTorques_ =
            nh_.advertise<std_msgs::Float32MultiArray>("/lwr_EstimatedExternalCartForcesAndTorques", 1);
        pub_DriveTemperatues_ = nh_.advertise<std_msgs::Float32MultiArray>("/lwr_DriveTemperatues", 1);
        pub_EstimatedExternalJointTorquesInNm_ =
            nh_.advertise<std_msgs::Float32MultiArray>("/lwr_EstimatedExternalJointTorquesInNm", 1);

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

        // TODO check
        ser_fk_ = nh_.serviceClient<moveit_msgs::GetPositionFK>("/compute_fk");

        goal_active_ = false;
        cancel_goal_ = false;
        seq_counterFJTF_ = 0;
        seqCounterJointStates_ = 0;
        seqCounterFK_ = 0;
        setCancelOnForceAtTCP_ = false;

        memset(commandedJointValuesInRad_, 0x0, LBR_MNJ * sizeof(float));
        memset(measuredJointValuesInRad_, 0x0, LBR_MNJ * sizeof(float));
        memset(TorqueValuesInNm_, 0x0, LBR_MNJ * sizeof(float));
        memset(EstimatedExternalJointTorquesInNm_, 0x0, LBR_MNJ * sizeof(float));
        memset(EstimatedExternalCartForcesAndTorques_, 0x0, NUMBER_OF_FRAME_ELEMENTS * sizeof(float));
        memset(DriveTemperatues_, 0x0, LBR_MNJ * sizeof(float));

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

        // TODO try stiffness mode
        for (unsigned int i = 0; i < LBR_MNJ; i++)
        {
            commandedStiffness_[i] = (float)200.0;
            commandedDamping_[i] = (float)0.7;
            commandedTorquesInNm_[i] = (float)0.0;
        }
        // nothing must happen...
        FRI_->SetCommandedJointStiffness(commandedStiffness_);
        FRI_->SetCommandedJointDamping(commandedDamping_);
        FRI_->SetCommandedJointTorques(commandedTorquesInNm_);

        // test
        /*sensor_msgs::JointState joint_values;
        joint_values.position.push_back(-0.17);
        joint_values.position.push_back(90.21 - 90.0);
        joint_values.position.push_back(0.36);
        joint_values.position.push_back(0.31);
        joint_values.position.push_back(0.03);
        joint_values.position.push_back(-0.31);
        joint_values.position.push_back(-0.34);
         calculate_FK(joint_values);
    */
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
            float remember = EstimatedExternalCartForcesAndTorques_[1];
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
                    commandedJointValuesInRad_[j] = g->trajectory.points[i].positions[j];
                }
                FRI_->SetCommandedJointPositions(commandedJointValuesInRad_);

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

                // check measured and commanded JointValues and stop motion if necessary
                if (i > 0)
                {
                    for (unsigned int j = 0; j < LBR_MNJ; j++)
                    {
                        //	ROS_INFO("Don't touch me )) -- %f", fabs(commandedJointValuesInRad_[j] - measuredJointValuesInRad_[j]));
                        if (fabs(commandedJointValuesInRad_[j] - measuredJointValuesInRad_[j]) > 0.02)
                        {
                            ROS_INFO("lwr_action_server ERROR: The differenz between measured and commanded "
                                     "jointvalues is higher "
                                     "then threshold!");
                            // cancel Goalhandle
                            goal_active_ = false;
                            stopLwr(g);
                            agh_.setAborted();
                            return;
                        }
                    }
                }

                // cancel trajectory if force is exerted at the y-axis
                if (setCancelOnForceAtTCP_ == true)
                {
                    ROS_INFO("CancelOnForceAtTCP!%f\n", fabs(EstimatedExternalCartForcesAndTorques_[1] - remember));

                    if (fabs(EstimatedExternalCartForcesAndTorques_[1] - remember) > forceThreshold)
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
                remember = EstimatedExternalCartForcesAndTorques_[1];
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
                    commandedJointValuesInRad_[j] = g->trajectory.points[i].positions[j];
                }
                FRI_->SetCommandedJointPositions(commandedJointValuesInRad_);
                // need????
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
                //		std::string linkNames[] = {"lwr_arm_1_link", "lwr_arm_2_link", "lwr_arm_3_link",
                //"lwr_arm_4_link", "lwr_arm_5_link", "lwr_arm_6_link", "lwr_arm_7_link"};

                for (int j = 0; j < LBR_MNJ; j++)
                {
                    //				request.fk_link_names.push_back(linkNames[j]);	//TODO Link names!!!!!
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

                Eigen::Isometry3d end_effector_state;
                tf::poseMsgToEigen(response.pose_stamped[0].pose, end_effector_state);

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

                for (unsigned int j = 0; j < LBR_MNJ; j++)
                {
                    commandedJointValuesInRad_[j] = g->trajectory.points[i].positions[j];
                }
                FRI_->SetCommandedJointPositions(commandedJointValuesInRad_);
                // need????
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
                commandedJointValuesInRad_[j] = RAD((double)(OP_->NewPosition->VecData[j]));
            }
            FRI_->SetCommandedJointPositions(commandedJointValuesInRad_);

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
                commandedJointValuesInRad_[j] = RAD((double)(OP_->NewPosition->VecData[j]));
            }
            FRI_->SetCommandedJointPositions(commandedJointValuesInRad_);

            // prepare for next iteration
            *(IP_->CurrentPosition) = *(OP_->NewPosition);
            *(IP_->CurrentVelocity) = *(OP_->NewVelocity);
        }
    }

    void publishFeedback(Goal g, unsigned int counter, ros::Duration time_from_start)
    {
        FRI_->GetMeasuredJointPositions(measuredJointValuesInRad_);
        FRI_->GetMeasuredJointTorques(TorqueValuesInNm_);
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

            actual.effort.push_back(TorqueValuesInNm_[j]);
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
        FRI_->GetMeasuredJointTorques(TorqueValuesInNm_);

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
            js.effort.push_back(TorqueValuesInNm_[i]);
        }

        pub_joints_.publish(js);
        seqCounterJointStates_++;
        pthread_mutex_unlock(&mutex);
    }

    void publishLWRStates()
    {
        FRI_->GetEstimatedExternalJointTorques(EstimatedExternalJointTorquesInNm_);
        FRI_->GetEstimatedExternalCartForcesAndTorques(EstimatedExternalCartForcesAndTorques_);
        FRI_->GetDriveTemperatures(DriveTemperatues_);

        std_msgs::Float32MultiArray DriveTemperatues;
        std_msgs::Float32MultiArray EstimatedExternalJointTorquesInNm;
        std_msgs::Float32MultiArray EstimatedExternalCartForcesAndTorques;

        for (unsigned int j = 0; j < LBR_MNJ; j++)
        {
            DriveTemperatues.data.push_back(DriveTemperatues_[j]);
            EstimatedExternalJointTorquesInNm.data.push_back(EstimatedExternalJointTorquesInNm_[j]);
        }

        for (unsigned int j = 0; j < NUMBER_OF_FRAME_ELEMENTS; j++)
        {
            EstimatedExternalCartForcesAndTorques.data.push_back(EstimatedExternalCartForcesAndTorques_[j]);
        }

        pub_EstimatedExternalCartForcesAndTorques_.publish(EstimatedExternalCartForcesAndTorques);
        pub_DriveTemperatues_.publish(DriveTemperatues);
        pub_EstimatedExternalJointTorquesInNm_.publish(EstimatedExternalJointTorquesInNm);
    }

    void calculate_FK(sensor_msgs::JointState joint_values)
    {
        robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
        robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
        ROS_INFO("Model frame: %s", kinematic_model->getModelFrame().c_str());
        moveit::core::RobotStatePtr kinematic_state(new robot_state::RobotState(kinematic_model));
        kinematic_state->setToDefaultValues();
        // const moveit::core::JointModelGroup* joint_model_group = kinematic_model->getJointModelGroup("arm");

        std::string names[] = { "lwr_arm_0_joint", "lwr_arm_1_joint", "lwr_arm_2_joint", "lwr_arm_3_joint",
                                "lwr_arm_4_joint", "lwr_arm_5_joint", "lwr_arm_6_joint" };
        for (int i = 0; i < LBR_MNJ; i++)
        {
            joint_values.name.push_back(names[i]);
        }

        kinematic_state->setVariableValues(joint_values);
        const Eigen::Isometry3d& end_effector_state = kinematic_state->getGlobalLinkTransform("lwr_arm_1_link");

        // only for test
        geometry_msgs::Pose pose;
        tf::poseEigenToMsg(end_effector_state, pose);
        ROS_INFO("position_x %f", pose.position.x);
        ROS_INFO("position_y %f", pose.position.y);
        ROS_INFO("position_z %f", pose.position.z);
        ROS_INFO("orientation_x %f", pose.orientation.x);
        ROS_INFO("orientation_y %f", pose.orientation.y);
        ROS_INFO("orientation_z %f", pose.orientation.z);
        ROS_INFO("orientation_w %f", pose.orientation.w);

        const Eigen::Isometry3d& end_effector_state1 = kinematic_state->getGlobalLinkTransform("lwr_arm_2_link");

        tf::poseEigenToMsg(end_effector_state1, pose);
        ROS_INFO("position_x %f", pose.position.x);
        ROS_INFO("position_y %f", pose.position.y);
        ROS_INFO("position_z %f", pose.position.z);
        ROS_INFO("orientation_x %f", pose.orientation.x);
        ROS_INFO("orientation_y %f", pose.orientation.y);
        ROS_INFO("orientation_z %f", pose.orientation.z);
        ROS_INFO("orientation_w %f", pose.orientation.w);
        const Eigen::Isometry3d& end_effector_state2 = kinematic_state->getGlobalLinkTransform("lwr_arm_3_link");

        tf::poseEigenToMsg(end_effector_state2, pose);
        ROS_INFO("position_x %f", pose.position.x);
        ROS_INFO("position_y %f", pose.position.y);
        ROS_INFO("position_z %f", pose.position.z);
        ROS_INFO("orientation_x %f", pose.orientation.x);
        ROS_INFO("orientation_y %f", pose.orientation.y);
        ROS_INFO("orientation_z %f", pose.orientation.z);
        ROS_INFO("orientation_w %f", pose.orientation.w);
        const Eigen::Isometry3d& end_effector_state3 = kinematic_state->getGlobalLinkTransform("lwr_arm_4_link");

        tf::poseEigenToMsg(end_effector_state3, pose);
        ROS_INFO("position_x %f", pose.position.x);
        ROS_INFO("position_y %f", pose.position.y);
        ROS_INFO("position_z %f", pose.position.z);
        ROS_INFO("orientation_x %f", pose.orientation.x);
        ROS_INFO("orientation_y %f", pose.orientation.y);
        ROS_INFO("orientation_z %f", pose.orientation.z);
        ROS_INFO("orientation_w %f", pose.orientation.w);
        const Eigen::Isometry3d& end_effector_state4 = kinematic_state->getGlobalLinkTransform("lwr_arm_5_link");

        tf::poseEigenToMsg(end_effector_state4, pose);
        ROS_INFO("position_x %f", pose.position.x);
        ROS_INFO("position_y %f", pose.position.y);
        ROS_INFO("position_z %f", pose.position.z);
        ROS_INFO("orientation_x %f", pose.orientation.x);
        ROS_INFO("orientation_y %f", pose.orientation.y);
        ROS_INFO("orientation_z %f", pose.orientation.z);
        ROS_INFO("orientation_w %f", pose.orientation.w);
        const Eigen::Isometry3d& end_effector_state5 = kinematic_state->getGlobalLinkTransform("lwr_arm_6_link");

        tf::poseEigenToMsg(end_effector_state5, pose);
        ROS_INFO("position_x %f", pose.position.x);
        ROS_INFO("position_y %f", pose.position.y);
        ROS_INFO("position_z %f", pose.position.z);
        ROS_INFO("orientation_x %f", pose.orientation.x);
        ROS_INFO("orientation_y %f", pose.orientation.y);
        ROS_INFO("orientation_z %f", pose.orientation.z);
        ROS_INFO("orientation_w %f", pose.orientation.w);
        const Eigen::Isometry3d& end_effector_state6 = kinematic_state->getGlobalLinkTransform("lwr_arm_7_link");

        tf::poseEigenToMsg(end_effector_state6, pose);
        ROS_INFO("position_x %f", pose.position.x);
        ROS_INFO("position_y %f", pose.position.y);
        ROS_INFO("position_z %f", pose.position.z);
        ROS_INFO("orientation_x %f", pose.orientation.x);
        ROS_INFO("orientation_y %f", pose.orientation.y);
        ROS_INFO("orientation_z %f", pose.orientation.z);
        ROS_INFO("orientation_w %f", pose.orientation.w);

        // const Eigen::Isometry3d &end_effector_state = kinematic_state->getGlobalLinkTransform("lwr_arm_7_link");

        // need? kuka 12 ....eigen 16... 	#include <Eigen/Geometry>

        const double* pdata = end_effector_state.matrix().data();
        ROS_INFO("*(pdata+0) %f", *(pdata + 0));
        ROS_INFO("*(pdata+1) %f", *(pdata + 1));
        ROS_INFO("*(pdata+2) %f", *(pdata + 2));

        ROS_INFO("*(pdata+4) %f", pdata[4]);
        ROS_INFO("*(pdata+5) %f", pdata[5]);
        ROS_INFO("*(pdata+6) %f", pdata[6]);

        ROS_INFO("*(pdata+8) %f", pdata[8]);
        ROS_INFO("*(pdata+9) %f", pdata[9]);
        ROS_INFO("*(pdata+10) %f", pdata[10]);

        ROS_INFO("*(pdata+12) %f", pdata[12]);
        ROS_INFO("*(pdata+13) %f", pdata[13]);
        ROS_INFO("*(pdata+14) %f", pdata[14]);

        ROS_INFO("*(pdata+3) %f", *(pdata + 3));
        ROS_INFO("*(pdata+7) %f", *(pdata + 7));
        ROS_INFO("*(pdata+11) %f", *(pdata + 11));
        ROS_INFO("*(pdata+15) %f", *(pdata + 15));
        /*
         //----------------------------------------------------------------
            const Eigen::Isometry3d &end_effector_state = kinematic_state->getGlobalLinkTransform("tcp");
          joint_values.position.push_back(-37.10);
          joint_values.position.push_back(137.45 - 90.0);
          joint_values.position.push_back(158.38);
          joint_values.position.push_back(49.51);
          joint_values.position.push_back(18.56);
          joint_values.position.push_back(-78.05);
          joint_values.position.push_back(-146.31);

        [ INFO] [1400852888.193309720]: position_x 0.000000
        [ INFO] [1400852888.193390303]: position_y 0.000000
        [ INFO] [1400852888.193410175]: position_z 0.914000
        [ INFO] [1400852888.193426104]: orientation_x -0.000000
        [ INFO] [1400852888.193445259]: orientation_y -0.000000
        [ INFO] [1400852888.193461638]: orientation_z -0.955481
        [ INFO] [1400852888.193479747]: orientation_w 0.295052

        [ INFO] [1400852888.193496795]: position_x 0.000000
        [ INFO] [1400852888.193514454]: position_y 0.000000
        [ INFO] [1400852888.193534931]: position_z 1.114000
        [ INFO] [1400852888.193550229]: orientation_x 0.700534
        [ INFO] [1400852888.193568101]: orientation_y 0.096189
        [ INFO] [1400852888.193583726]: orientation_z 0.315543
        [ INFO] [1400852888.193598842]: orientation_w 0.632798

        [ INFO] [1400852888.193614506]: position_x 0.052916
        [ INFO] [1400852888.193629400]: position_y 0.036126
        [ INFO] [1400852888.193644362]: position_z 0.924541
        [ INFO] [1400852888.193659956]: orientation_x -0.926685
        [ INFO] [1400852888.193678433]: orientation_y 0.338974
        [ INFO] [1400852888.193697243]: orientation_z -0.094467
        [ INFO] [1400852888.193717350]: orientation_w 0.132016

        [ INFO] [1400852888.193737535]: position_x 0.105833
        [ INFO] [1400852888.193756154]: position_y 0.072252
        [ INFO] [1400852888.193771868]: position_z 0.735082
        [ INFO] [1400852888.193789806]: orientation_x -0.586067
        [ INFO] [1400852888.193807953]: orientation_y -0.046543
        [ INFO] [1400852888.193825943]: orientation_z -0.560987
        [ INFO] [1400852888.193843906]: orientation_w 0.582797

        [ INFO] [1400852888.193863398]: position_x 0.247521
        [ INFO] [1400852888.193881741]: position_y 0.008979
        [ INFO] [1400852888.193899764]: position_z 0.608902
        [ INFO] [1400852888.193917989]: orientation_x 0.870346
        [ INFO] [1400852888.193936436]: orientation_y -0.240718
        [ INFO] [1400852888.193954739]: orientation_z 0.424761
        [ INFO] [1400852888.193970116]: orientation_w 0.064266
        [ INFO] [1400852888.193989332]: position_x 0.382124
        [ INFO] [1400852888.194008213]: position_y -0.051130
        [ INFO] [1400852888.194025680]: position_z 0.489032
        [ INFO] [1400852888.194043624]: orientation_x 0.594725
        [ INFO] [1400852888.194062109]: orientation_y 0.438880
        [ INFO] [1400852888.194080542]: orientation_z -0.609596
        [ INFO] [1400852888.194098114]: orientation_w 0.286494
        [ INFO] [1400852888.194116749]: position_x 0.314161
        [ INFO] [1400852888.194137426]: position_y -0.015982
        [ INFO] [1400852888.194155958]: position_z 0.504188
        [ INFO] [1400852888.194173725]: orientation_x -0.482568
        [ INFO] [1400852888.194191459]: orientation_y -0.412280
        [ INFO] [1400852888.194209341]: orientation_z 0.291292
        [ INFO] [1400852888.194227559]: orientation_w 0.715754
        */

        float measuredCartPose[NUMBER_OF_FRAME_ELEMENTS];
        FRI_->GetMeasuredCartPose(measuredCartPose);

        for (unsigned int j = 0; j < NUMBER_OF_FRAME_ELEMENTS; j++)
        {
            ROS_INFO("measuredCartPose[ %d ]  %f", j, measuredCartPose[j]);
        }

        /*
        pdata[0] = measuredCartPose[0];
        pdata[1] = measuredCartPose[4];
        pdata[2] = measuredCartPose[8];

        pdata[4] = measuredCartPose[1];
        pdata[5] = measuredCartPose[5];
        pdata[6] = measuredCartPose[9];

        pdata[8] = measuredCartPose[2];
        pdata[9] = measuredCartPose[6];
        pdata[10] = measuredCartPose[10];

        pdata[12] = measuredCartPose[3];
        pdata[13] = measuredCartPose[7];
        pdata[14] = measuredCartPose[11];
        */
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

    void setJointDamping(const std_msgs::Float32MultiArray::ConstPtr& msg)
    {
        if (FRIMode_ != FastResearchInterface::JOINT_POSITION_CONTROL)
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
        if (FRIMode_ != FastResearchInterface::JOINT_POSITION_CONTROL)
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
        if (FRIMode_ != FastResearchInterface::JOINT_POSITION_CONTROL)
        {
            for (int i = 0; i < LBR_MNJ; i++)
            {
                commandedTorquesInNm_[i] = msg->data[i];
            }
            FRI_->SetCommandedJointTorques(commandedTorquesInNm_);
        }
    }

    void setCartDamping(const std_msgs::Float32MultiArray::ConstPtr& msg)
    {
        if (FRIMode_ != FastResearchInterface::JOINT_POSITION_CONTROL)
        {
            float commandedDamping[NUMBER_OF_CART_DOFS];
            for (int i = 0; i < NUMBER_OF_CART_DOFS; i++)
            {
                commandedDamping[i] = msg->data[i];
            }
            FRI_->SetCommandedCartDamping(commandedDamping);
        }
    }

    void setCartStiffness(const std_msgs::Float32MultiArray::ConstPtr& msg)
    {
        if (FRIMode_ != FastResearchInterface::JOINT_POSITION_CONTROL)
        {
            float commandedStiffness[NUMBER_OF_CART_DOFS];
            for (int i = 0; i < NUMBER_OF_CART_DOFS; i++)
            {
                commandedStiffness[i] = msg->data[i];
            }
            FRI_->SetCommandedJointStiffness(commandedStiffness);
        }
    }

    void setCartForcesAndTorques(const std_msgs::Float32MultiArray::ConstPtr& msg)
    {
        if (FRIMode_ != FastResearchInterface::JOINT_POSITION_CONTROL)
        {
            float commandedForcesAndTorques[NUMBER_OF_CART_DOFS];
            for (int i = 0; i < NUMBER_OF_CART_DOFS; i++)
            {
                commandedForcesAndTorques[i] = msg->data[i];
            }
            FRI_->SetCommandedCartForcesAndTorques(commandedForcesAndTorques);
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
