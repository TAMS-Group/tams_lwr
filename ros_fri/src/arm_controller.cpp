// Johannes Liebrecht --- 8liebrec@informatik.uni-hamburg.de

/* TODO / WARNING / ERRORS:
  - the use of coordinate systems is really a mess;
    while all ROS related things are consistent with our URDF,
    the KUKA systems (XYZ-ABC) are weird, to say the least.

  - read, configure, and respect joint LIMITS
  - same for velocity, acceleration, and efforts
  - rename topics without leading slash (/lwr_xxx -> lwr_xxx)
  - configurable debug messages, don't print one line
    of output for every loop iteration unless requested
  - try to detect when the robot has died...
*/

/**
 * 2017.02.14 - allow incremental position goals
 * 2017.02.10 - fix signs on wrench, shutdown() hook
 * 2017.02.01 - publish wrench on correctly rotated tool frame
 * 2017.01.19 - publish full fri_status
 * 2017.01.18 - got KuKA Cartesian pose matched to ROS, add debuglevel
 * 2017.01.17 - tf broadcaster for Kuka vs. ROS pose debugging
 * 2017.01.17 - add kinematic limits, update jointVelocityGoal stuff
 * 2017.01.17 - rename arm base link, rename topics
 * 2017.01.04 - node params for joint/vel/acc limits
 * 2017.01.03 - re-start working on this...
 * 2016.07.04 - use Wrench instead of FloatArray
 * 2016.06.29 - catkinize fnh

   (C) 2016, 2017 Norman Hendrich, hendrich@informatik.uni-hamburg.de

   Early version

   (C) 2014, 2015 Johannes Liebrecht, 8liebrec@informatik.uni-hamburg.de
 */

#include <string>
#include <csignal>
#include <ros/ros.h>

#include <std_msgs/Bool.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/UInt32.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/WrenchStamped.h>

#include <actionlib/server/action_server.h>
#include <actionlib/server/server_goal_handle.h>
#include <control_msgs/FollowJointTrajectoryAction.h>

//#include <TypeIRML.h>

#include <ReflexxesAPI.h>
#include <RMLPositionFlags.h>
#include <RMLPositionInputParameters.h>
#include <RMLPositionOutputParameters.h>
#include <RMLVelocityInputParameters.h>
#include <RMLVelocityOutputParameters.h>

#include <FastResearchInterface.h>
#include <LinuxAbstraction.h>

#include <ros_fri_msgs/RMLVelocityInputParameters.h>
#include <ros_fri_msgs/RMLPositionInputParameters.h>
#include <ros_fri_msgs/LWRStates.h>
#include <ros_fri_msgs/fri_status.h>

// we need MoveIt RobotState to do our own FK calculations as fast as possible
//
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_model/joint_model_group.h>
#include <eigen_conversions/eigen_msg.h>

#include <moveit_msgs/GetPositionFK.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

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

// static pthread_mutex_t mutex = PTHREAD_MUTEX_INITIALIZER;
static pthread_mutex_t mutex;

typedef typename actionlib::ActionServer<control_msgs::FollowJointTrajectoryAction>::GoalHandle GoalHandle;
typedef boost::shared_ptr<const control_msgs::FollowJointTrajectoryGoal> Goal;

enum motionState
{
    UNKNOWN = -1,
    READY = 0,
    TRAJECTORY_GOAL = 1,
    JOINT_POS_GOAL = 2,
    JOINT_VEL_GOAL = 3,
};

class lwr_action_server
{
protected:
    ros::NodeHandle nh_;

    // note: NodeHandle instance must be created before this line. Otherwise strange error may occur.
    actionlib::ActionServer<control_msgs::FollowJointTrajectoryAction> as_;
    std::string action_name_;
    bool goal_active_, cancel_goal_;
    int seq_counterFJTF_, seqCounterJointStates_, seqCounterFK_;
    GoalHandle agh_;

    // Fri
    FastResearchInterface* FRI;
    unsigned int currentControlScheme_;
    unsigned int cycles;
    bool shutdown_flag;

    double Position_old_[LBR_MNJ], Position_new_[LBR_MNJ];
    ros::Time time_old_, time_new_, time_delta_;
    double velocity_new_[LBR_MNJ], velocity_old_[LBR_MNJ];
    double acceleration_[LBR_MNJ];

    bool setCancelOnForceAtTCP_;

    float commandedJointPositions_[LBR_MNJ];
    float measuredJointPositions_[LBR_MNJ];
    float commandedJointPositionOffsets_[LBR_MNJ];
    float measuredJointTorques_[LBR_MNJ];
    float estimatedExternalJointTorques_[LBR_MNJ];
    float commandedTorquesInNm_[LBR_MNJ];
    float commandedJointStiffness_[LBR_MNJ];
    float commandedJointDamping_[LBR_MNJ];
    float driveTemperatues_[LBR_MNJ];

    float commandedWrench[NUMBER_OF_CART_DOFS];
    float commandedCartStiffness_[NUMBER_OF_CART_DOFS];
    float commandedCartDamping_[NUMBER_OF_CART_DOFS];
    float estimatedExternalWrench[NUMBER_OF_CART_DOFS];
    float commandedCartPose_[NUMBER_OF_FRAME_ELEMENTS];
    float measuredCartPose_[NUMBER_OF_FRAME_ELEMENTS];
    float commandedCartPoseOffsets_[NUMBER_OF_FRAME_ELEMENTS];

    // kinematic limits as specified in node parameters
    std::vector<std::string> jointNames;          // lwr_arm_0_joint ...
    std::vector<double> lowerJointLimits;         // radians
    std::vector<double> upperJointLimits;         // radians
    std::vector<double> jointVelocityLimits;      // rad/s
    std::vector<double> jointAccelerationLimits;  // rad/s/s
    std::vector<double> jointTorqueLimits;        // Nm

    // Msg
    std_msgs::UInt32 mc_;

    ros::Publisher friStatusPublisher;
    ros::Publisher pub_joints_;

    ros::Publisher pub_motionCompletion_;
    ros::Publisher pub_estimatedExternalWrench;
    ros::Publisher pub_estimatedExternalTcpWrench;

    ros::Publisher pub_driveTemperatues_;
    ros::Publisher pub_estimatedExternalJointTorques_;
    ros::Publisher pub_measuredCartPose_;
    ros::Publisher pub_measuredJointPositions_;
    ros::Publisher pub_commandedJointPositions_;
    ros::Publisher pub_commandedJointPositionOffsets_;
    ros::Publisher pub_measuredJointTorques_;
    ros::Publisher pub_commandedCartPose_;
    ros::Publisher pub_commandedCartPoseOffsets_;

    ros::Subscriber sub_setCurrentControlScheme_;
    ros::Subscriber sub_setJointDamping_;
    ros::Subscriber sub_setJointStiffness_;
    ros::Subscriber sub_setJointTorques_;
    ros::Subscriber sub_setCartDamping_;
    ros::Subscriber sub_setCartStiffness_;
    ros::Subscriber sub_setWrench;
    ros::Subscriber sub_setCancelOnForceAtTCP_;
    ros::Subscriber sub_cancelLWRTrajectory_;
    ros::Subscriber sub_setJointPosition_;
    ros::Subscriber sub_setCartPose_;

    ros::Subscriber sub_jntPosGoal_;
    ros::Subscriber sub_jntVelGoal_;

    tf::TransformBroadcaster* tbr;

    // MoveIt and forward kinematics
    // robot_model_loader::RobotModelLoader robot_model_loader;
    robot_model::RobotModelPtr kinematic_model;
    robot_state::RobotStatePtr kinematic_state;
    robot_model::JointModelGroup* lwr_arm_joint_model_group;
    std::vector<std::string> lwr_arm_joint_model_names;
    ros::ServiceClient ser_fk_;

    double CycleTime_;
    ReflexxesAPI* RML_;
    RMLVelocityInputParameters* VIP_;
    RMLVelocityOutputParameters* VOP_;
    RMLVelocityFlags VFlags_;
    bool VOP_initialized;
    ros::Time msgVOP_timestamp;

    RMLPositionInputParameters* PIP_;
    RMLPositionOutputParameters* POP_;
    RMLPositionFlags PFlags_;
    bool POP_initialized;
    ros::Time msgPOP_timestamp;

    motionState state_;
    ros_fri_msgs::RMLPositionInputParameters msgPOP_;
    ros_fri_msgs::RMLVelocityInputParameters msgVOP_;

    moveit::core::RobotState* kinematic_state_;

    int debuglevel;

public:
    /**
     * constructor for the lwr_action_server (aka arm_controller aka ros_fri).
     * Initializes the data-structures, creates the ROS publishers and subscribers,
     * and starts the action server for trajectory requests.
     */
    lwr_action_server(std::string name)
        : as_(nh_, name, false), action_name_("lwr_action_server/joint_trajectory_action")
    {
        // initialize data structures
        //
        goal_active_ = false;
        cancel_goal_ = false;
        seq_counterFJTF_ = 0;
        seqCounterJointStates_ = 0;
        seqCounterFK_ = 0;
        setCancelOnForceAtTCP_ = false;
        cycles = 0;
        shutdown_flag = false;

        memset(commandedJointPositions_, 0x0, LBR_MNJ * sizeof(float));
        memset(commandedJointPositionOffsets_, 0x0, LBR_MNJ * sizeof(float));
        memset(measuredJointPositions_, 0x0, LBR_MNJ * sizeof(float));
        memset(measuredJointTorques_, 0x0, LBR_MNJ * sizeof(float));
        memset(estimatedExternalJointTorques_, 0x0, LBR_MNJ * sizeof(float));
        memset(driveTemperatues_, 0x0, LBR_MNJ * sizeof(float));

        memset(commandedWrench, 0x0, NUMBER_OF_CART_DOFS * sizeof(float));
        memset(commandedCartStiffness_, 0x0, NUMBER_OF_CART_DOFS * sizeof(float));
        memset(commandedCartDamping_, 0x0, NUMBER_OF_CART_DOFS * sizeof(float));
        memset(estimatedExternalWrench, 0x0, NUMBER_OF_CART_DOFS * sizeof(float));
        memset(commandedCartPose_, 0x0, NUMBER_OF_FRAME_ELEMENTS * sizeof(float));
        memset(measuredCartPose_, 0x0, NUMBER_OF_FRAME_ELEMENTS * sizeof(float));
        memset(commandedCartPoseOffsets_, 0x0, NUMBER_OF_FRAME_ELEMENTS * sizeof(float));

        // node params, in particular, the kinematic limits
        //
        ros::NodeHandle nnh("lwr");
        nnh.param("debuglevel", debuglevel, 7);  // 11 for all
        ROS_INFO("ros_fri: debuglevel %d", debuglevel);

        std::string ljl, ujl, jvl, jal, jtl;
        nnh.param("lower_joint_limits", ljl, std::string("-2.90 -2.00 -2.90 -2.00 -2.90 -2.00 -2.90"));
        nnh.param("upper_joint_limits", ujl, std::string("2.90 2.00 2.90 2.00 2.90 2.00 2.90"));
        nnh.param("joint_velocity_limits", jvl, std::string("1.90 1.90 2.25 2.25 2.25 3.14 3.14"));
        nnh.param("joint_acceleration_limits", jal, std::string("0.3 0.3 0.5 0.5 0.5 1.0 1.0"));
        nnh.param("joint_torque_limits", jtl, std::string("200 300 200 300 200 300 200"));

        jointNames.push_back("lwr_arm_0_joint");
        jointNames.push_back("lwr_arm_1_joint");
        jointNames.push_back("lwr_arm_2_joint");
        jointNames.push_back("lwr_arm_3_joint");
        jointNames.push_back("lwr_arm_4_joint");
        jointNames.push_back("lwr_arm_5_joint");
        jointNames.push_back("lwr_arm_6_joint");

        parseAsVector(lowerJointLimits, ljl);
        parseAsVector(upperJointLimits, ujl);
        parseAsVector(jointVelocityLimits, jvl);
        parseAsVector(jointAccelerationLimits, jal);
        parseAsVector(jointTorqueLimits, jtl);

        if (debuglevel > 0)
        {
            ROS_INFO("ros_fri: kinematics limits configured as follows:");
            for (int j = 0; j < LBR_MNJ; j++)
            {
                ROS_INFO("joint %d name '%s' lower %10.4lf upper %10.4lf  max_vel %10.4lf max_acc %10.4lf max_torque "
                         "%10.4lf",
                         j, jointNames[j].c_str(), lowerJointLimits[j], upperJointLimits[j], jointVelocityLimits[j],
                         jointAccelerationLimits[j], jointTorqueLimits[j]);
            }
        }

        // transform broadcaster (KuKA FRI Pose)
        //
        try
        {
            // tfl = new tf::TransformListener( nh, ros::Duration(10) );
            // tfl->waitForTransform( world_frame, camera_frame, ros::Time(0), ros::Duration(10) );
            // tfl->lookupTransform( world_frame, camera_frame, ros::Time(0), world_camera_transform);
            // ROS_WARN( "got the %s -> %s transform ok.", world_frame.c_str(), camera_frame.c_str() );

            tbr = new tf::TransformBroadcaster();
        }
        catch (tf::TransformException& exception)
        {
            ROS_ERROR("TransformListener failed: %s", exception.what());
            exit(1);
        }
        if (debuglevel > 0)
            ROS_INFO("ros_fri: got the transform broadcaster.");

        /* MOVEIT crap
            // we need a RobotModel to do our own (fast) FK
            // see MoveIt kinematics tutorial and
           https://github.com/ros-planning/moveit_tutorials/blob/kinetic-devel/doc/pr2_tutorials/kinematics/src/kinematic_model_tutorial.cpp
            // http://docs.ros.org/indigo/api/moveit_ros_planning/html/classrobot__model__loader_1_1RobotModelLoader.html
            //
            robot_model_loader::RobotModelLoader robot_model_loader( "robot_description" );
            kinematic_model = robot_model_loader.getModel();
            ROS_INFO("ros_fri: got the robot model, frame='%s'", kinematic_model->getModelFrame().c_str());

            // the node is configured to also load the SRDF model, so that
            // it can access the predefined planning groups: here, the "arm"
            //
            robot_state::RobotStatePtr tmp_kinematic_state( new robot_state::RobotState(kinematic_model));
            kinematic_state = tmp_kinematic_state;

            kinematic_state->setToDefaultValues();
            lwr_arm_joint_model_group = kinematic_model->getJointModelGroup("arm");
            lwr_arm_joint_model_names = lwr_arm_joint_model_group->getJointModelNames();


            // Forward Kinematics
            // Now, we can compute forward kinematics for a set of random joint
            // values. Note that we would like to find the pose of the
            // "lwr_arm_7_link", which is the "tool" link of the LWR robot.
            //
            // kinematic_state->setToDefaultValues();
            kinematic_state->setToRandomPositions( lwr_arm_joint_model_group);
            const Eigen::Isometry3d &end_effector_state = kinematic_state->getGlobalLinkTransform("lwr_arm_7_link");
            ROS_INFO_STREAM("Translation: " << end_effector_state.translation());
            ROS_INFO_STREAM("Rotation: " << end_effector_state.rotation());
        */

        // register the action server goal and cancel callbacks
        //
        as_.registerGoalCallback(boost::bind(&lwr_action_server::goalCB, this, _1));
        as_.registerCancelCallback(boost::bind(&lwr_action_server::cancelCB, this, _1));

        // publisher
        //
        friStatusPublisher = nnh.advertise<ros_fri_msgs::fri_status>("fri_status", 1);

        pub_joints_ = nnh.advertise<sensor_msgs::JointState>("joint_states", 1);
        pub_motionCompletion_ = nh_.advertise<std_msgs::UInt32>("/lwr_motion_completion", 1);

        // pub_estimatedExternalWrench =
        // nh_.advertise<std_msgs::Float32MultiArray>("/lwr_estimatedExternalCartForcesAndTorques", 1);
        pub_driveTemperatues_ = nh_.advertise<std_msgs::Float32MultiArray>("/lwr_driveTemperatues", 1);

        pub_estimatedExternalWrench = nnh.advertise<geometry_msgs::WrenchStamped>("estimatedExternalWrench", 1);
        pub_estimatedExternalTcpWrench = nnh.advertise<geometry_msgs::WrenchStamped>("estimatedExternalTcpWrench", 1);
        pub_estimatedExternalJointTorques_ = nnh.advertise<sensor_msgs::JointState>("estimatedExternalJointTorques", 1);

        // pub_measuredCartPose_ = nh_.advertise<std_msgs::Float32MultiArray>("/lwr_measuredCartPose", 1);
        pub_measuredCartPose_ = nnh.advertise<geometry_msgs::PoseStamped>("measuredCartPose", 1);

        pub_commandedCartPose_ = nh_.advertise<std_msgs::Float32MultiArray>("/lwr_commandedCartPose", 1);
        pub_commandedCartPoseOffsets_ = nh_.advertise<std_msgs::Float32MultiArray>("/lwr_commandedCartPoseOffsets", 1);
        pub_measuredJointPositions_ = nh_.advertise<std_msgs::Float32MultiArray>("/lwr_measuredJointPositions", 1);
        pub_commandedJointPositions_ = nh_.advertise<std_msgs::Float32MultiArray>("/lwr_commandedJointPositions", 1);
        pub_commandedJointPositionOffsets_ =
            nh_.advertise<std_msgs::Float32MultiArray>("/lwr_commandedJointPositionOffsets", 1);
        pub_measuredJointTorques_ = nh_.advertise<std_msgs::Float32MultiArray>("/lwr_measuredJointTorques", 1);

        // subscribers
        //
        sub_setCurrentControlScheme_ =
            nnh.subscribe("setCurrentControlScheme", 1, &lwr_action_server::setCurrentControlSchemeSubscriber, this);
        sub_setJointDamping_ = nnh.subscribe("setJointDamping", 1, &lwr_action_server::setJointDamping, this);
        sub_setJointStiffness_ = nnh.subscribe("setJointStiffness", 1, &lwr_action_server::setJointStiffness, this);
        sub_setJointTorques_ = nnh.subscribe("setJointTorques", 1, &lwr_action_server::setJointStiffness, this);
        sub_setCartDamping_ = nnh.subscribe("setCartDamping", 1, &lwr_action_server::setCartDamping, this);
        sub_setCartStiffness_ = nnh.subscribe("setCartStiffness", 1, &lwr_action_server::setCartStiffness, this);

        sub_setWrench = nnh.subscribe("setCartForcesAndTorques", 1, &lwr_action_server::setCartForcesAndTorques, this);
        sub_setCancelOnForceAtTCP_ =
            nnh.subscribe("setCancelOnForceAtTCP", 1, &lwr_action_server::setCancelOnForceAtTCP, this);
        sub_cancelLWRTrajectory_ =
            nnh.subscribe("setCancelLWRTrajectory", 1, &lwr_action_server::setCancelLWRTrajectory, this);
        sub_setJointPosition_ = nnh.subscribe("setJointPosition", 1, &lwr_action_server::setJointPosition, this);
        sub_setCartPose_ = nnh.subscribe("setCartPose", 1, &lwr_action_server::setCartPose, this);

        sub_jntPosGoal_ = nnh.subscribe("jointPositionGoal", 0, &lwr_action_server::setJntPosGoal, this);
        sub_jntVelGoal_ = nnh.subscribe("jointVelocityGoal", 0, &lwr_action_server::setJntVelGoal, this);

        // forward kinematics service
        ser_fk_ = nnh.serviceClient<moveit_msgs::GetPositionFK>("/compute_fk");

        // FRI
        FRI = new FastResearchInterface("/opt/FRILibrary/etc/980039-FRI-Driver.init");

        setCurrentControlScheme(FastResearchInterface::JOINT_POSITION_CONTROL);
        // setCurrentControlScheme(FastResearchInterface::JOINT_IMPEDANCE_CONTROL);

        // init position and timers for velocity
        //
        FRI->GetMeasuredJointPositions(measuredJointPositions_);
        for (unsigned int j = 0; j < LBR_MNJ; j++)
        {
            Position_old_[j] = measuredJointPositions_[j];
        }
        time_old_ = ros::Time::now();
        FRI->WaitForKRCTick();
        FRI->GetMeasuredJointPositions(measuredJointPositions_);
        for (unsigned int j = 0; j < LBR_MNJ; j++)
        {
            Position_new_[j] = measuredJointPositions_[j];
        }
        time_new_ = ros::Time::now();
        time_delta_.sec = time_new_.sec - time_old_.sec;
        time_delta_.nsec = time_new_.nsec - time_old_.nsec;

        // ReflexxesAPI
        //
        CycleTime_ = 0.01;
        RML_ = new ReflexxesAPI(LBR_MNJ, CycleTime_);
        VIP_ = new RMLVelocityInputParameters(LBR_MNJ);
        VOP_ = new RMLVelocityOutputParameters(LBR_MNJ);
        VOP_initialized = false;

        PIP_ = new RMLPositionInputParameters(LBR_MNJ);
        POP_ = new RMLPositionOutputParameters(LBR_MNJ);
        POP_initialized = false;

        // joint impedance mode
        //
        ROS_ERROR("before joint impedance mode...");
        for (unsigned int i = 0; i < LBR_MNJ; i++)
        {
            commandedJointStiffness_[i] = (float)200.0;
            commandedJointDamping_[i] = (float)0.7;
            commandedTorquesInNm_[i] = (float)0.0;
        }
        /*
            FRI->SetCommandedJointStiffness(commandedJointStiffness_);
            FRI->SetCommandedJointDamping(commandedJointDamping_);
            FRI->SetCommandedJointTorques(commandedTorquesInNm_);
        */

        ROS_ERROR("before cart stiffness mode...");
        // TODO check cart impedance mode
        //
        for (unsigned int i = 0; i < NUMBER_OF_CART_DOFS; i++)
        {
            commandedCartStiffness_[i] = (float)200.0;
            commandedCartDamping_[i] = (float)0.7;
            commandedWrench[i] = (float)0.0;
        }
        /*
            FRI->SetCommandedCartStiffness(commandedCartStiffness_);
            FRI->SetCommandedCartDamping(commandedCartDamping_);
            FRI->SetCommandedCartForcesAndTorques(commandedWrench);
        */
        ROS_ERROR("after cart stiffness mode...");

        // start the (trajectory execution) action server
        as_.start();
        state_ = READY;
        if (debuglevel > 0)
            ROS_INFO("ros_fri: <init> completed.");
    }  // end constructor

    /**
     * destructor
     */
    ~lwr_action_server()
    {
        FRI->StopRobot();
        delete FRI;
    }

    void parseAsVector(std::vector<double>& data, std::string tokens)
    {
        double a, b, c, d, e, f, g;
        int n = sscanf(tokens.c_str(), "%lf %lf %lf %lf %lf %lf %lf", &a, &b, &c, &d, &e, &f, &g);
        if (n != LBR_MNJ)
        {
            ROS_ERROR("ros_fri: Failed to parse initialization data '%s', found %d tokens.", tokens.c_str(), n);
            exit(1);
        }

        data.resize(LBR_MNJ);
        data[0] = a;
        data[1] = b;
        data[2] = c;
        data[3] = d;
        data[4] = e;
        data[5] = f;
        data[6] = g;
    }

    double clamp(double value, double lower, double upper)
    {
        assert(upper > lower);
        if (value >= upper)
            return upper;
        if (value <= lower)
            return lower;
        return value;
    }

    /**
     * actionlib trajectory goal callback
     */
    void goalCB(GoalHandle gh)
    {
        if (!goal_active_)
        {
            // TODO check if order and joint names are correct else gh.setRejected();
            gh.setAccepted();
            agh_ = gh;
            goal_active_ = true;
            state_ = TRAJECTORY_GOAL;
        }
        else
        {
            gh.setRejected();
        }
    }

    /**
     * initialize a joint position trajectory and begin execution.
     */
    void executeJointPositionTrajectory()
    {
        if (goal_active_)
        {
            // helper variables
            float remember = estimatedExternalWrench[1];
            float forceThreshold = 1.0;

            // send each trajectory point to FRI
            Goal g = agh_.getGoal();
            ros::Duration time_from_start;
            time_from_start.sec = 0;
            time_from_start.nsec = 0;
            ros::Time time_start = ros::Time::now();

            for (unsigned int i = 0; i < g->trajectory.points.size(); i++)
            {
                FRI->WaitForKRCTick();

                if (!FRI->IsMachineOK())
                {
                    ROS_ERROR("ros_fri: ERROR the machine is not ready anymore.");
                    agh_.setAborted();
                    exit(EXIT_FAILURE);
                }

                for (unsigned int j = 0; j < LBR_MNJ; j++)
                {
                    commandedJointPositions_[j] = g->trajectory.points[i].positions[j];
                }
                FRI->SetCommandedJointPositions(commandedJointPositions_);

                publishJointStates();

                time_from_start = (ros::Time::now() - time_start);
                publishFeedback(g, i, time_from_start);

                mc_.data = (i + 1) * 100 / g->trajectory.points.size();
                pub_motionCompletion_.publish(mc_);

                publishLWRStates();

                // check cancel request
                if (cancel_goal_)
                {
                    ROS_INFO("ros_fri: Goal canceled\n");
                    agh_.setCanceled();
                    cancel_goal_ = false;
                    stopLwr();
                    return;
                }

                /*
                // check measured and commanded JointValues and stop motion if necessary
                if (i > 0)
                {
                  for (unsigned int j = 0; j < LBR_MNJ; j++)
                  {
                  //  ROS_INFO("Don't touch me )) -- %f", fabs(commandedJointPositions_[j] -
                measuredJointPositions_[j])); if (fabs(commandedJointPositions_[j] - measuredJointPositions_[j]) > 0.02)
                    {
                      ROS_INFO("lwr_action_server ERROR: The differenz between measured and commanded jointvalues is
                higher then threshold!");
                      //cancel Goalhandle
                        goal_active_ = false;
                          stopLwr();
                      agh_.setAborted();
                      return;
                    }
                  }
                }
                */

                // cancel trajectory if force is exerted at the y-axis
                if (setCancelOnForceAtTCP_)
                {
                    ROS_INFO("CancelOnForceAtTCP!%f\n", fabs(estimatedExternalWrench[1] - remember));

                    if (fabs(estimatedExternalWrench[1] - remember) > forceThreshold)
                    {
                        ROS_INFO("lwr_action_server INFO: CancelOnForceAtTCP was activated. \n");
                        // cancel Goalhandle
                        goal_active_ = false;
                        stopLwr();
                        agh_.setAborted();
                        setCancelOnForceAtTCP_ = false;
                        return;
                    }
                }
                remember = estimatedExternalWrench[1];

            }  // end for all trajectory points

            agh_.setSucceeded();
            goal_active_ = false;
            state_ = READY;
        }  // end if goal_active
    }

    /**
     * execute a given trajectory in joint impedance control mode
     */
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
                FRI->WaitForKRCTick();

                if (!FRI->IsMachineOK())
                {
                    ROS_INFO("lwr_action_server: ERROR the machine is not ready anymore.");
                    agh_.setAborted();
                    exit(EXIT_FAILURE);
                }

                for (unsigned int j = 0; j < LBR_MNJ; j++)
                {
                    commandedJointPositions_[j] = g->trajectory.points[i].positions[j];
                }

                FRI->SetCommandedJointPositions(commandedJointPositions_);

                /*
                ROS_ERROR( "setting joint stiffness and damping..." );
                        FRI->SetCommandedJointPositions(commandedJointPositions_);
                        FRI->SetCommandedJointStiffness(commandedJointStiffness_);
                        FRI->SetCommandedJointDamping(commandedJointDamping_);
                        FRI->SetCommandedJointTorques(commandedTorquesInNm_);
                ROS_ERROR( "setting joint stiffness and damping... ok" );
                */

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
                    stopLwr();
                    return;
                }
            }  // end for all trajectory points
            agh_.setSucceeded();
            goal_active_ = false;
            state_ = READY;
            ;
        }
    }  // end joint impedance trajectory

    /**
     * execute a joint trajectory using cartesian impedance control mode
     */
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

            std::string names[] = { "lwr_arm_0_joint", "lwr_arm_1_joint", "lwr_arm_2_joint", "lwr_arm_3_joint",
                                    "lwr_arm_4_joint", "lwr_arm_5_joint", "lwr_arm_6_joint" };
            sensor_msgs::JointState joint_values;

            for (unsigned int i = 0; i < g->trajectory.points.size(); i++)
            {
                FRI->WaitForKRCTick();

                if (!FRI->IsMachineOK())
                {
                    ROS_INFO("lwr_action_server: ERROR the machine is not ready anymore.");
                    agh_.setAborted();
                    exit(EXIT_FAILURE);
                }

                ROS_INFO("Time1 %i . %i \n", ros::Time::now().sec, ros::Time::now().nsec);

                for (int j = 0; j < LBR_MNJ; j++)
                {
                    joint_values.name.push_back(names[j]);
                    joint_values.position.push_back(g->trajectory.points[i].positions[j]);
                }

                kinematic_state_->setVariableValues(joint_values);
                const Eigen::Isometry3d& end_effector_state_temp = kinematic_state_->getGlobalLinkTransform("tip");

                geometry_msgs::Pose pose;
                tf::poseEigenToMsg(end_effector_state_temp, pose);
                /*
                ROS_INFO("position_x %f", pose.position.x);
                ROS_INFO("position_y %f", pose.position.y);
                ROS_INFO("position_z %f", pose.position.z);
                ROS_INFO("orientation_x %f", pose.orientation.x);
                ROS_INFO("orientation_y %f", pose.orientation.y);
                ROS_INFO("orientation_z %f", pose.orientation.z);
                ROS_INFO("orientation_w %f", pose.orientation.w);
                */
                ROS_INFO("Time2 %i . %i \n", ros::Time::now().sec, ros::Time::now().nsec);

                // URDF specific changes <origin rpy="0 0 3.1415" xyz="0 0 0.804"/>
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
                FRI->GetMeasuredCartPose(measuredCartPose);

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

                ROS_ERROR("setting cart stiffness and damping...");
                // now send data to robot
                FRI->SetCommandedCartStiffness(commandedCartStiffness_);
                FRI->SetCommandedCartDamping(commandedCartDamping_);
                FRI->SetCommandedCartForcesAndTorques(commandedWrench);
                FRI->SetCommandedCartPose(commandedCartPose_);
                ROS_ERROR("setting cart stiffness and damping...");

                ROS_INFO("RUN %i\n", i);
                ROS_INFO("Time3 %i . %i \n", ros::Time::now().sec, ros::Time::now().nsec);
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
                    stopLwr();
                    return;
                }
            }  // for all trajectory points
            agh_.setSucceeded();
            goal_active_ = false;
        }
    }  // end cartesian impedance trajectory

    void cancelCB(GoalHandle gh)
    {
        // tell FRI we want to stop the current (if any) trajectory
        cancel_goal_ = true;
        // cancel Goalhandle
        goal_active_ = false;
    }

    /**
     * use FRI velocity mode to stop the robot as quickly as possible
     * (given the active velocity and acceleration limits).
     */
    void stopLwr()
    {
        ROS_INFO("ros_fri: stopLwr...");

        // load current position
        //
        FRI->GetMeasuredJointPositions(measuredJointPositions_);
        for (unsigned int j = 0; j < LBR_MNJ; j++)
        {
            VIP_->CurrentPositionVector->VecData[j] = measuredJointPositions_[j];
            VIP_->CurrentVelocityVector->VecData[j] = velocity_new_[j];
            VIP_->TargetVelocityVector->VecData[j] = 0.0;
            VIP_->MaxAccelerationVector->VecData[j] = jointAccelerationLimits[j];
            VIP_->SelectionVector->VecData[j] = true;
            VIP_->SetMinimumSynchronizationTime(0.0);  // as fast as possible
        }

        // stop trajectory calculated by RML velocity algorithm. The calculated
        // intermediate joint positions are then sent to the robot.
        //
        int ResultValue = 0;
        while ((FRI->IsMachineOK()) && (ResultValue != ReflexxesAPI::RML_FINAL_STATE_REACHED))
        {
            FRI->WaitForKRCTick();
            ResultValue = RML_->RMLVelocity(*VIP_, VOP_, VFlags_);
            VOP_initialized = true;
            if (ResultValue < 0)
            {
                printf("lwr_action_server: ERROR during trajectory generation  (%d).\n", ResultValue);
                break;
            }

            for (unsigned int j = 0; j < LBR_MNJ; j++)
            {
                commandedJointPositions_[j] = VOP_->NewPositionVector->VecData[j];
            }
            FRI->SetCommandedJointPositions(commandedJointPositions_);

            // prepare for next iteration
            *VIP_->CurrentPositionVector = *VOP_->NewPositionVector;
            *VIP_->CurrentVelocityVector = *VOP_->NewVelocityVector;
            *VIP_->CurrentAccelerationVector = *VOP_->NewAccelerationVector;

            publishJointStates();
            publishLWRStates();
        }

        VOP_initialized = false;

        // FIXME: update node state to indicate that the robot is now safely stopped
    }  // stop LWR

    void publishFeedback(Goal g, unsigned int counter, ros::Duration time_from_start)
    {
        FRI->GetMeasuredJointPositions(measuredJointPositions_);
        FRI->GetMeasuredJointTorques(commandedTorquesInNm_);
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
            error.positions.push_back(fabs(g->trajectory.points[counter].positions[j]) -
                                      fabs(measuredJointPositions_[j]));

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

    /**
     * publish current LWR joint_state
     */
    void publishJointStates()
    {
        pthread_mutex_lock(&mutex);

        FRI->GetMeasuredJointPositions(measuredJointPositions_);
        FRI->GetMeasuredJointTorques(measuredJointTorques_);

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
        js.header.frame_id = "lwr_arm_base_link";

        for (int j = 0; j < LBR_MNJ; j++)
        {
            // ROS_INFO("names %f value \n",JointValuesInRad_[j]);
            js.name.push_back(jointNames[j]);
            js.position.push_back(measuredJointPositions_[j]);
            js.velocity.push_back(velocity_new_[j]);
            js.effort.push_back(measuredJointTorques_[j]);
        }

        // publish current joint_states (/lwr/joint_states)
        //
        pub_joints_.publish(js);
        seqCounterJointStates_++;

        // now re-send the message with estimatedExternalTorques
        //
        FRI->GetEstimatedExternalJointTorques(estimatedExternalJointTorques_);
        for (int j = 0; j < LBR_MNJ; j++)
        {
            js.effort[j] = estimatedExternalJointTorques_[j];
        }
        pub_estimatedExternalJointTorques_.publish(js);
        pthread_mutex_unlock(&mutex);
    }

    /*
    //new untested Version
    void publishLWRStates()
    {
      FRI->GetDriveTemperatures(driveTemperatues_);
      FRI->GetMeasuredJointPositions(measuredJointPositions_);
      FRI->GetCommandedJointPositions(commandedJointPositions_);
      FRI->GetCommandedJointPositionOffsets(commandedJointPositionOffsets_);
      FRI->GetMeasuredJointTorques(measuredJointTorques_);
      FRI->GetEstimatedExternalJointTorques(estimatedExternalJointTorques_);
      FRI->GetEstimatedExternalCartForcesAndTorques(estimatedExternalWrench);

      std_msgs::Float32MultiArray driveTemperatues, measuredJointPositions, commandedJointPositions,
    commandedJointPositionOffsets, measuredJointTorques, estimatedExternalJointTorques,
    estimatedExternalCartForcesAndTorques;

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
        estimatedExternalCartForcesAndTorques.data.push_back(estimatedExternalWrench[j]);
      }

      pub_estimatedExternalWrench.publish(estimatedExternalCartForcesAndTorques);
      pub_driveTemperatues_.publish(driveTemperatues);
      pub_measuredJointPositions_.publish(measuredJointPositions);
      pub_commandedJointPositions_.publish(commandedJointPositions);
      pub_commandedJointPositionOffsets_.publish(commandedJointPositionOffsets);
      pub_measuredJointTorques_.publish(measuredJointTorques);
      pub_estimatedExternalJointTorques_.publish(estimatedExternalJointTorques);

      //onyl if cart impedance mode
      if (currentControlScheme_ == FastResearchInterface::CART_IMPEDANCE_CONTROL)
      {
        FRI->GetMeasuredCartPose(measuredCartPose_);
        FRI->GetCommandedCartPose(commandedCartPose_);
        FRI->GetCommandedCartPoseOffsets(commandedCartPoseOffsets_);
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
    } // publishLWRStates_untested
    */

    void publishLWRStates()
    {
        ros_fri_msgs::fri_status msg;

        msg.driveTemperatures.resize(LBR_MNJ);
        msg.measuredJointPositions.resize(LBR_MNJ);
        msg.measuredJointTorques.resize(LBR_MNJ);
        msg.estimatedExternalJointTorques.resize(LBR_MNJ);
        msg.estimatedJointVelocities.resize(LBR_MNJ);
        msg.estimatedJointPositionErrors.resize(LBR_MNJ);
        msg.estimatedJointVelocityErrors.resize(LBR_MNJ);

        msg.commandedJointPositions.resize(LBR_MNJ);
        msg.commandedJointPositionOffsets.resize(LBR_MNJ);
        msg.commandedJointTorques.resize(LBR_MNJ);
        msg.commandedJointStiffness.resize(LBR_MNJ);
        msg.commandedJointDamping.resize(LBR_MNJ);

        msg.header.stamp = ros::Time::now();
        msg.header.frame_id = "lwr_arm_base_link";
        msg.header.seq = cycles++;

        msg.friMode = FRI->GetFRIMode();
        msg.friCurrentControlScheme = FRI->GetCurrentControlScheme();
        msg.friMachineOK = FRI->IsMachineOK();
        msg.anyDriveError = FRI->DoesAnyDriveSignalAnError();
        msg.anyDriveWarning = FRI->DoesAnyDriveSignalAWarning();

        msg.jointNames = jointNames;

        FRI->GetDriveTemperatures(&(msg.driveTemperatures[0]));
        FRI->GetMeasuredJointPositions(&(msg.measuredJointPositions[0]));
        FRI->GetMeasuredJointTorques(&(msg.measuredJointTorques[0]));
        FRI->GetEstimatedExternalJointTorques(&(msg.estimatedExternalJointTorques[0]));

        friStatusPublisher.publish(msg);

        // xxxzzz

        FRI->GetEstimatedExternalJointTorques(estimatedExternalWrench);
        FRI->GetEstimatedExternalCartForcesAndTorques(estimatedExternalWrench);
        FRI->GetDriveTemperatures(driveTemperatues_);

        std_msgs::Float32MultiArray DriveTemperatues;
        std_msgs::Float32MultiArray EstimatedExternalJointTorquesInNm;

        for (unsigned int j = 0; j < LBR_MNJ; j++)
        {
            DriveTemperatues.data.push_back(driveTemperatues_[j]);
            EstimatedExternalJointTorquesInNm.data.push_back(estimatedExternalWrench[j]);
        }
        pub_driveTemperatues_.publish(DriveTemperatues);
        // we now publish JointState here!!! pub_estimatedExternalJointTorques_.publish(EstimatedExternalJointTorquesInNm);

        /*
        std_msgs::Float32MultiArray EstimatedExternalCartForcesAndTorques;
        for (unsigned int j = 0; j < NUMBER_OF_FRAME_ELEMENTS; j++)
        {
          EstimatedExternalCartForcesAndTorques.data.push_back(estimatedExternalWrench[j]);
        }
        pub_estimatedExternalWrench.publish(EstimatedExternalCartForcesAndTorques);
        */

        // FIXME: this needs to be transformed to the local frame to be useful...

        FRI->GetMeasuredCartPose(measuredCartPose_);  // 12 elements of data
        if (debuglevel > 10)
        {
            ROS_INFO("ros_fri: FRI measured Cartesian (tool) pose: ");
            ROS_INFO("%10.4lf %10.4lf %10.4lf %10.4lf", measuredCartPose_[0], measuredCartPose_[1],
                     measuredCartPose_[2], measuredCartPose_[3]);
            ROS_INFO("%10.4lf %10.4lf %10.4lf %10.4lf", measuredCartPose_[4], measuredCartPose_[5],
                     measuredCartPose_[6], measuredCartPose_[7]);
            ROS_INFO("%10.4lf %10.4lf %10.4lf %10.4lf", measuredCartPose_[8], measuredCartPose_[9],
                     measuredCartPose_[10], measuredCartPose_[11]);
            ROS_INFO("%10.4lf %10.lf %10.4lf %10.4lf", 0.0, 0.0, 0.0, 1.0);
            ROS_INFO(" ");
        }

        // convert the weird coordinate system used by Kuka into
        // something that makes sense in ROS... I have not fully
        // understood _why_ some signs have to be inverted, but
        // the resulting frame matches the "tcp" frame from the URDF.
        //
        tf::Vector3 origin(-measuredCartPose_[3], -measuredCartPose_[7], measuredCartPose_[11]);
        tf::Quaternion quat;
        tf::Matrix3x3 rot(  // check row/column ordering!!!
            -measuredCartPose_[1], measuredCartPose_[0], -measuredCartPose_[2], -measuredCartPose_[5],
            measuredCartPose_[4], -measuredCartPose_[6], measuredCartPose_[9], -measuredCartPose_[8],
            measuredCartPose_[10]);
        rot.getRotation(quat);
        if (debuglevel > 10)
        {
            ROS_INFO("Quaternion is %10.4lf %10.4lf %10.4lf %10.4lf", quat.getX(), quat.getY(), quat.getZ(),
                     quat.getW());
        }

        // transform broadcaster for the KukA transform...
        // the frame _should_ match the "tcp" frame from the URDF
        tf::Transform friPose(quat, origin);
        tf::StampedTransform SFP(friPose, ros::Time::now(), "lwr_arm_base_link", "kuka_fri_tcp");
        tbr->sendTransform(SFP);

        // publish current tool center point pose from FRI
        //
        geometry_msgs::PoseStamped pose;
        pose.header.stamp = ros::Time::now();
        pose.header.frame_id = "kuka_fri_tcp";  // TCP as calculated by FRI
        pose.pose.position.x = origin.getX();
        pose.pose.position.y = origin.getY();
        pose.pose.position.z = origin.getZ();
        pose.pose.orientation.x = quat.getX();
        pose.pose.orientation.y = quat.getY();
        pose.pose.orientation.z = quat.getZ();
        pose.pose.orientation.w = quat.getW();
        pub_measuredCartPose_.publish(pose);

        // xxxzzz;

        // publish external wrench in global (arm base) coordinate frame
        //
        geometry_msgs::WrenchStamped wrench;
        wrench.header.stamp = ros::Time::now();
        wrench.header.frame_id = "lwr_arm_base_link";  // "lwr_arm_7_link"; // KUKA LWR tool flange
        //
        // note that Kuka numbers the axes as x-y-z, but rotations ABC are around z-y-x;
        // also note that the z-axis needs to be inverted...
        //
        wrench.wrench.force.x = estimatedExternalWrench[0];
        wrench.wrench.force.y = estimatedExternalWrench[1];
        wrench.wrench.force.z = -estimatedExternalWrench[2];
        wrench.wrench.torque.x = estimatedExternalWrench[5];
        wrench.wrench.torque.y = estimatedExternalWrench[4];
        wrench.wrench.torque.z = -estimatedExternalWrench[3];
        pub_estimatedExternalWrench.publish(wrench);

        // publish external wrench rotated in tool/TCP coordinate frame
        //
        /* this did NEVER work, neither getBasis() nor getBasis().inverse() */
        tf::Vector3 root_force(wrench.wrench.force.x, wrench.wrench.force.y, wrench.wrench.force.z);
        tf::Vector3 root_torque(wrench.wrench.torque.x, wrench.wrench.torque.y, wrench.wrench.torque.z);
        tf::Vector3 tcp_force = friPose.getBasis().inverse() * root_force;  // .inverse() ??
        // tf::Vector3 tcp_torque = friPose.getOrigin().cross( root_torque ); ???
        tf::Vector3 tcp_torque = friPose.getBasis().inverse() * root_torque;

        // xxxzzz
        geometry_msgs::WrenchStamped wrench2;
        wrench2.header.stamp = ros::Time::now();
        wrench2.header.frame_id = "kuka_fri_tcp";  // "lwr_arm_7_link"; // KUKA LWR tool flange
        wrench2.wrench.force.x = tcp_force.getX();
        wrench2.wrench.force.y = tcp_force.getY();
        wrench2.wrench.force.z = tcp_force.getZ();
        wrench2.wrench.torque.x = tcp_torque.getX();
        wrench2.wrench.torque.y = tcp_torque.getY();
        wrench2.wrench.torque.z = tcp_torque.getZ();
        // wrench2.wrench.force.x  =  estimatedExternalWrench[0];
        // wrench2.wrench.force.y  =  estimatedExternalWrench[1];
        // wrench2.wrench.force.z  = -estimatedExternalWrench[2]; // (!)
        // wrench2.wrench.torque.x =  estimatedExternalWrench[5];
        // wrench2.wrench.torque.y =  estimatedExternalWrench[4];
        // wrench2.wrench.torque.z = -estimatedExternalWrench[3];
        pub_estimatedExternalTcpWrench.publish(wrench2);

        /* current robot pose: TODO check whether T7 or lwr_arm_7_link or TCP...
        for (unsigned int j = 0; j < NUMBER_OF_FRAME_ELEMENTS; j++)
        {
            measuredCartPose.data.push_back(measuredCartPose_[j]);
            commandedCartPose.data.push_back(commandedCartPose_[j]);
            measuredCartPose.data.push_back(commandedCartPoseOffsets_[j]);
        }
        */

    }  // publishLWRStates

    /**
     * set FRI contro mode
     */
    void setCurrentControlScheme(unsigned int CurrentControlScheme)
    {
        ROS_INFO("setCurrentControlScheme %i", CurrentControlScheme);
        int ResultValue = 0;
        ResultValue = FRI->StartRobot(CurrentControlScheme);
        currentControlScheme_ = CurrentControlScheme;

        if ((ResultValue != EOK) && (ResultValue != EALREADY))
        {
            ROS_ERROR("ros_fri: An error occurred during starting up the robot...\n");
            exit(EXIT_FAILURE);
        }
    }

    unsigned int getCurrentControlScheme()
    {
        return currentControlScheme_;
    }

    void setCurrentControlSchemeSubscriber(const std_msgs::UInt32::ConstPtr& msg)
    {
        setCurrentControlScheme(msg->data);
    }

    void setJointPosition(const std_msgs::Float32MultiArray::ConstPtr& msg)
    {
        if ((currentControlScheme_ == FastResearchInterface::JOINT_POSITION_CONTROL) or
            (currentControlScheme_ == FastResearchInterface::JOINT_IMPEDANCE_CONTROL))
        {
            for (int i = 0; i < LBR_MNJ; i++)
            {
                commandedJointPositions_[i] = msg->data[i];
            }
            // FRI->SetCommandedJointTorques(commandedJointPositions_);  // what ????
            FRI->SetCommandedJointPositions(commandedJointPositions_);  // ?!?!?!?!?
        }
    }

    void setJointDamping(const std_msgs::Float32MultiArray::ConstPtr& msg)
    {
        if (currentControlScheme_ == FastResearchInterface::JOINT_IMPEDANCE_CONTROL)
        {
            for (int i = 0; i < LBR_MNJ; i++)
            {
                commandedJointDamping_[i] = msg->data[i];
            }
            FRI->SetCommandedJointDamping(commandedJointDamping_);
        }
    }

    void setJointStiffness(const std_msgs::Float32MultiArray::ConstPtr& msg)
    {
        ROS_INFO("setJointStiffness");
        if (currentControlScheme_ == FastResearchInterface::JOINT_IMPEDANCE_CONTROL)
        {
            for (int i = 0; i < LBR_MNJ; i++)
            {
                commandedJointStiffness_[i] = msg->data[i];
            }
            FRI->SetCommandedJointStiffness(commandedJointStiffness_);
            ROS_INFO("setJointStiffness b");
        }
        ROS_INFO("setJointStiffness c");
    }

    void setJointTorques(const std_msgs::Float32MultiArray::ConstPtr& msg)
    {
        if ((currentControlScheme_ == FastResearchInterface::JOINT_IMPEDANCE_CONTROL) or
            (currentControlScheme_ == FastResearchInterface::JOINT_TORQUE_CONTROL))
        {
            for (int i = 0; i < LBR_MNJ; i++)
            {
                commandedTorquesInNm_[i] = msg->data[i];
            }
            FRI->SetCommandedJointTorques(commandedTorquesInNm_);
        }
    }

    void setCartPose(const std_msgs::Float32MultiArray::ConstPtr& msg)
    {
        if (currentControlScheme_ == FastResearchInterface::CART_IMPEDANCE_CONTROL)
        {
            for (int i = 0; i < LBR_MNJ; i++)
            {
                commandedCartPose_[i] = msg->data[i];
            }
            FRI->SetCommandedCartPose(commandedCartPose_);
        }
    }

    void setCartDamping(const std_msgs::Float32MultiArray::ConstPtr& msg)
    {
        if (currentControlScheme_ == FastResearchInterface::CART_IMPEDANCE_CONTROL)
        {
            for (int i = 0; i < NUMBER_OF_CART_DOFS; i++)
            {
                commandedCartDamping_[i] = msg->data[i];
            }
            FRI->SetCommandedCartDamping(commandedCartDamping_);
        }
    }

    void setCartStiffness(const std_msgs::Float32MultiArray::ConstPtr& msg)
    {
        if (currentControlScheme_ == FastResearchInterface::CART_IMPEDANCE_CONTROL)
        {
            for (int i = 0; i < NUMBER_OF_CART_DOFS; i++)
            {
                commandedCartStiffness_[i] = msg->data[i];
            }
            FRI->SetCommandedCartStiffness(commandedCartStiffness_);
        }
    }

    void setCartForcesAndTorques(const std_msgs::Float32MultiArray::ConstPtr& msg)
    {
        if (currentControlScheme_ == FastResearchInterface::CART_IMPEDANCE_CONTROL)
        {
            for (int i = 0; i < NUMBER_OF_CART_DOFS; i++)
            {
                commandedWrench[i] = msg->data[i];
            }
            FRI->SetCommandedCartForcesAndTorques(commandedWrench);
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

    /**
     * set a new joint position goal for Reflexxes interpolation.
     */
    void setJntPosGoal(const ros_fri_msgs::RMLPositionInputParameters::ConstPtr& msg)
    {
        if (debuglevel > 5)
            ROS_ERROR("ros_fri::setJntPosGoal...");

        msgPOP_timestamp = ros::Time::now();
        if (debuglevel > 2)
            ROS_ERROR("setJntPosGoal t=%lf", msgPOP_timestamp.toSec());

        // ensure that the data structures are allocated
        if (msgPOP_.TargetPositionVector.size() != LBR_MNJ)
            msgPOP_.TargetPositionVector.resize(LBR_MNJ);
        if (msgPOP_.TargetVelocityVector.size() != LBR_MNJ)
            msgPOP_.TargetVelocityVector.resize(LBR_MNJ);
        if (msgPOP_.MaxAccelerationVector.size() != LBR_MNJ)
            msgPOP_.MaxAccelerationVector.resize(LBR_MNJ);
        if (msgPOP_.MaxVelocityVector.size() != LBR_MNJ)
            msgPOP_.MaxVelocityVector.resize(LBR_MNJ);

        pthread_mutex_lock(&mutex);

        for (unsigned int j = 0; j < LBR_MNJ; j++)
        {
            double x = msg->TargetPositionVector[j];
            double xx = clamp(x, lowerJointLimits[j], upperJointLimits[j]);
            if (x != xx)
            {
                ROS_ERROR("Invalid goal, joint %d, requested position %10.4lf clamped %10.4lf", j, x, xx);
            }
            double v = msg->TargetVelocityVector[j];
            double vv = clamp(v, -jointVelocityLimits[j], jointVelocityLimits[j]);
            if (v != vv)
            {
                ROS_ERROR("Invalid goal, joint %d, requested velocity %10.4lf clamped %10.4lf", j, v, vv);
            }
            double a = msg->MaxAccelerationVector[j];
            double aa = clamp(a, -jointAccelerationLimits[j], jointAccelerationLimits[j]);
            if (a != aa)
            {
                ROS_ERROR("Invalid goal, joint %d, requested acceleration %10.4lf clamped %10.4lf", j, a, aa);
            }
            double w = msg->MaxVelocityVector[j];
            double ww = clamp(w, -jointVelocityLimits[j], jointVelocityLimits[j]);
            if (w != ww)
            {
                ROS_ERROR("Invalid goal, joint %d, req. max velocity %10.4lf clamped %10.4lf", j, w, ww);
            }

            msgPOP_.TargetPositionVector[j] = xx;
            msgPOP_.TargetVelocityVector[j] = vv;
            msgPOP_.MaxAccelerationVector[j] = aa;
            msgPOP_.MaxVelocityVector[j] = ww;
        }  // for all joints

        pthread_mutex_unlock(&mutex);

        if (debuglevel > 0)
            ROS_INFO("ros_fri::msgPOP_ target is (%8.4lf %8.4lf %8.4lf %8.4lf %8.4lf %8.4lf %8.4lf)",
                     msgPOP_.TargetPositionVector[0], msgPOP_.TargetPositionVector[1], msgPOP_.TargetPositionVector[2],
                     msgPOP_.TargetPositionVector[3], msgPOP_.TargetPositionVector[4], msgPOP_.TargetPositionVector[5],
                     msgPOP_.TargetPositionVector[6]);

        // load first position: to avoid massive jerk due to slight
        // differences between current motion and planned motion,
        // we do as follows.
        // If we are already in POSITION_MODE, we just use the existing
        // PIP/POP RML data, and fill in a new target velocity. The robot
        // will then track the new velocity.
        //
        // Otherwise, we have to initialize the PIP data from the current
        // data of the robot. This is only a problem if the current velocity
        // is non-zero...
        //
        if ((state_ == JOINT_POS_GOAL) && POP_initialized)
        {
            ROS_ERROR("Received new position target in JOINT_POS_GOAL mode,");
            ROS_INFO("splicing-in the new target position and velocity to existing");
            ROS_INFO("RML PIP/POP data...");

            pthread_mutex_lock(&mutex);
            for (unsigned int j = 0; j < LBR_MNJ; j++)
            {
                PIP_->CurrentPositionVector->VecData[j] = POP_->NewPositionVector->VecData[j];
                PIP_->CurrentVelocityVector->VecData[j] = POP_->NewVelocityVector->VecData[j];

                PIP_->TargetPositionVector->VecData[j] = msgPOP_.TargetPositionVector[j];
                PIP_->TargetVelocityVector->VecData[j] = msgPOP_.TargetVelocityVector[j];
                PIP_->MaxVelocityVector->VecData[j] = msgPOP_.MaxVelocityVector[j];
                PIP_->MaxAccelerationVector->VecData[j] = msgPOP_.MaxAccelerationVector[j];
                PIP_->SelectionVector->VecData[j] = true;

                ROS_INFO("Joint %d target p %10.4lf v %10.4lf  current v %10.4lf  max accel %10.4lf", j,
                         PIP_->TargetPositionVector->VecData[j], PIP_->TargetVelocityVector->VecData[j],
                         PIP_->CurrentVelocityVector->VecData[j], PIP_->MaxAccelerationVector->VecData[j]);
            }
            pthread_mutex_unlock(&mutex);
        }
        else
        {
            ROS_ERROR("Received new position target outside of JOINT_POS_GOAL mode.");
            ROS_ERROR("I need to re-initialize RML PIP/POP from scratch.");

            FRI->GetMeasuredJointPositions(measuredJointPositions_);

            pthread_mutex_lock(&mutex);
            for (unsigned int j = 0; j < LBR_MNJ; j++)
            {
                PIP_->CurrentPositionVector->VecData[j] = measuredJointPositions_[j];
                PIP_->CurrentVelocityVector->VecData[j] = velocity_new_[j];
                PIP_->TargetPositionVector->VecData[j] = msgPOP_.TargetPositionVector[j];
                PIP_->TargetVelocityVector->VecData[j] = msgPOP_.TargetVelocityVector[j];
                PIP_->MaxVelocityVector->VecData[j] = msgPOP_.MaxVelocityVector[j];
                PIP_->MaxAccelerationVector->VecData[j] = msgPOP_.MaxAccelerationVector[j];
                PIP_->SelectionVector->VecData[j] = true;

                ROS_INFO("Joint %d target p %10.4lf v %10.4lf  current v %10.4lf  max accel %10.4lf", j,
                         PIP_->TargetPositionVector->VecData[j], PIP_->TargetVelocityVector->VecData[j],
                         PIP_->CurrentVelocityVector->VecData[j], PIP_->MaxAccelerationVector->VecData[j]);

                if (fabs(velocity_new_[j]) > 0.01)
                {
                    ROS_ERROR("Joint %d current v %10.4lf non zero, can cause jerk.", j, velocity_new_[j]);
                }
            }
            pthread_mutex_unlock(&mutex);
        }

        if (state_ == TRAJECTORY_GOAL)
        {
            // cancel FRI
            cancel_goal_ = true;
        }
        state_ = JOINT_POS_GOAL;

        if (debuglevel > 5)
            ROS_ERROR("ros_fri:  state is now jntPosGoal.");
    }

    // tttzzz
    void stateJntPosGoal()
    {
        if (debuglevel > 5)
            ROS_ERROR("stateJntPosGoal() started...");
        if (debuglevel > 5)
            ROS_INFO("CheckForValidity  %d \n", PIP_->CheckForValidity());

        // target position-based motion generation.
        int ResultValue = 0;
        int iteration = 0;
        while ((FRI->IsMachineOK()) && (ResultValue != ReflexxesAPI::RML_FINAL_STATE_REACHED))
        {
            iteration++;
            if (debuglevel > 5)
                ROS_INFO("jntPosGoal... iteration %d", iteration);
            FRI->WaitForKRCTick();

            pthread_mutex_lock(&mutex);
            ResultValue = RML_->RMLPosition(*PIP_, POP_, PFlags_);
            POP_initialized = true;

            if (ResultValue < 0)
            {
                ROS_ERROR("ros_fri: ERROR during joint position motion generation  (%d).\n", ResultValue);
                break;
            }

            ROS_INFO("joint target  pos meas-pos diff-pos   vel meas-vel diff-vel");
            for (unsigned int j = 0; j < LBR_MNJ; j++)
            {
                ROS_INFO("%d %8.3lf %8.3lf %8.3lf %10.5lf   %8.3lf %8.3lf %10.5lf  %8.3lf", j,
                         PIP_->TargetPositionVector->VecData[j], POP_->NewPositionVector->VecData[j],
                         measuredJointPositions_[j], (POP_->NewPositionVector->VecData[j] - measuredJointPositions_[j]),
                         POP_->NewVelocityVector->VecData[j], velocity_new_[j],
                         (POP_->NewVelocityVector->VecData[j] - velocity_new_[j]),
                         PIP_->TargetVelocityVector->VecData[j]);
            }

            // prepare for next iteration
            *PIP_->CurrentPositionVector = *POP_->NewPositionVector;
            *PIP_->CurrentVelocityVector = *POP_->NewVelocityVector;
            *PIP_->CurrentAccelerationVector = *POP_->NewAccelerationVector;
            pthread_mutex_unlock(&mutex);

            // command robot
            pthread_mutex_lock(&mutex);
            for (unsigned int j = 0; j < LBR_MNJ; j++)
            {
                commandedJointPositions_[j] = POP_->NewPositionVector->VecData[j];
            }
            pthread_mutex_unlock(&mutex);
            FRI->SetCommandedJointPositions(commandedJointPositions_);

            // publish states
            publishJointStates();
            publishLWRStates();

        }  // while not finished
        ROS_ERROR("jntPosGoal: finished.");
        POP_initialized = false;

        // allow accept new Goalhandle in trajectory state
        goal_active_ = false;
        state_ = READY;
        if (debuglevel > 5)
            ROS_INFO("jntPosGoal finished.");
    }

    /**
     * set a new joint velocity goal, then switch mode to execute this motion via RML.
     */
    void setJntVelGoal(const ros_fri_msgs::RMLVelocityInputParameters::ConstPtr& msg)
    {
        pthread_mutex_lock(&mutex);
        msgVOP_timestamp = ros::Time::now();

        if (debuglevel > 2)
            ROS_ERROR("setJntVelGoal t=%lf", msgVOP_timestamp.toSec());

        if (msgVOP_.TargetVelocityVector.size() != LBR_MNJ)
            msgVOP_.TargetVelocityVector.resize(LBR_MNJ);
        if (msgVOP_.MaxAccelerationVector.size() != LBR_MNJ)
            msgVOP_.MaxAccelerationVector.resize(LBR_MNJ);

        for (unsigned int j = 0; j < LBR_MNJ; j++)
        {
            double v = msg->TargetVelocityVector[j];
            double vv = clamp(v, -jointVelocityLimits[j], jointVelocityLimits[j]);
            if (v != vv)
                ROS_ERROR("Velocity goal exceeds limits, clamped: joint %d given %10.4lf using %10.4lf", j, v, vv);
            msgVOP_.TargetVelocityVector[j] = vv;

            double a = msg->MaxAccelerationVector[j];
            double aa = clamp(a, -jointAccelerationLimits[j], jointAccelerationLimits[j]);
            if (a != aa)
                ROS_ERROR("Velocity goal exceeds acc limits: joint %d given %10.4lf using %10.4lf", j, a, aa);
            msgVOP_.MaxAccelerationVector[j] = aa;
        }
        pthread_mutex_unlock(&mutex);

        // load first position: to avoid massive jerk due to slight
        // differences between current motion and planned motion,
        // we do as follows.
        // If we are already in VELOCITY_MODE, we just use the existing
        // VIP/VOP RML data, and fill in a new target velocity. The robot
        // will then track the new velocity.
        //
        // Otherwise, we have to initialize the VIP data from the current
        // data of the robot. This is only a problem if the current velocity
        // is non-zero...
        //
        if ((state_ == JOINT_VEL_GOAL) && VOP_initialized)
        {
            ROS_ERROR("Received new velocity target in JOINT_VEL_GOAL mode,");
            ROS_INFO("splicing-in the new target velocity to existing");
            ROS_INFO("RML VIP/VOP data...");

            pthread_mutex_lock(&mutex);
            for (unsigned int j = 0; j < LBR_MNJ; j++)
            {
                VIP_->CurrentPositionVector->VecData[j] = VOP_->NewPositionVector->VecData[j];
                VIP_->CurrentVelocityVector->VecData[j] = VOP_->NewVelocityVector->VecData[j];
                VIP_->TargetVelocityVector->VecData[j] = msgVOP_.TargetVelocityVector[j];
                VIP_->MaxAccelerationVector->VecData[j] = msgVOP_.MaxAccelerationVector[j];
                VIP_->SelectionVector->VecData[j] = true;
                VIP_->SetMinimumSynchronizationTime(0.05);  // FIXME: don't hardcode

                ROS_INFO("Joint %d target v %10.4lf  current v %10.4lf  max accel %10.4lf", j,
                         VIP_->TargetVelocityVector->VecData[j], VIP_->CurrentVelocityVector->VecData[j],
                         VIP_->MaxAccelerationVector->VecData[j]);
            }
            pthread_mutex_unlock(&mutex);
        }
        else
        {
            ROS_ERROR("Received new velocity target outside of JOINT_VEL_GOAL mode.");
            ROS_ERROR("I need to re-initialize RML VIP/VOP from scratch.");

            FRI->GetMeasuredJointPositions(measuredJointPositions_);

            pthread_mutex_lock(&mutex);
            for (unsigned int j = 0; j < LBR_MNJ; j++)
            {
                VIP_->CurrentPositionVector->VecData[j] = measuredJointPositions_[j];
                VIP_->CurrentVelocityVector->VecData[j] = velocity_new_[j];
                VIP_->TargetVelocityVector->VecData[j] = msgVOP_.TargetVelocityVector[j];
                VIP_->MaxAccelerationVector->VecData[j] = msgVOP_.MaxAccelerationVector[j];
                VIP_->SelectionVector->VecData[j] = true;
                VIP_->SetMinimumSynchronizationTime(0.05);  // FIXME: don't hardcode

                ROS_INFO("Joint %d target v %10.4lf  current v %10.4lf  max accel %10.4lf", j,
                         VIP_->TargetVelocityVector->VecData[j], VIP_->CurrentVelocityVector->VecData[j],
                         VIP_->MaxAccelerationVector->VecData[j]);

                if (fabs(velocity_new_[j]) > 0.01)
                {
                    ROS_ERROR("Joint %d current v %10.4lf non zero, can cause jerk.", j, velocity_new_[j]);
                }
            }
            pthread_mutex_unlock(&mutex);
        }

        if (state_ == TRAJECTORY_GOAL)
        {
            // cancel active trajectory
            cancel_goal_ = true;
        }
        state_ = JOINT_VEL_GOAL;

    }  // setJntVelGoal

    void stateJntVelGoal()
    {
        int ResultValue = 0;

        // target velocity-based motion generation.

        // FNH: actually, we do not want to stop the motion once the
        // target velocity has been reached. However, velocity motion
        // only makes sense when new goals are received frequently.
        // Therefore, we stop when no new messages have been received
        // for a given timeout value. Here we use max 0.5 seconds...

        ros::Time t0 = ros::Time::now();
        bool alive = (t0 - msgVOP_timestamp) < ros::Duration(0.5);

        ROS_ERROR("stateJntVelGoal: now %lf timestamp %lf duration %lf alive %d", t0.toSec(), msgVOP_timestamp.toSec(),
                  (t0 - msgVOP_timestamp).toSec(), alive);

        int iteration = 0;
        // while ((FRI->IsMachineOK()) && (ResultValue != ReflexxesAPI::RML_FINAL_STATE_REACHED))
        while ((FRI->IsMachineOK()) && alive)
        {
            FRI->WaitForKRCTick();

            pthread_mutex_lock(&mutex);
            ResultValue = RML_->RMLVelocity(*VIP_, VOP_, VFlags_);
            VOP_initialized = true;

            if (ResultValue < 0)
            {
                ROS_ERROR("ros_fri: RMLVelocity ERROR result=(%d).", ResultValue);
                break;
            }

            for (unsigned int j = 0; j < LBR_MNJ; j++)
            {
                ROS_INFO("%d %8.3lf %8.3lf %10.5lf   %8.3lf %8.3lf %10.5lf  %8.3lf", j,
                         VOP_->NewPositionVector->VecData[j], measuredJointPositions_[j],
                         (VOP_->NewPositionVector->VecData[j] - measuredJointPositions_[j]),
                         VOP_->NewVelocityVector->VecData[j], velocity_new_[j],
                         (VOP_->NewVelocityVector->VecData[j] - velocity_new_[j]),
                         VIP_->TargetVelocityVector->VecData[j]);
            }

            // prepare for next iteration
            *VIP_->CurrentPositionVector = *VOP_->NewPositionVector;
            *VIP_->CurrentVelocityVector = *VOP_->NewVelocityVector;
            *VIP_->CurrentAccelerationVector = *VOP_->NewAccelerationVector;
            VIP_->SetMinimumSynchronizationTime(0.05);  // FIXME: don't hardcode

            // actually command pos/vel calculated in this cycle to the robot
            for (unsigned int j = 0; j < LBR_MNJ; j++)
            {
                double v = VOP_->NewVelocityVector->VecData[j];
                double x = VOP_->NewPositionVector->VecData[j];
                double xx = clamp(x, lowerJointLimits[j], upperJointLimits[j]);
                if (x != xx)
                {
                    ROS_ERROR("Velocity goal invalid, joint %d requested position %10.4lf limit %10.4lf", j, x, xx);
                    if ((x >= upperJointLimits[j]) && (v >= 0))
                    {
                        ROS_ERROR("Changing target velocity to v=0 on joint %d", j);
                        VIP_->TargetVelocityVector->VecData[j] = 0.0;
                    }
                    if ((x <= lowerJointLimits[j]) && (v <= 0))
                    {
                        ROS_ERROR("Changing target velocity to v=0 on joint %d", j);
                        VIP_->TargetVelocityVector->VecData[j] = 0.0;
                    }
                }
                commandedJointPositions_[j] = xx;
            }
            pthread_mutex_unlock(&mutex);

            FRI->SetCommandedJointPositions(commandedJointPositions_);

            // publish states
            publishJointStates();
            publishLWRStates();

            ros::Time now = ros::Time::now();
            alive = (now - msgVOP_timestamp) < ros::Duration(0.5);
            ROS_INFO("stateJntVelGoal: now %lf timestampd %lf duration %lf alive %d", now.toSec(),
                     msgVOP_timestamp.toSec(), (now - msgVOP_timestamp).toSec(), alive);
        }
        VOP_initialized = false;

        // if we arrive here, we have not received a command for a long time.
        // stop the robot smoothly.
        ROS_ERROR("RML jntVelGoal: no command received for 0.5 secs, stopping the robot!");
        stopLwr();

        // allow accept new Goalhandle in trajectory state
        goal_active_ = false;
        state_ = READY;
    }

    /**
     * returns the current Reflexxes mode: READY, TRAJECTORY, JOINT_POS_GOAL, JOINT_VEL_GOAL, ...
     */
    motionState getState()
    {
        return state_;
    }

    /**
     * shutdown hook.
     */
    int shutdown()
    {
        ROS_ERROR("ros_fri: shutdown() started...");

        goal_active_ = false;
        shutdown_flag = true;
        usleep(2 * 100 * 1000);  // 2 control cycles at 100 Hz

        ROS_ERROR("ros_fri: calling StopRobot() now...");
        FRI->StopRobot();
        // ROS_ERROR( "ros_fri: calling the destructor..." );
        // FRI->~FastResearchInterface();
        ROS_ERROR("ros_fri finished.");

        return 0;
    }

    /**
     * main ROS loop.
     */
    int run()
    {
        ROS_ERROR("ros_fri: run() started...");
        int iterations = 0;

        while (!shutdown_flag && ros::ok())
        {
            iterations++;
            if ((iterations < 10) || ((iterations % 32768) == 0))
            {
                ROS_INFO("ros_fri: iteration %d state %d", iterations, getState());
            }

            switch (getState())  // ready / trajectory / joint-pos / joint-vel / ...
            {
                case READY:  // idle wait
                    FRI->WaitForKRCTick();
                    // ROS_WARN( "ros_fri: run: ready, iterations %d", iterations );
                    break;

                case TRAJECTORY_GOAL:
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

                case JOINT_POS_GOAL:
                    stateJntPosGoal();
                    break;

                case JOINT_VEL_GOAL:
                    stateJntVelGoal();
                    break;

                default:
                    ROS_ERROR("ros_fri: unknown motionState %d", getState());
                    exit(-1);
                    break;
            }

            publishJointStates();
            publishLWRStates();
            // ros::spinOnce(); // not needed with asycn spinners?
            // loop_rate.sleep();
        }
        ROS_INFO("ros_fri: main loop finished.");
        return 0;
    }

};  // end class

static lwr_action_server* lwr_action_server_ptr;

/**
 * C-style signal handler to stop Polhemus tracker streaming
 * when the user hits cntl-c.
 */
void SIGINT_handler(int signal)
{
    ROS_ERROR("ros_fri: received cntl-c (SIGINT), shutting down...");

    if (lwr_action_server_ptr != NULL)
    {
        lwr_action_server_ptr->shutdown();
        usleep(500 * 1000);
    }
    ROS_ERROR("ros_fri: Sigint handler completed, exiting now.");
    exit(0);
}

int main(int argc, char** argv)
{
    pthread_mutexattr_t attr;
    pthread_mutexattr_init(&attr);
    pthread_mutexattr_settype(&attr, PTHREAD_MUTEX_RECURSIVE);
    if (0 != pthread_mutex_init(&mutex, &attr))
    {
        ROS_ERROR("pthread_mutex_init (recursive mutex) failed.");
        exit(1);
    }

    ROS_WARN("ros_fri: starting...");
    ros::init(argc, argv, "ros_fri_lwr_action_server", 1);  // NoSigintHandler
    std::signal(SIGINT, SIGINT_handler);

    ros::AsyncSpinner spinner(4);
    spinner.start();
    ROS_WARN("ros_fri: async spinner started...");

    // instantiate the LWR FRI real-time controller
    //
    lwr_action_server lwr_action_server(ros::this_node::getName());
    lwr_action_server_ptr = &lwr_action_server;
    ROS_WARN("ros_fri: action server started...");

    // we would like to run as fast as possible, but on our
    // hardware we get bad-communication-quality for everything
    // above 100 Hz...
    ros::Rate loop_rate(100);  // 100Hz

    lwr_action_server.run();
}

// rostopic pub -1 /lwr_setCurrentControlScheme std_msgs/UInt32 "data: 10"
