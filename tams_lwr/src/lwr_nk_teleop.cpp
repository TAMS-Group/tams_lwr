/** lwr_nk_teleop.cpp - basic Kuka LWR teleoperation via the
    Korg Nanokontrol MIDI controller.

  This class provides interactive teleoperation (aka "jogging")
  for the KuKA LWR4+ from the Korg Nanokontrol MIDI controller.
  Button-presses on the Nanokontrol are translated to either

  1) incremental joint-space motions,
  2) joint-space motions to selected pre-defined positions,
  3) incremental Cartesian motions in the world frame,
  4) incremental Cartesian motions in the tool (TCP) frame.

  Actual robot motions are performed using the Reflexxes motion
  library and the KuKA Fast Research Interface.

  Notes:

  The necessary FK and IK calculations are performed using the
  MoveIt API; note that "robot_description" and also the corresponding
  semantic description "robot_description_semantic" need to be on
  the param server for this to work.
  (But move_group is not necessary.)

  See:

  http://docs.ros.org/kinetic/api/moveit_tutorials/html/doc/pr2_tutorials/kinematics/src/doc/kinematic_model_tutorial.html
  http://docs.ros.org/indigo/api/moveit_core/html/classmoveit_1_1core_1_1RobotState.html
  https://eigen.tuxfamily.org/dox-devel/group__TutorialGeometry.html

  History:

  2017.02.16 - map knob 8 to gripper control
  2017.02.16 - fix XYZ mode, implement NOA mode, implement known positions
  2017.02.14 - setIK
  2017.02.13 - implement
  2017.02.10 - new file (copied from pa10_nk_teleop)

  (c) 2016-2017 fnh, hendrich@informatik.uni-hamburg.de
*/

#include <string>
#include <vector>
#include <map>
#include <iostream>
#include <csignal>

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/Joy.h>
#include <sensor_msgs/JoyFeedback.h>

// KuKA LWR reflexxes
#include <ros_fri_msgs/RMLVelocityInputParameters.h>
#include <ros_fri_msgs/RMLPositionInputParameters.h>

// Schunk WSG-50 gripper
#include <wsg_50_common/Move.h>

#include <tf/tf.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf_conversions/tf_eigen.h>

// MoveIt
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>

/*
#include <trajectory_msgs/JointTrajectory.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <actionlib/server/simple_action_server.h>
*/

#define NUMBER_OF_JOINTS 7
#define CYCLE_TIME_IN_SECONDS 0.01

#define BLINK_SECONDS 0.5
#define ROBOT_TIMEOUT_SECONDS 1
#define INACTIVE_TIMEOUT_SECONDS 30

#define NUMBER_NK_BUTTONS 35
#define NUMBER_NK_AXES 16
#define NUMBER_NK_CHANNELS 8

// default velocities and position-increments
//
#define SHIFT_SPEEDUP_FACTOR 5
#define JOINTS_MOTION_RAD_PER_SEC 0.03
#define CARTESIAN_MOTION_RAD_PER_SEC 0.03
#define CARTESIAN_MOTION_MM_PER_SEC 0.02

// whether to process cartesian xyz and noa commands
// relative to the wrist (=T6) or the current tool (=TCP)
//
#define CARTESIAN_WRIST_MODE 1
#define CARTESIAN_TCP_MODE 2

// max allowed summed joint-state error for doing incremental
// cartesian motions
#define DELTA_MIN_THRESHOLD 0.1

// NK2 transport-section buttons: index in Joy.buttons vector
//
#define NK_REWIND 24
#define NK_FORWARD 25
#define NK_STOP 26
#define NK_PLAY 27
#define NK_DOT 28

#define NK_CYCLE 29
#define NK_PREV_TRACK 30
#define NK_NEXT_TRACK 31
#define NK_SET_MARKER 32
#define NK_PREV_MARKER 33
#define NK_NEXT_MARKER 34

// NK2 faders and knobs: index in Joy.axes vector
//
#define NK_FADER_0 0
#define NK_FADER_1 1
#define NK_FADER_2 2
#define NK_FADER_3 3
#define NK_FADER_4 4
#define NK_FADER_5 5
#define NK_FADER_6 6
#define NK_FADER_7 7

#define NK_KNOB_0 8
#define NK_KNOB_1 9
#define NK_KNOB_2 10
#define NK_KNOB_3 11
#define NK_KNOB_4 12
#define NK_KNOB_5 13
#define NK_KNOB_6 14
#define NK_KNOB_7 15

// NK2 channel section buttons: index in Joy.buttons messsage
//
#define NK_SOLO_0 0
#define NK_SOLO_1 1
#define NK_SOLO_2 2
#define NK_SOLO_3 3
#define NK_SOLO_4 4
#define NK_SOLO_5 5
#define NK_SOLO_6 6
#define NK_SOLO_7 7

#define NK_MUTE_0 8
#define NK_MUTE_1 9
#define NK_MUTE_2 10
#define NK_MUTE_3 11
#define NK_MUTE_4 12
#define NK_MUTE_5 13
#define NK_MUTE_6 14
#define NK_MUTE_7 15

#define NK_RECORD_0 16
#define NK_RECORD_1 17
#define NK_RECORD_2 18
#define NK_RECORD_3 19
#define NK_RECORD_4 20
#define NK_RECORD_5 21
#define NK_RECORD_6 22
#define NK_RECORD_7 23

// add channel index (0..7) to access the per-channel buttons
//
#define NK_SOLO_BASE 0
#define NK_MUTE_BASE 8
#define NK_RECORD_BASE 16

#define LED_OFF 0
#define LED_ON 1

/**
 * the main teleoperation mode: joints-motion,
 * xyz-Cartesian, noa-Cartesian
 */
enum nk_teleop_state
{
    UNKNOWN = 0,
    WAITING_FOR_ROBOT,   // no joint_states received yet
    ROBOT_ERROR,         // robot died on us?
    STOP,                // teleop disabled
    JOINTS_MOTION_MODE,  // joint space increments
    XYZ_CARTESIAN_MODE,  // Cartesian in global axes
    NOA_CARTESIAN_MODE   // Cartesian in gripper/tool axes
};

enum nk_wrist_or_tcp_ik
{
    UNKNOWN_IK_MODE = 0,
    WRIST_IK_MODE,
    TCP_IK_MODE,
};

/**
 * intialize the given transfrom from a string with six double tokens,
 * namely x y z roll pitch yaw (in meters and radians).
 * Allocates the transform if NULL is given as input.
 * Returns true on success, false on failure.
 */
bool xyzRpyToTransform(tf::Transform* trafo, std::string tokens)  // x y z r p y -> transform
{
    if (trafo == NULL)
        trafo = new tf::Transform();

    double x, y, z, r, p, Y;
    if (6 == sscanf(tokens.c_str(), "%lf %lf %lf %lf %lf %lf", &x, &y, &z, &r, &p, &Y))
    {
        tf::Matrix3x3 basis;
        basis.setEulerYPR(Y, p, r);
        trafo->setOrigin(tf::Vector3(x, y, z));
        trafo->setBasis(basis);
        return true;
    }
    else
    {
        fprintf(stderr, "Failed to create transform from given xyz rpy string '%s'", tokens.c_str());
        return false;
    }
}

/**
 * Teleoperation of the KuKA LWR4+ robot via the KORG
 * nanoKONTROL2 (aka NK2) MIDI controller and the Reflexxes
 * real-time motion library.
 */
class lwr_nk_teleop
{
public:
    lwr_nk_teleop();   // constructor
    ~lwr_nk_teleop();  // desctructor
    int run();         // main loop

    void nanokontrolCallback(const sensor_msgs::Joy joyMessage);
    void jointStateCallback(const sensor_msgs::JointState jointStateMessage);
    void blinkTimerCallback(const ros::TimerEvent& wakeupEvent);
    void schedulerCallback(const ros::TimerEvent& wakeupEvent);

    void publishEmergencyStop(bool stop);
    void publishStopRobot(bool stop);
    void publishCancelTrajectory();
    void publishVelocityGoal();
    void publishPositionGoal();
    void publishNKLEDs();

    void initNK();
    void setNodeState(nk_teleop_state state);
    void setIKMode(nk_wrist_or_tcp_ik ikmode);
    void shutdown();

    bool lookupTransform(std::string root_frame, std::string target_frame, tf::StampedTransform& trafo);
    void printTransform(std::string prefix, tf::Transform trafo);
    void printTransform(std::string prefix, Eigen::Isometry3d trafo);

private:
    // event handler stuff for incoming Joy (=MIDI) messages
    //
    void calculateNanokontrolMessageDiff(sensor_msgs::Joy msg, sensor_msgs::Joy previous);
    bool hasModeChanges();
    bool hasButtonChanges();
    bool hasKnobsChanges();
    bool hasFadersChanges();
    void processModeChangeEvents();
    void processButtonEvents();
    void processJointsMotionButtonEvents();
    void processCartesianXyzButtonEvents();
    void processCartesianNoaButtonEvents();
    void processKnobEvents();
    void processFaderEvents();
    void updateNodeStateLEDs();

    // ROS nodehandle(s)
    ros::NodeHandle nh;
    ros::NodeHandle nnh;
    bool verbose;

    // NK2/teleop state
    nk_teleop_state nodeState;
    pthread_mutex_t nodeStateMutex;
    nk_wrist_or_tcp_ik ikMode;

    std::vector<int> nkButtons;         // current button states
    std::vector<double> nkFaders;       // current (or zero) slider values
    std::vector<bool> nkFadersTracked;  // whether a slider is tracked
    std::vector<double> nkKnobs;        // current (or zero) knob values
    std::vector<bool> nkKnobsTracked;   // whether a knob is tracked already

    std::map<int, bool> nkStateButtonsMap;         // index in Joy message -> state button on/off
    std::map<int, bool> nkStateButtonsChangedMap;  // index in Joy message -> changed true/false
    std::map<int, bool> nkButtonsChangedMap;
    std::map<int, bool> nkKnobsChangedMap;
    std::map<int, bool> nkFadersChangedMap;

    std::vector<int> nkLEDs;         // current commanded LED states
    std::map<int, int> nkBlinkMask;  // LED index -> 0/1 for blink off/on

    // robot state (actual robot state)
    std::map<std::string, int> jointIndexMap;  // joint name -> array index
    std::vector<std::string> jointNames;       // lwr_arm_0_joint, ...
    std::vector<double> jointAngles;           // current robot joint angles, radians
    std::vector<double> jointVelocities;       // radians per second
    std::vector<double> jointAccelerations;    // radians per second per second
    std::vector<double> jointTorques;          // Newton-meters

    // target values (commanded robot state)
    std::vector<double> referenceAngles;      // set on mode-changes
    std::vector<double> targetAngles;         // where the robot should be
    std::vector<double> targetVelocities;     // rad/sec
    std::vector<double> targetAccelerations;  // unused (so far)
    std::vector<double> zeroVelocities;       // rad/sec

    // motion constraints and robot limits
    std::vector<double> lowerJointLimits;         // rad
    std::vector<double> upperJointLimits;         // rad
    std::vector<double> jointVelocityLimits;      // rad/sec
    std::vector<double> jointAccelerationLimits;  // rad/sec^2
    std::vector<double> jointJerkLimits;          // rad/sec^3

    // ROS stuff
    ros::Subscriber jointStatesSubscriber;  // sensor_msgs::JointState
    ros::Subscriber nanokontrolSubscriber;  // sensor_msgs::Joy

    ros::Publisher emergencyStopPublisher;
    ros::Publisher stopRobotPublisher;
    ros::Publisher cancelTrajectoryPublisher;
    ros::Publisher positionGoalPublisher;  // joint state
    ros::Publisher velocityGoalPublisher;
    ros::Publisher rmlPositionGoalPublisher;  // RML messages
    ros::Publisher rmlVelocityGoalPublisher;
    ros::Publisher ledPublisher;  // visual feedback on the NK2

    bool haveWSG50;
    ros::ServiceClient moveWSG50Client;

    tf::TransformListener* tfl;
    tf::TransformBroadcaster* tbr;

    Eigen::Isometry3d world_base_transform;  // world -> lwr_arm_base_link
    Eigen::Isometry3d tool_transform;        // lwr_arm_7_link -> tcp

    robot_state::RobotStatePtr robot_state;
    robot_model::RobotModelPtr kinematic_model;
    robot_model_loader::RobotModelLoader robot_model_loader;
    const robot_state::JointModelGroup* joint_model_group;

    sensor_msgs::Joy currentNanokontrolMessage;     // current MIDI/joy inputs
    sensor_msgs::Joy lastNanokontrolMessage;        // previous MIDI/joy inputs
    sensor_msgs::JointState lastJointStateMessage;  // received robot state

    ros::Timer scheduler;        // main logic of this class
    ros::Timer blinkTimer;       // just for blinking LEDs
    ros::Duration robotTimeout;  // ROBOT_ERROR when joint_states dry out

    long n_scheduler_callbacks;  // counters, mostly for debugging
    long n_blink_callbacks;
    long n_lwr_callbacks;
    long n_nk_callbacks;
    long nk_teleop_seq;
};

/**
 * checks and clamps the given value against the given lower and upper limits.
 */
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
 * utility method to parse seven double values from a given param string.
 */
void parseInitializationValues(std::vector<double>& data, std::string s)
{
    assert(NUMBER_OF_JOINTS == 7);
    double a, b, c, d, e, f, g;
    int n = sscanf(s.c_str(), "%lf %lf %lf %lf %lf %lf %lf", &a, &b, &c, &d, &e, &f, &g);
    if (n != NUMBER_OF_JOINTS)
    {
        ROS_ERROR("Failed to parse initialization data '%s', found %d tokens.", s.c_str(), n);
        exit(1);
    }
    data.resize(NUMBER_OF_JOINTS);
    data[0] = a;
    data[1] = b;
    data[2] = c;
    data[3] = d;
    data[4] = e;
    data[5] = f;
    data[6] = g;
}

/**
 * constructor. Reads the node params and initializes everything.
 * Initial state is STOP.
 */
lwr_nk_teleop::lwr_nk_teleop()
{
    // node state and mutex
    //
    int s = pthread_mutex_init(&nodeStateMutex, NULL);
    if (s != 0)
    {
        ROS_ERROR("pthread_mutex_init failed.");
        exit(1);
    }

    // get parameters
    ros::NodeHandle nnh("~");  // private node handle, private namespace

    nnh.param("verbose", verbose, true);
    ROS_INFO("verbose mode is %d", verbose);

    std::string tf_prefix;
    nnh.param("tf_prefix", tf_prefix, std::string(""));
    ROS_INFO("tf_prefix is '%s'", tf_prefix.c_str());

    double robotTimeout;
    nnh.param("robot_timeout", robotTimeout, 1.0);
    ROS_INFO("robot timeout is %8.4lf seconds", robotTimeout);
    // ...

    std::string robot_description;
    nnh.param("robot_description", robot_description, std::string("robot_description"));
    ROS_INFO("robot description name is '%s'", robot_description.c_str());

    // those are the "standard" joint-names for the Kuka
    jointNames.push_back(tf_prefix + "lwr_arm_0_joint");
    jointNames.push_back(tf_prefix + "lwr_arm_1_joint");
    jointNames.push_back(tf_prefix + "lwr_arm_2_joint");
    jointNames.push_back(tf_prefix + "lwr_arm_3_joint");
    jointNames.push_back(tf_prefix + "lwr_arm_4_joint");
    jointNames.push_back(tf_prefix + "lwr_arm_5_joint");
    jointNames.push_back(tf_prefix + "lwr_arm_6_joint");
    for (unsigned int j = 0; j < jointNames.size(); j++)
    {
        jointIndexMap[jointNames[j]] = j;
    }

    jointAngles.resize(NUMBER_OF_JOINTS);
    jointVelocities.resize(NUMBER_OF_JOINTS);
    jointTorques.resize(NUMBER_OF_JOINTS);
    jointAccelerations.resize(NUMBER_OF_JOINTS);

    lowerJointLimits.resize(NUMBER_OF_JOINTS);
    upperJointLimits.resize(NUMBER_OF_JOINTS);
    jointVelocityLimits.resize(NUMBER_OF_JOINTS);
    jointAccelerationLimits.resize(NUMBER_OF_JOINTS);
    jointJerkLimits.resize(NUMBER_OF_JOINTS);

    referenceAngles.resize(NUMBER_OF_JOINTS);  // updated on mode changes
    targetAngles.resize(NUMBER_OF_JOINTS);     // latest robot target position
    targetVelocities.resize(NUMBER_OF_JOINTS);
    targetAccelerations.resize(NUMBER_OF_JOINTS);
    zeroVelocities.resize(NUMBER_OF_JOINTS);
    for (int j = 0; j < NUMBER_OF_JOINTS; j++)
        zeroVelocities[j] = 0.0;

    // default limits are for wallmount PA-10 with Shadow-hand @ TAMS
    //
    std::string lower_radians, upper_radians;
    std::string velocity_limits, acceleration_limits, jerk_limits;
    nnh.param("lower_joint_limits_radians", lower_radians,  // -170 -120 -170 -120 -170 -120 -170
              std::string("-2.9 -2.0 -2.9 -2.0 -2.9 -2.0 -2.9"));
    parseInitializationValues(lowerJointLimits, lower_radians);

    nnh.param("upper_joint_limits_radians", upper_radians,  // 170 120 170 120 170 120 170
              std::string("2.9 2.0 2.9 2.0 2.9 2.0 2.9"));
    parseInitializationValues(upperJointLimits, upper_radians);

    nnh.param("velocity_limits_radians", velocity_limits,  // 55 deg/sec = 0.9599 rad/sec
              std::string("1.9 1.9 2.25 2.25 2.25 3.14 3.14"));
    parseInitializationValues(jointVelocityLimits, velocity_limits);

    nnh.param("acceleration_limits_radians", acceleration_limits,  // 50 deg/sec^2 = 0.87 rad/sec/sec
              std::string("0.3 0.3 0.5 0.5 0.5 0.5 0.5"));
    parseInitializationValues(jointAccelerationLimits, acceleration_limits);

    nnh.param("jerk_limits_radians", jerk_limits,  // 100 deg/sec^3 = 0.18 rad/sec/sec/sec
              std::string("0.1 0.1 0.1 0.1 0.1 0.1 0.1"));
    parseInitializationValues(jointJerkLimits, jerk_limits);
    ROS_INFO("joint position, velocity, acceleration limits initialized.");

    ROS_WARN("lwr_nk_teleop: robot limits after node initialization:");
    ROS_WARN("   limits   lower : upper   velocity  acceleration  jerk:");
    for (int j = 0; j < NUMBER_OF_JOINTS; j++)
    {
        ROS_WARN("   %.1d  %8.4lf : %8.4lf  %8.4lf  %8.4lf  %8.4lf", j, lowerJointLimits[j], upperJointLimits[j],
                 jointVelocityLimits[j], jointAccelerationLimits[j], jointJerkLimits[j]);
    }

    // nanokontrol stuff
    //
    nkButtons.resize(NUMBER_NK_BUTTONS);
    nkKnobs.resize(NUMBER_NK_CHANNELS);
    nkKnobsTracked.resize(NUMBER_NK_CHANNELS);
    nkFaders.resize(NUMBER_NK_CHANNELS);
    nkFadersTracked.resize(NUMBER_NK_CHANNELS);

    nkLEDs.resize(NUMBER_NK_BUTTONS);
    nkBlinkMask.clear();

    for (int ch = 0; ch < NUMBER_NK_CHANNELS; ch++)
    {
        nkKnobs[ch] = 0.0;
        nkKnobsTracked[ch] = false;
        nkFaders[ch] = 0.0;
        nkFadersTracked[ch] = false;
    }

    nkStateButtonsMap.clear();
    nkStateButtonsMap[NK_REWIND] = 1;
    nkStateButtonsMap[NK_FORWARD] = 1;
    nkStateButtonsMap[NK_STOP] = 1;
    nkStateButtonsMap[NK_PLAY] = 1;
    nkStateButtonsMap[NK_DOT] = 1;

    nkStateButtonsChangedMap.clear();
    nkButtonsChangedMap.clear();
    nkKnobsChangedMap.clear();
    nkFadersChangedMap.clear();
    nkStateButtonsChangedMap.clear();

    // MoveIt robot model loader
    // robot_model_loader::RobotModelLoader robot_model_loader( "robot_description" );
    // robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
    // robot_model_loader = new robot_model_loader::RobotModelLoader( robot_description );
    ROS_INFO("MoveIt: creating the RobotModelLoader now...");
    robot_model_loader = robot_model_loader::RobotModelLoader(robot_description);
    kinematic_model = robot_model_loader.getModel();
    ROS_INFO("MoveIt: model frame: %s", kinematic_model->getModelFrame().c_str());

    robot_state.reset(new robot_state::RobotState(kinematic_model));
    robot_state->setToDefaultValues();
    joint_model_group = kinematic_model->getJointModelGroup("arm");
    ROS_INFO("MoveIt: got the kinematic_state...");

    std::vector<double> joint_values;
    robot_state->copyJointGroupPositions(joint_model_group, joint_values);
    for (std::size_t i = 0; i < jointNames.size(); ++i)
    {
        ROS_INFO("Joint %s: %f", jointNames[i].c_str(), joint_values[i]);
    }

    world_base_transform = robot_state->getGlobalLinkTransform("lwr_arm_base_link");
    ROS_INFO_STREAM("lwr_arm_base_link translation: " << world_base_transform.translation());
    ROS_INFO_STREAM("lwr_arm_base_link Rotation: " << world_base_transform.rotation());

    tool_transform =
        robot_state->getGlobalLinkTransform("lwr_arm_7_link").inverse() * robot_state->getGlobalLinkTransform("tcp");
    ROS_INFO_STREAM("tool_transformation  translation: " << tool_transform.translation());
    // ROS_ERROR( "tip name is '%d'", robot_state->getTipName() );

    // ROS publishers and subscribers
    //
    emergencyStopPublisher = nnh.advertise<std_msgs::Bool>("emergency_stop", 1);
    stopRobotPublisher = nnh.advertise<std_msgs::Bool>("stop_robot", 1);
    cancelTrajectoryPublisher = nnh.advertise<sensor_msgs::JointState>("cancel", 1);
    positionGoalPublisher = nnh.advertise<sensor_msgs::JointState>("joint_position_goal", 1);
    velocityGoalPublisher = nnh.advertise<sensor_msgs::JointState>("joint_velocity_goal", 1);

    // extra topics for RML-style commands
    rmlPositionGoalPublisher = nnh.advertise<ros_fri_msgs::RMLPositionInputParameters>("jointPositionGoal", 1);
    rmlVelocityGoalPublisher = nnh.advertise<ros_fri_msgs::RMLVelocityInputParameters>("jointVelocityGoal", 1);

    ledPublisher = nnh.advertise<sensor_msgs::Joy>("leds", 1);

    jointStatesSubscriber =
        nnh.subscribe<sensor_msgs::JointState>("/lwr/joint_states", 1, &lwr_nk_teleop::jointStateCallback, this);

    nanokontrolSubscriber = nnh.subscribe<sensor_msgs::Joy>("joy", 1, &lwr_nk_teleop::nanokontrolCallback, this);

    // Schunk WSG-50 gripper client
    haveWSG50 = ros::service::waitForService("/wsg_50/move", ros::Duration(10));  // move?
    if (haveWSG50)
    {
        moveWSG50Client = nh.serviceClient<wsg_50_common::Move>("wsg_50/move");  // move?
        ROS_ERROR("Gripper move client created.");
    }

    setNodeState(WAITING_FOR_ROBOT);
    nk_teleop_seq = 0;  // incremented on every robot publication
    nkBlinkMask[NK_CYCLE] = 1;

    setIKMode(TCP_IK_MODE);

    // scheduler and blink timer
    //
    n_scheduler_callbacks = 0;
    n_blink_callbacks = 0;
    n_lwr_callbacks = 0;
    n_nk_callbacks = 0;

    // now we wait for the first incoming JointState message: does not work...
    // boost::shared_ptr<sensor_msgs::JointState const> sharedPtr =
    //  ros::topic::waitForMessage<sensor_msgs::JointState>( "joint_states", ros::Duration(10));
    // ROS_INFO( "NK teleop: received the first joint_states, starting now..." );

    // a single transformlistener and broadcaster
    //
    try
    {
        tfl = new tf::TransformListener(nh, ros::Duration(10));
        ROS_INFO("got a transform listener.");

        tbr = new tf::TransformBroadcaster();
        ROS_INFO("created a transform broadcaster.");
    }
    catch (tf::TransformException& exception)
    {
        ROS_ERROR("TransformListener failed: %s", exception.what());
        exit(1);
    }

    scheduler = nnh.createTimer(ros::Duration(0.01), &lwr_nk_teleop::schedulerCallback, this);
    blinkTimer = nnh.createTimer(ros::Duration(0.5), &lwr_nk_teleop::blinkTimerCallback, this);
}  // end constructor

/**
 * descructor: currently empty.
 */
lwr_nk_teleop::~lwr_nk_teleop()
{
    ROS_ERROR("xxx: IMPLEMENT ME!");
}

/**
 * callback for incoming joint states from the robot (real or simulated).
 * We check the message and extract only those joint values that we need.
 */
void lwr_nk_teleop::jointStateCallback(const sensor_msgs::JointState jointState)
{
    n_lwr_callbacks++;
    if (verbose && n_lwr_callbacks < 5)
    {
        ROS_INFO("NK teleop: received lwr joint state %ld", n_lwr_callbacks);
    }

    pthread_mutex_lock(&nodeStateMutex);

    unsigned int matched_lwr_joints = 0;
    for (unsigned int i = 0; i < jointState.name.size(); i++)
    {
        std::string name = jointState.name[i];
        int found = jointIndexMap.count(name);
        if (found > 0)
        {
            int lwrIndex = jointIndexMap[name];
            jointAngles[lwrIndex] = jointState.position[i];
            jointVelocities[lwrIndex] = jointState.velocity[i];
            jointTorques[lwrIndex] = jointState.effort[i];
            matched_lwr_joints++;
        }
        else if (verbose)
        {
            ROS_INFO("NK teleop: unmatched joint name '%s'", name.c_str());
        }
    }

    if (matched_lwr_joints == NUMBER_OF_JOINTS)
    {
        lastJointStateMessage = jointState;
    }
    else
    {
        ROS_ERROR("NK teleop: lwr joint state invalid/incomplete,");
        ROS_ERROR("NK teleop: ... only matched %d LWR joints.", matched_lwr_joints);
    }
    pthread_mutex_unlock(&nodeStateMutex);
}

/**
 * callback for incoming MIDI control messages. Every message has the
 * full set of Joy buttons and axes values, so we first calculate the
 * diff from the previous message to find the changes.
 * If there is a state-change (one of the rev/fwd/stop/play/dot buttons),
 * this is handled first. If not, the other button, knob, and slider
 * events are processed.
 */
void lwr_nk_teleop::nanokontrolCallback(const sensor_msgs::Joy msg)
{
    // if (verbose) ROS_INFO( "NK teleop: received nanokontrol callback (Joy message)" );
    n_nk_callbacks++;

    // plausibility check to avoid accidental other publishers
    if ((msg.buttons.size() != NUMBER_NK_BUTTONS) || (msg.axes.size() != NUMBER_NK_AXES))
    {
        ROS_ERROR("NK teleop: Joy message with wrong number of buttons (%ld vs %d) or axes (%ld vs %d)",
                  msg.buttons.size(), NUMBER_NK_BUTTONS, msg.axes.size(), NUMBER_NK_AXES);
        return;
    }

    pthread_mutex_lock(&nodeStateMutex);
    lastNanokontrolMessage = currentNanokontrolMessage;
    currentNanokontrolMessage = msg;
    pthread_mutex_unlock(&nodeStateMutex);

    calculateNanokontrolMessageDiff(currentNanokontrolMessage, lastNanokontrolMessage);
    if (hasModeChanges())
    {
        processModeChangeEvents();
    }

    if (hasKnobsChanges())
        processKnobEvents();

    /*
      else if (hasButtonChanges()) {
         processButtonEvents();
      }
      else {
         if (hasKnobsChanges()) processKnobEvents();
         if (hasFadersChanges()) processFaderEvents();
      }
      publishVelocityGoal();
    */
}

/**
 * callback for the LED blink timer. We just increment the callback
 * count; the real work is later done by publishNKLEDs().
 */
void lwr_nk_teleop::blinkTimerCallback(const ros::TimerEvent& wakeupMsg)
{
    n_blink_callbacks++;
    if (nkBlinkMask.size() > 0)
    {
        publishNKLEDs();
    }
}

/**
 * callback for the scheduler timer.
 * If robot joint-state messages are too old, we switch to the ERROR state
 * and also send a robot stop and cancel-trajectory. We blink the STOP
 * button to indicate the situation.
 * If MIDI messages are too old, we simply switch to the STOP state.
 */
void lwr_nk_teleop::schedulerCallback(const ros::TimerEvent& wakeupMsg)
{
    n_scheduler_callbacks++;
    if (verbose && (n_scheduler_callbacks < 5))
    {
        ROS_INFO("NK teleop: scheduler callback at %d, %ld", wakeupMsg.current_real.sec, n_scheduler_callbacks);
    }

    ros::Time t0 = ros::Time::now();

    ros::Duration dt_lwr = t0 - lastJointStateMessage.header.stamp;
    if (dt_lwr.toSec() > ROBOT_TIMEOUT_SECONDS)
    {
        if (nodeState == WAITING_FOR_ROBOT)
        {
            return;  // try again later
        }
        if (nodeState != ROBOT_ERROR)
        {
            ROS_ERROR("NK teleop: did not receive joint_states for %8.4lf seconds,", dt_lwr.toSec());
            ROS_ERROR("NK teleop: stopping the robot and switching to ERROR state.");
            setNodeState(ROBOT_ERROR);  // also stops the robot, blinks
        }
    }

    ros::Duration dt_nk = t0 - lastNanokontrolMessage.header.stamp;
    if (dt_nk.toSec() > INACTIVE_TIMEOUT_SECONDS)
    {
        if (nodeState == ROBOT_ERROR)
            ;
        else if (nodeState != STOP)
        {
            ROS_ERROR("NK teleop: no user event for %8.4lf seconds, switching to STOP state.", dt_nk.toSec());
            setNodeState(STOP);  // also stops the robot
        }
    }

    // xxxzzz: IMPLEMENT
    if (hasButtonChanges())
    {
        processButtonEvents();
    }
    else
    {
        if (hasKnobsChanges())
            processKnobEvents();
        if (hasFadersChanges())
            processFaderEvents();
    }
}

/**
 * Every incoming Joy message has the full set of buttons and axes values.
 * This method compares the received message with the previous message
 * to find any changes; then builds the corresponding bit masks for the
 * later event handling.
 */
void lwr_nk_teleop::calculateNanokontrolMessageDiff(sensor_msgs::Joy msg, sensor_msgs::Joy lastNanokontrolMessage)
{
    if (n_nk_callbacks == 1)
    {  // the first call, lastNanokontrolMessage still empty
        ROS_INFO("NK teleop: calculate message diff: skipping on first message.");
        return;
    }

    pthread_mutex_lock(&nodeStateMutex);
    nkButtonsChangedMap.clear();
    nkStateButtonsChangedMap.clear();
    for (int i = 0; i < NUMBER_NK_BUTTONS; i++)
    {
        if (msg.buttons[i] != lastNanokontrolMessage.buttons[i])
        {
            if (nkStateButtonsMap[i] == 1)
            {
                nkStateButtonsChangedMap[i] = 1;
            }
            else
            {
                nkButtonsChangedMap[i] = 1;
            }
        }
    }

    nkKnobsChangedMap.clear();
    // ROS_INFO( "XXXX nkKnobsChangedMap.clear.... size=%lu", nkKnobsChangedMap.size() );
    for (int i = 0; i < NUMBER_NK_CHANNELS; i++)
    {  //
        if (msg.axes[i + NUMBER_NK_CHANNELS] != lastNanokontrolMessage.axes[i + NUMBER_NK_CHANNELS])
        {
            nkKnobs[i] = msg.axes[i + NUMBER_NK_CHANNELS];
            nkKnobsChangedMap[i] = 1;
            ROS_INFO("cNMDiff: knobs changed %d %8.3lf %8.3lf", i, msg.axes[i + NUMBER_NK_CHANNELS],
                     lastNanokontrolMessage.axes[i + NUMBER_NK_CHANNELS]);
        }
    }
    // ROS_INFO( "XXXX nkKnobsChangedMap.loop.... size=%lu", nkKnobsChangedMap.size() );

    nkFadersChangedMap.clear();
    for (int i = 0; i < NUMBER_NK_CHANNELS; i++)
    {  //
        if (msg.axes[i] != lastNanokontrolMessage.axes[i])
        {
            nkFaders[i] = msg.axes[i];
            nkFadersChangedMap[i] = 1;
        }
    }
    pthread_mutex_unlock(&nodeStateMutex);
}

bool lwr_nk_teleop::hasModeChanges()
{
    pthread_mutex_lock(&nodeStateMutex);
    bool b = nkStateButtonsChangedMap.size() > 0;
    pthread_mutex_unlock(&nodeStateMutex);
    return b;
}

bool lwr_nk_teleop::hasButtonChanges()
{
    pthread_mutex_lock(&nodeStateMutex);
    bool b = nkButtonsChangedMap.size() > 0;
    pthread_mutex_unlock(&nodeStateMutex);
    return b;
}

bool lwr_nk_teleop::hasKnobsChanges()
{
    pthread_mutex_lock(&nodeStateMutex);
    bool b = nkKnobsChangedMap.size() > 0;
    // ROS_INFO( "XXXX hasKnobsChanges: %lu", nkKnobsChangedMap.size() );
    pthread_mutex_unlock(&nodeStateMutex);
    return b;
}

bool lwr_nk_teleop::hasFadersChanges()
{
    pthread_mutex_lock(&nodeStateMutex);
    bool b = nkFadersChangedMap.size() > 0;
    pthread_mutex_unlock(&nodeStateMutex);
    return b;
}

/**
 * one of the mode change buttons (transport section buttons)
 * was pressed or released. Calculate the new teleop mode
 * and command the robot as necessary.
 */
void lwr_nk_teleop::processModeChangeEvents()
{
    // We have just five mode buttons, so we can create a bitmap first,
    // and then work on that...
    int modeMask = (currentNanokontrolMessage.buttons[NK_REWIND] << 4) +
                   (currentNanokontrolMessage.buttons[NK_FORWARD] << 3) +
                   (currentNanokontrolMessage.buttons[NK_STOP] << 2) +
                   (currentNanokontrolMessage.buttons[NK_PLAY] << 1) + (currentNanokontrolMessage.buttons[NK_DOT] << 0);
    if (verbose)
        ROS_INFO("NK teleop: process mode-change: mode bitmask is %d", modeMask);

    // stop wins
    if (currentNanokontrolMessage.buttons[NK_STOP] == 1)
    {  // STOP state wins...
        setNodeState(STOP);
    }
    else if (currentNanokontrolMessage.buttons[NK_FORWARD] == 1)
    {  // joint-motion
        setNodeState(JOINTS_MOTION_MODE);
    }
    else if (currentNanokontrolMessage.buttons[NK_PLAY] == 1)
    {  // xyz-cartesian
        setNodeState(XYZ_CARTESIAN_MODE);
    }
    else if (currentNanokontrolMessage.buttons[NK_DOT] == 1)
    {  // NOA-cartesian
        setNodeState(NOA_CARTESIAN_MODE);
    }
    else if (currentNanokontrolMessage.buttons[NK_REWIND] == 1)
    {  // undefined so far
        setNodeState(STOP);
    }
    else
    {  // nothing pressed, so button released: keep current state
        return;
    }

    // iterate through change map...
    // std::map<int,bool>::iterator it;
    // std::stringstream ss;
    // for( it=nkStateButtonsChangedMap.begin(); it != nkStateButtonsChangedMap.end(); ++it ) {
    // ss << it-> first << " => " << it->second << "\n";
    // }
    // ROS_INFO( "state button changes:\n%s", ss.str().c_str() );
}

void lwr_nk_teleop::setIKMode(nk_wrist_or_tcp_ik nextMode)
{
    if (verbose)
        ROS_INFO("setIKMode: request mode = %d", nextMode);
    ikMode = nextMode;

    nkLEDs[NK_PREV_TRACK] = (ikMode == WRIST_IK_MODE) ? 1 : 0;
    nkLEDs[NK_NEXT_TRACK] = (ikMode == TCP_IK_MODE) ? 1 : 0;

    ROS_INFO("setIKMode: ik mode is now %s", (ikMode == WRIST_IK_MODE) ? "WRIST_IK_MODE (T6)" : "TCP_IK_MODE (tool)");
}

void lwr_nk_teleop::setNodeState(nk_teleop_state nextState)
{
    if (verbose)
        ROS_INFO("setNodeState: requested state = %d", nextState);

    pthread_mutex_lock(&nodeStateMutex);
    if (nextState == ROBOT_ERROR)
    {  // this needs to be processed quickly anyway
        ROS_WARN("NK teleop: switching to state ROBOT_ERROR!!!");
        publishCancelTrajectory();
        publishEmergencyStop(true);
        nodeState = ROBOT_ERROR;
        nkBlinkMask[NK_STOP] = 1;
    }
    else if (nodeState == ROBOT_ERROR)
    {  // can we recover?
        if (nextState == STOP)
        {  // trying to recover via STOP state
            nodeState = STOP;
            ROS_WARN("NK teleop: trying to exit ROBOT_ERROR state to STOP state...");
            publishEmergencyStop(false);
        }
        else
        {  // but this is forbidden
            ROS_ERROR("NK teleop: cannot go from ROBOT_ERROR to state %d, ignored.", nextState);
        }
    }
    else if (nodeState == WAITING_FOR_ROBOT)
    {
        if (n_lwr_callbacks == 0)
        {
            ROS_ERROR("NK teleop: still waiting for robot joint_states, cannot change mode.");
            return;
        }
    }

    if ((nextState != nodeState) && (n_lwr_callbacks > 0))
    {
        ROS_ERROR("NK teleop: updating targetAngles from current robot joint_states...");
        for (int j = 0; j < NUMBER_OF_JOINTS; j++)
        {
            targetAngles[j] = jointAngles[j];  // FIXME: do we really want this? any position errors are "accepted" here
            referenceAngles[j] = jointAngles[j];
        }
        ROS_ERROR("NK teleop: updated target %8.4lf %8.4lf %8.4lf %8.4lf %8.4lf %8.4lf %8.4lf", targetAngles[0],
                  targetAngles[1], targetAngles[2], targetAngles[3], targetAngles[4], targetAngles[5], targetAngles[6]);
    }

    // finally, accept requested state
    nodeState = nextState;
    pthread_mutex_unlock(&nodeStateMutex);

    // after all changes, stop the robot, update the mode LEDs
    updateNodeStateLEDs();
    publishNKLEDs();
}

void lwr_nk_teleop::updateNodeStateLEDs()
{
    nkLEDs[NK_REWIND] = 0;
    nkLEDs[NK_FORWARD] = (nodeState == JOINTS_MOTION_MODE) ? 1 : 0;
    nkLEDs[NK_STOP] = ((nodeState == STOP) || (nodeState == ROBOT_ERROR)) ? 1 : 0;
    nkLEDs[NK_PLAY] = (nodeState == XYZ_CARTESIAN_MODE) ? 1 : 0;
    nkLEDs[NK_DOT] = (nodeState == NOA_CARTESIAN_MODE) ? 1 : 0;
}

/**
 * react to button release or press events on the channel
 * strip buttons (record, mute, solo). Actual function
 * depends on the current mode.
 */
void lwr_nk_teleop::processButtonEvents()
{
    // if (verbose) ROS_INFO( "NK teleop: processButtonEvents..." );
    std::vector<int> buttons = currentNanokontrolMessage.buttons;

    if ((nodeState == STOP) || (nodeState == ROBOT_ERROR))
    {
        return;  // do nothing, ignore other buttons and knobs etc
    }

    // check the track button(s)
    if ((nkButtonsChangedMap[NK_PREV_TRACK] == 1) || (nkButtonsChangedMap[NK_NEXT_TRACK] == 1))
    {
        if (buttons[NK_PREV_TRACK] == 1)
            setIKMode(WRIST_IK_MODE);
        if (buttons[NK_NEXT_TRACK] == 1)
            setIKMode(TCP_IK_MODE);
    }

    if (nodeState == JOINTS_MOTION_MODE)
    {
        processJointsMotionButtonEvents();
    }
    else if (nodeState == XYZ_CARTESIAN_MODE)
    {
        processCartesianXyzButtonEvents();
    }
    else if (nodeState == NOA_CARTESIAN_MODE)
    {
        processCartesianNoaButtonEvents();
    }
    else
    {
        ROS_ERROR("NK teleop: processButtonEvents/unknown mode: should not happen, ignored!");
    }
}

/**
 * in joints mode, we do the following on button press events
 * on the channel buttons, where channel <x>=0..7:
 * R<x>: command negative joint velocity ("decrease angle") ("record button")
 * M<x>: command positive joint velocity ("increase angle") ("mute button")
 * <REW>+R<x>, <REW>+<M<x>: faster negative/positive joint velocity ("shift")
 * S<x>: ignore
 */
void lwr_nk_teleop::processJointsMotionButtonEvents()
{
    std::vector<int> buttons = currentNanokontrolMessage.buttons;
    // SOLO buttons index 0..7, MUTE 8..15, RECORD 16..23

    double DELTA_T = 0.01;  // FIXME: don't hardcode
    double factor;
    bool hasUpdates = false;

    // first, check "global" buttons:
    if (buttons[NK_SOLO_7] == 1)
    {  // SOLO_7 means STOP NOW
        ROS_ERROR("STOPPING now, due to NK_SOLO_7 pressed...");
        pthread_mutex_lock(&nodeStateMutex);
        for (int j = 0; j < NUMBER_OF_JOINTS; j++)
        {
            targetAngles[j] = jointAngles[j];
            targetVelocities[j] = 0.0;
        }
        pthread_mutex_unlock(&nodeStateMutex);
        publishPositionGoal();
        setNodeState(STOP);
        nkLEDs[NK_SOLO_7] = 1;
        publishNKLEDs();
        return;
    }
    else
    {
        nkLEDs[NK_SOLO_7] = 0;
    }

    if ((buttons[NK_FORWARD] == 1) && (buttons[NK_SOLO_6] == 1))
    {  // "candle position"
        for (int j = 0; j < NUMBER_OF_JOINTS; j++)
        {
            targetAngles[j] = 0.0;
        }
        publishPositionGoal();
        nkLEDs[NK_SOLO_6] = 1;
        publishNKLEDs();
        return;
    }
    else
    {
        nkLEDs[NK_SOLO_6] = 0;
    }

    if ((buttons[NK_FORWARD] == 1) && (buttons[NK_MUTE_6] == 1))
    {  // "retracted" position
        // 0.0000   0.3501   0.0000   0.7590   0.0027   0.4275   0.0000)
        targetAngles[0] = 0.000;
        targetAngles[1] = 0.350;
        targetAngles[2] = 0.000;
        targetAngles[3] = 0.700;
        targetAngles[4] = 0.000;
        targetAngles[5] = 0.350;
        targetAngles[6] = 0.000;
        publishPositionGoal();
        nkLEDs[NK_MUTE_6] = 1;
        publishNKLEDs();
        return;
    }
    else
    {
        nkLEDs[NK_MUTE_6] = 0;
    }

    if ((buttons[NK_FORWARD] == 1) && (buttons[NK_RECORD_6] == 1))
    {  // "all-0.2 candle"
        targetAngles[0] = 0.2;
        targetAngles[1] = 0.2;
        targetAngles[2] = 0.2;
        targetAngles[3] = 0.2;
        targetAngles[4] = 0.2;
        targetAngles[5] = 0.2;
        targetAngles[6] = 0.2;
        publishPositionGoal();
        nkLEDs[NK_RECORD_6] = 1;
        publishNKLEDs();
        return;
    }
    else
    {
        nkLEDs[NK_RECORD_6] = 0;
    }

    if ((buttons[NK_FORWARD] == 1) && (buttons[NK_SOLO_5] == 1))
    {  // arm towards +x, hand down
        // 1.8506   0.5832  -1.7437  -1.6396   0.5599   1.5565   0.0999)
        targetAngles[0] = 1.8506;
        targetAngles[1] = 0.5832;
        targetAngles[2] = -1.7437;
        targetAngles[3] = -1.6396;
        targetAngles[4] = 0.5599;
        targetAngles[5] = 1.5565;
        targetAngles[6] = 0.0999;
        nkLEDs[NK_SOLO_5] = 1;
        publishNKLEDs();
        return;
    }
    else
    {
        nkLEDs[NK_SOLO_5] = 0;
    }

    if ((buttons[NK_FORWARD] == 1) && (buttons[NK_MUTE_5] == 1))
    {  // arm towards +x, hand towards +y
        //  1.8506   0.5832  -1.7437  -1.6396   0.5599   1.5565   0.0999)
        targetAngles[0] = 1.8506;
        targetAngles[1] = 0.5832;
        targetAngles[2] = -1.7437;
        targetAngles[3] = -1.6396;
        targetAngles[4] = 0.5599;
        targetAngles[5] = 1.5565;
        targetAngles[6] = 0.0999;
        nkLEDs[NK_MUTE_5] = 1;
        publishNKLEDs();
        return;
    }
    else
    {
        nkLEDs[NK_MUTE_5] = 0;
    }

    if ((buttons[NK_FORWARD] == 1) && (buttons[NK_RECORD_5] == 1))
    {  // hand towards +x, +x
        //  1.8506   0.5832  -1.7437  -1.6396   0.4774   0.0101   0.0998
        targetAngles[0] = 1.8506;
        targetAngles[1] = 0.5832;
        targetAngles[2] = -1.7437;
        targetAngles[3] = -1.6396;
        targetAngles[4] = 0.4774;
        targetAngles[5] = 0.0101;
        targetAngles[6] = 0.0998;
        nkLEDs[NK_RECORD_5] = 1;
        publishNKLEDs();
        return;
    }
    else
    {
        nkLEDs[NK_RECORD_5] = 0;
    }

    if ((buttons[NK_FORWARD] == 1) && (buttons[NK_SOLO_4] == 1))
    {  // arm towards +x, hand down, +x
        // -0.4340   0.1475   0.6903  -1.5325  -0.0811   1.5015   0.1620)
        targetAngles[0] = -0.4340;
        targetAngles[1] = 0.1475;
        targetAngles[2] = 0.6903;
        targetAngles[3] = -1.5325;
        targetAngles[4] = -0.0811;
        targetAngles[5] = 1.5015;
        targetAngles[6] = 0.1620;
        nkLEDs[NK_SOLO_4] = 1;
        publishNKLEDs();
        return;
    }
    else
    {
        nkLEDs[NK_SOLO_4] = 0;
    }

    if ((buttons[NK_FORWARD] == 1) && (buttons[NK_MUTE_4] == 1))
    {  // arm towards +x, hand towards +y
        //  -0.4340   0.1475   0.6903  -1.5325   1.5200   1.4085  -0.0456)
        targetAngles[0] = -0.4340;
        targetAngles[1] = 0.1475;
        targetAngles[2] = 0.6903;
        targetAngles[3] = -1.5325;
        targetAngles[4] = 1.5200;
        targetAngles[5] = 1.4085;
        targetAngles[6] = -0.0456;
        nkLEDs[NK_MUTE_4] = 1;
        publishNKLEDs();
        return;
    }
    else
    {
        nkLEDs[NK_MUTE_4] = 0;
    }

    if ((buttons[NK_FORWARD] == 1) && (buttons[NK_RECORD_4] == 1))
    {  // arm towards +x, hand towards +x
       //  -0.4340   0.1475   0.6903  -1.4605   1.5464  -0.1516  -0.0455)
        targetAngles[0] = -0.4340;
        targetAngles[1] = 0.1475;
        targetAngles[2] = 0.6903;
        targetAngles[3] = -1.4605;
        targetAngles[4] = 1.5464;
        targetAngles[5] = -0.1516;
        targetAngles[6] = -0.0455;
        nkLEDs[NK_RECORD_4] = 1;
        publishNKLEDs();
        return;
    }
    else
    {
        nkLEDs[NK_RECORD_4] = 0;
    }

    if ((buttons[NK_FORWARD] == 1) && (buttons[NK_SOLO_3] == 1))
    {  // arm towards +y, hands down, camera free
        //  0.5836   1.0427  -1.3282   1.2069   2.0015   1.9413   0.3645
        targetAngles[0] = 0.5836;
        targetAngles[1] = 1.0427;
        targetAngles[2] = -1.3282;
        targetAngles[3] = 1.2069;
        targetAngles[4] = 2.0015;
        targetAngles[5] = 1.9413;
        targetAngles[6] = 0.3645;
        nkLEDs[NK_SOLO_3] = 1;
        publishNKLEDs();
        return;
    }
    else
    {
        nkLEDs[NK_SOLO_3] = 0;
    }

    if ((buttons[NK_FORWARD] == 1) && (buttons[NK_MUTE_3] == 1))
    {  // arm towards +y, hands down, camera free
        // 1.2714   0.9650  -1.4834   1.1472   2.0934   1.8718   0.9513)
        targetAngles[0] = 1.2714;
        targetAngles[1] = 0.9650;
        targetAngles[2] = -1.4834;
        targetAngles[3] = 1.1472;
        targetAngles[4] = 2.0934;
        targetAngles[5] = 1.8718;
        targetAngles[6] = 0.9513;
        nkLEDs[NK_MUTE_3] = 1;
        publishNKLEDs();
        return;
    }
    else
    {
        nkLEDs[NK_MUTE_3] = 0;
    }

    if ((buttons[NK_FORWARD] == 1) && (buttons[NK_RECORD_3] == 1))
    {  // arm towards +y, hands 45 deg down, camera free
        //  0.2240   1.1337  -1.0681   1.5732   1.7323   1.2970   0.4090)
        targetAngles[0] = 0.2240;
        targetAngles[1] = 1.1337;
        targetAngles[2] = -1.0681;
        targetAngles[3] = 1.5732;
        targetAngles[4] = 1.7323;
        targetAngles[5] = 1.2970;
        targetAngles[6] = 0.4090;
        nkLEDs[NK_RECORD_3] = 1;
        publishNKLEDs();
        return;
    }
    else
    {
        nkLEDs[NK_RECORD_3] = 0;
    }

    if ((buttons[NK_FORWARD] == 1) && (buttons[NK_SOLO_2] == 1))
    {  // arm towards +y, hand towards Eugen, above camera
       // 0.6609   1.3706  -0.9313   1.3665  -0.2751  -1.1507   0.7311
        targetAngles[0] = 0.6609;
        targetAngles[1] = 1.3706;
        targetAngles[2] = -0.9313;
        targetAngles[3] = 1.3665;
        targetAngles[4] = -0.2751;
        targetAngles[5] = -1.1507;
        targetAngles[6] = 0.7311;
        nkLEDs[NK_SOLO_2] = 1;
        publishNKLEDs();
        return;
    }
    else
    {
        nkLEDs[NK_SOLO_2] = 0;
    }

    // tttzzz; xxxzzz;

    // now, the per-channel (0..6) buttons for per-joint control of joints S1..W2
    //
    for (int k = 0; k < NUMBER_OF_JOINTS; k++)
    {  // 8 channels but only 6 joints
        factor = 0.0;

        if ((buttons[16 + k] == 1) && (buttons[8 + k] == 0))
        {  // R but not M
            factor = (buttons[0 + k] == 1) ? -5 : -1;
            nkLEDs[16 + k] = 1;
            nkLEDs[8 + k] = 0;
            nkLEDs[0 + k] = buttons[0 + k];
        }
        else if ((buttons[16 + k] == 0) && (buttons[8 + k] == 1))
        {  // M but not R
            factor = (buttons[0 + k] == 1) ? 5 : 1;
            nkLEDs[16 + k] = 0;
            nkLEDs[8 + k] = 1;
            nkLEDs[0 + k] = buttons[0 + k];
        }
        else
        {  // no button or both: invalid
            factor = 0.0;
            nkLEDs[16 + k] = 0;
            nkLEDs[8 + k] = 0;
            nkLEDs[0 + k] = 0;
        }

        // now comes the control part. Just sending targetVelocities is not good,
        // as then the positions are undefined (or updated too late):
        // targetVelocities[k] = factor*JOINTS_MOTION_RAD_PER_SEC;
        //
        // Sending targets based on previous target + velocity * delta_t
        // results in overshooting when velocity is reduced to zero...
        //
        targetVelocities[k] = factor * JOINTS_MOTION_RAD_PER_SEC;
        double tmp_target = clamp(targetAngles[k] + DELTA_T * factor * JOINTS_MOTION_RAD_PER_SEC, lowerJointLimits[k],
                                  upperJointLimits[k]);
        if (tmp_target != targetAngles[k])
        {
            hasUpdates = true;
        }
        targetAngles[k] = tmp_target;

        // check joint angle limits: M buttons (upper limit)
        if (targetAngles[k] >= upperJointLimits[k])
        {
            nkBlinkMask[8 + k] = 1;
            nkLEDs[8 + k] = 1;
        }
        else
        {
            nkBlinkMask[8 + k] = 0;
        }

        if (targetAngles[k] <= lowerJointLimits[k])
        {
            nkBlinkMask[16 + k] = 1;
            nkLEDs[16 + k] = 1;
        }
        else
        {
            nkBlinkMask[16 + k] = 0;
        }
    }

    if (hasUpdates)
        ROS_INFO("Joints target is (%8.4lf %8.4lf %8.4lf %8.4lf %8.4lf %8.4lf %8.4lf)", targetAngles[0],
                 targetAngles[1], targetAngles[2], targetAngles[3], targetAngles[4], targetAngles[5], targetAngles[6]);

    publishPositionGoal();
    publishNKLEDs();
}

/**
 * process Cartesian NOA mode: cartesian motion and rotation in gripper
 * coordinate system. We use channels 0..2 for NOA, and channels 3..5
 * for rotation.
 * SOLO buttons index 0..7, MUTE 8..15, RECORD 16..23
 *
 * TODO: use one button (e.g. S6) for configuration switch?!
 * Knobs and faders are not yet used.
 */
void lwr_nk_teleop::processCartesianNoaButtonEvents()
{
    std::vector<int> buttons = currentNanokontrolMessage.buttons;

    double DELTA_T = 0.01;  // FIXME: don't hardcode
    double factor;
    bool haveChanges = false;
    std::vector<double> changes;
    changes.resize(3 + 3);
    for (int k = 0; k < (3 + 3); k++)
    {  // 8 channels, but we use only NOA and RPY
        factor = 0.0;

        if ((buttons[NK_RECORD_BASE + k] == 1) && (buttons[NK_MUTE_BASE + k] == 0))
        {  // R but not M
            factor = (buttons[NK_SOLO_BASE + k] == 1) ? -5 : -1;
            nkLEDs[NK_RECORD_BASE + k] = 1;
            nkLEDs[NK_MUTE_BASE + k] = 0;
            nkLEDs[NK_SOLO_BASE + k] = buttons[NK_SOLO_BASE + k];
            haveChanges = true;
        }
        else if ((buttons[NK_RECORD_BASE + k] == 0) && (buttons[NK_MUTE_BASE + k] == 1))
        {  // M but not R
            factor = (buttons[NK_SOLO_BASE + k] == 1) ? 5 : 1;
            nkLEDs[NK_RECORD_BASE + k] = 0;
            nkLEDs[NK_MUTE_BASE + k] = 1;
            nkLEDs[NK_SOLO_BASE + k] = buttons[NK_SOLO_BASE + k];
            haveChanges = true;
        }
        else
        {  // no button or both: invalid
            factor = 0.0;
            nkLEDs[NK_RECORD_BASE + k] = 0;
            nkLEDs[NK_MUTE_BASE + k] = 0;
            nkLEDs[NK_SOLO_BASE + k] = 0;
        }

        if (k <= 2)
            changes[k] = DELTA_T * factor * CARTESIAN_MOTION_MM_PER_SEC;  // 0.0001; // 1..5 mm
        else
            changes[k] = DELTA_T * factor * CARTESIAN_MOTION_RAD_PER_SEC;
    }

    // if (!haveChanges) return; // don't run through here if no button pressed

    ROS_INFO("NK teleop: delta NOA (%9.4lf %9.4lf %9.4lf) delta rpy (%9.4lf %9.4lf %9.4lf)", changes[0], changes[1],
             changes[2], changes[3], changes[4], changes[5]);

    // run FK on the start configuration to get initial world->T0->T6->tcp transform
    ros::Time t0 = ros::Time::now();
    robot_state->setJointGroupPositions(joint_model_group, targetAngles);  // goal position from previous cycle
    Eigen::Isometry3d initial_tcp = robot_state->getGlobalLinkTransform("tcp");
    ros::Time t1 = ros::Time::now();
    printTransform("NOA: initial world->tcp", initial_tcp);
    ROS_ERROR("NOA: FK took %8.3lf msecs.", (t1 - t0).toSec() * 1000.0);

    std::vector<double> startAngles = targetAngles;

    // update pose with changes from NK2 buttons, ordering on the NK2 is n-o-a--R-P-Y
    //
    tf::Transform initial_T6;
    tf::transformEigenToTF(initial_tcp, initial_T6);

    tf::Matrix3x3 matrix2(initial_T6.getBasis());  // original rotation
    tf::Vector3 delta_xyz(0, 0, 0);
    delta_xyz += matrix2.getColumn(0) * changes[0];  // N update (normal)
    delta_xyz += matrix2.getColumn(1) * changes[1];  // O update (orientation)
    delta_xyz += matrix2.getColumn(2) * changes[2];  // A update (approach)

    tf::Matrix3x3 delta_rpy;
    delta_rpy.setEulerYPR(changes[5], changes[4], changes[3]);
    tfScalar yaw, pitch, roll;
    matrix2 = matrix2 * delta_rpy;  // apply incremental change in local NOA system
    matrix2.getEulerYPR(yaw, pitch, roll);
    ROS_INFO("NOA: updated rotation is RPY %6.3lf %6.3lf %6.3lf", roll, pitch, yaw);

    tf::Transform updated_T6(initial_T6.getBasis() * delta_rpy, initial_T6.getOrigin() + delta_xyz);

    Eigen::Isometry3d updated_tcp;
    tf::transformTFToEigen(updated_T6, updated_tcp);
    printTransform("NOA: updated world->tcp ", updated_tcp);

    // run IK to find new target angles; using seed angles.
    //
    std::vector<double> ik_angles;
    ik_angles.resize(NUMBER_OF_JOINTS);
    ik_angles = targetAngles;  // start with previous goal position

    bool have_good_solution = false;
    int attempts = 0;
    double delta;
    while (!have_good_solution && (attempts < 10))
    {
        attempts++;

        // try to initialize with seed position...
        robot_state->setJointGroupPositions(joint_model_group, targetAngles);

        ros::Time t3 = ros::Time::now();
        bool found_ik = robot_state->setFromIK(joint_model_group, updated_tcp, "tcp", 1, 0.1);
        ros::Time t4 = ros::Time::now();
        ROS_INFO("NOA: IK solution found %d, solver took %7.3lf msec", found_ik, 1000 * (t4 - t3).toSec());

        if (found_ik)
        {
            robot_state->copyJointGroupPositions(joint_model_group, ik_angles);

            // double check
            robot_state->setJointGroupPositions(joint_model_group, ik_angles);
            Eigen::Isometry3d fuck = robot_state->getGlobalLinkTransform("tcp");
            printTransform("NOA: IK->FK  world->tcp", fuck);

            int joint_limits_mask = 0;
            delta = 0.0;
            for (int j = 0; j < NUMBER_OF_JOINTS; j++)
            {
                if (ik_angles[j] < lowerJointLimits[j])
                    joint_limits_mask |= (1 << j);
                if (ik_angles[j] > upperJointLimits[j])
                    joint_limits_mask |= (1 << j);
                delta = delta + fabs(ik_angles[j] - startAngles[j]);
            }
            if (joint_limits_mask != 0)
            {
                ROS_ERROR("NOA: IK solution incurs joint limit violation, mask=%x", joint_limits_mask);
                continue;
            }
            if (delta < DELTA_MIN_THRESHOLD)
                have_good_solution = true;

            // if (have_good_solution) {
            ROS_INFO(
                "NOA: IK solution found, joint angles  (%8.4lf %8.4lf %8.4lf %8.4lf %8.4lf %8.4lf %8.4lf) delta %8.4lf",
                ik_angles[0], ik_angles[1], ik_angles[2], ik_angles[3], ik_angles[4], ik_angles[5], ik_angles[6],
                delta);
        }
        else
        {
            ROS_ERROR("NOA: no IK solution found..., attempt %d", attempts);
        }
    }  // while

    if (!have_good_solution)
    {
        ROS_ERROR("NOA: IK solution too far from current joint state, canceled. delta= %8.4lf", delta);
        return;
    }
    else
    {
        ROS_INFO("NOA: moving to IK configuration now.");
    }

    // command new robot target position, then updated the button LEDS
    {
        for (int j = 0; j < NUMBER_OF_JOINTS; j++)
        {
            targetVelocities[j] = (ik_angles[j] - targetAngles[j]) / DELTA_T;
            targetAngles[j] = ik_angles[j];
        }
    }

    publishPositionGoal();
    publishNKLEDs();
}

void lwr_nk_teleop::processCartesianXyzButtonEvents()
{
    ROS_ERROR("NK teleop: Cartesian XYZ mode...");

    std::vector<int> buttons = currentNanokontrolMessage.buttons;
    // SOLO buttons index 0..7, MUTE 8..15, RECORD 16..23

    double DELTA_T = 0.01;  // FIXME: don't hardcode
    double factor;
    bool haveChanges = false;
    std::vector<double> changes;
    changes.resize(3 + 3);
    for (int k = 0; k < (3 + 3); k++)
    {  // 8 channels, but we use only XYZ and RPY
        factor = 0.0;

        if ((buttons[16 + k] == 1) && (buttons[8 + k] == 0))
        {  // R but not M
            factor = (buttons[0 + k] == 1) ? -5 : -1;
            nkLEDs[16 + k] = 1;
            nkLEDs[8 + k] = 0;
            nkLEDs[0 + k] = buttons[0 + k];
            haveChanges = true;
        }
        else if ((buttons[16 + k] == 0) && (buttons[8 + k] == 1))
        {  // M but not R
            factor = (buttons[0 + k] == 1) ? 5 : 1;
            nkLEDs[16 + k] = 0;
            nkLEDs[8 + k] = 1;
            nkLEDs[0 + k] = buttons[0 + k];
            haveChanges = true;
        }
        else
        {  // no button or both: invalid
            factor = 0.0;
            nkLEDs[16 + k] = 0;
            nkLEDs[8 + k] = 0;
            nkLEDs[0 + k] = 0;
        }

        if (k <= 2)
            changes[k] = DELTA_T * factor * CARTESIAN_MOTION_MM_PER_SEC;  // 0.0001; // 1..5 mm
        else
            changes[k] = DELTA_T * factor * CARTESIAN_MOTION_RAD_PER_SEC;
    }

    // if (!haveChanges) return; // don't run through here if no button pressed
    ROS_INFO("NK teleop: delta xyz (%8.4lf %8.4lf %8.4lf) delta rpy (%8.4lf %8.4lf %8.4lf)", changes[0], changes[1],
             changes[2], changes[3], changes[4], changes[5]);

    // calculate FK from current targetAngles to find effector pose
    //
    ros::Time t0 = ros::Time::now();
    // robot_state->setVariablePositions( jointAngles ); CRASHES...
    robot_state->setJointGroupPositions(joint_model_group, targetAngles);  // where we wanted to be so far
    Eigen::Isometry3d world_tcp = robot_state->getGlobalLinkTransform("tcp");
    ros::Time t1 = ros::Time::now();
    printTransform("XYZ: initial world->tcp", world_tcp);

    // update pose with changes from NK2 buttons, ordering on the NK2 is x-y-z-R-P-Y
    //
    tf::Transform initial_T6;
    tf::transformEigenToTF(world_tcp, initial_T6);
    float roll, pitch, yaw;
    tf::Vector3 delta_xyz(changes[0], changes[1], changes[2]);
    tf::Matrix3x3 delta_rpy;
    delta_rpy.setEulerYPR(1 * changes[5], 1 * changes[4], 1 * changes[3]);
    tf::Matrix3x3 matrix2(initial_T6.getRotation());  // original rotation
    matrix2 = delta_rpy * matrix2;                    // pre-multiplication is in global (world) system

    // matrix2.getEulerYPR( yaw, pitch, roll );
    // ROS_INFO( "updated rotation is RPY %6.3lf %6.3lf %6.3lf", roll, pitch, yaw );

    tf::Transform updated_T6(delta_rpy * initial_T6.getBasis(), initial_T6.getOrigin() + delta_xyz);
    printTransform("XYZ: updated world->tcp", updated_T6);
    ros::Time t2 = ros::Time::now();

    std::vector<double> startAngles;
    startAngles.resize(NUMBER_OF_JOINTS);
    for (int j = 0; j < NUMBER_OF_JOINTS; j++)
    {
        startAngles[j] = jointAngles[j];
    }
    ROS_INFO("IK start, start joint angles are (%8.4lf %8.4lf %8.4lf %8.4lf %8.4lf %8.4lf %8.4lf)", startAngles[0],
             startAngles[1], startAngles[2], startAngles[3], startAngles[4], startAngles[5], startAngles[6]);

    // run IK to find new target angles; using seed angles.
    // We check all eight possible configurations, and will then select
    // run IK to find new target angles; using seed angles.
    // We check all eight possible configurations, and will then select
    // the best solution, if any.
    //
    double delta_min = std::numeric_limits<double>::max();

    Eigen::Isometry3d updated_tcp;
    tf::transformTFToEigen(updated_T6, updated_tcp);

    std::vector<double> ik_angles;
    ik_angles.resize(NUMBER_OF_JOINTS);
    ik_angles = targetAngles;  // initialize with target from previous iteration

    bool have_good_solution = false;
    int attempts = 0;
    double delta;
    while (!have_good_solution && (attempts < 10))
    {
        attempts++;

        // try to initialize with seed position...
        robot_state->setJointGroupPositions(joint_model_group, startAngles);

        ros::Time t3 = ros::Time::now();
        bool found_ik = robot_state->setFromIK(joint_model_group, updated_tcp, "tcp", 1, 0.1);
        ros::Time t4 = ros::Time::now();

        ROS_INFO("IK solution found %d, solver took %7.3lf msec", found_ik, 1000 * (t4 - t3).toSec());

        if (found_ik)
        {
            robot_state->copyJointGroupPositions(joint_model_group, ik_angles);

            // double check
            robot_state->setJointGroupPositions(joint_model_group, ik_angles);
            Eigen::Isometry3d fuck = robot_state->getGlobalLinkTransform("tcp");
            printTransform("XYZ: IK    world->tcp", fuck);

            int joint_limits_mask = 0;
            delta = 0.0;
            for (int j = 0; j < NUMBER_OF_JOINTS; j++)
            {
                if (ik_angles[j] < lowerJointLimits[j])
                    joint_limits_mask |= (1 << j);
                if (ik_angles[j] > upperJointLimits[j])
                    joint_limits_mask |= (1 << j);
                delta = delta + fabs(ik_angles[j] - startAngles[j]);
            }
            if (joint_limits_mask != 0)
            {
                ROS_ERROR("XYZ: IK solution incurs joint limit violation, mask=%x", joint_limits_mask);
                continue;
            }
            if (delta < DELTA_MIN_THRESHOLD)
                have_good_solution = true;

            // if (have_good_solution) {
            ROS_INFO("IK solution found, joint angles  (%8.4lf %8.4lf %8.4lf %8.4lf %8.4lf %8.4lf %8.4lf) delta %8.4lf",
                     ik_angles[0], ik_angles[1], ik_angles[2], ik_angles[3], ik_angles[4], ik_angles[5], ik_angles[6],
                     delta);
        }
        else
        {
            ROS_ERROR("XYZ: no IK solution found..., attempt %d", attempts);
        }
    }  // while

    if (!have_good_solution)
    {
        ROS_ERROR("XYZ: IK solution too far from current joint state, canceled. delta= %8.4lf", delta);
        return;
    }
    else
    {
        ROS_INFO("XYZ: moving to IK configuration now.");
    }

    // command new robot target position, then updated the button LEDS
    {
        for (int j = 0; j < NUMBER_OF_JOINTS; j++)
        {
            targetVelocities[j] = (ik_angles[j] - startAngles[j]) / DELTA_T;
            targetAngles[j] = ik_angles[j];
        }
    }
    publishPositionGoal();
    publishNKLEDs();
}

/**
 * process knob events.
 * Currently only handles
 */
void lwr_nk_teleop::processKnobEvents()
{
    // ROS_WARN( "processKnobEvents: IMPLEMENT ME!" );

    pthread_mutex_lock(&nodeStateMutex);

    // Knob on channel 7 controls gripper, if WSG50 detected...
    if (nkKnobsChangedMap[7] == 1)
    {
        double value = nkKnobs[7];
        ROS_INFO("processKnobEvents: haveWSG50 %d knob 8 value %8.3lf", haveWSG50, value);
        if (haveWSG50)
        {
            wsg_50_common::Move move;
            move.request.width = 50.0 * (value + 1.0);  // -1.0..1.0 -> 0..100mm
            move.request.speed = 40.0;

            bool status = moveWSG50Client.call(move);
            if (!status)
            {
                ROS_ERROR("moveWSG50 gripper move call failed, error status is %d", move.response.error);
            }
        }
        nkKnobsChangedMap.erase(7);
    }

    // we have processed all knob events know, clear changes...
    nkKnobsChangedMap.clear();
    pthread_mutex_unlock(&nodeStateMutex);
}

/**
 * process fader/slider events. Currently, does nothing.
 */
void lwr_nk_teleop::processFaderEvents()
{
    ;
    // ROS_WARN( "handleFaderEvents: IMPLEMENT ME!" );
}

void lwr_nk_teleop::printTransform(std::string prefix, tf::Transform trafo)
{
    tfScalar roll, pitch, yaw;
    trafo.getBasis().getEulerYPR(yaw, pitch, roll);
    ROS_INFO("%s: origin: %7.4lf %7.4lf %7.4lf quat: %7.4lf %7.4lf %7.4lf %7.4lf rpy: %6.3lf %6.3lf %6.3lf",
             prefix.c_str(), trafo.getOrigin().x(), trafo.getOrigin().y(), trafo.getOrigin().z(),
             trafo.getRotation().x(), trafo.getRotation().y(), trafo.getRotation().z(), trafo.getRotation().w(), roll,
             pitch, yaw);
}

void lwr_nk_teleop::printTransform(std::string prefix, Eigen::Isometry3d affine3d)
{
    tf::Transform trafo;
    tf::transformEigenToTF(affine3d, trafo);
    printTransform(prefix, trafo);
}

/**
 * wrapper around transform listener lookupTransform.
 * Returns true if successful, false if not.
 * Note: this assumes that the transform listener has been initialized.
 */
bool lwr_nk_teleop::lookupTransform(std::string root_frame, std::string target_frame, tf::StampedTransform& trafo)
{
    if (verbose)
        ROS_INFO("lookupTransform %s -> %s...", root_frame.c_str(), target_frame.c_str());
    try
    {
        tfl->waitForTransform(root_frame, target_frame, ros::Time(0), ros::Duration(1));
        tfl->lookupTransform(root_frame, target_frame, ros::Time(0), trafo);
        return true;
    }
    catch (tf::TransformException& exception)
    {
        ROS_ERROR("TransformListener failed: %s", exception.what());
        return false;
    }
}

/**
 * sends a stop-robot request to the LWR reflexxes node.
 */
void lwr_nk_teleop::publishStopRobot(bool stop)
{
    std_msgs::Bool msg;
    msg.data = stop;
    stopRobotPublisher.publish(msg);
}

/**
 * publishes a RML velocity goal inside a JointState message.
 */
void lwr_nk_teleop::publishVelocityGoal()
{
    sensor_msgs::JointState msg;
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = "world";
    msg.header.seq = nk_teleop_seq++;
    msg.name = jointNames;

    ros_fri_msgs::RMLVelocityInputParameters vv;
    vv.header = msg.header;
    vv.TargetVelocityVector.resize(NUMBER_OF_JOINTS);
    vv.MaxAccelerationVector.resize(NUMBER_OF_JOINTS);

    pthread_mutex_lock(&nodeStateMutex);
    // msg.position = targetAngles; // in radians
    msg.velocity = targetVelocities;

    for (int j = 0; j < NUMBER_OF_JOINTS; j++)
    {
        vv.TargetVelocityVector[j] = msg.velocity[j];
        vv.MaxAccelerationVector[j] = jointAccelerationLimits[j];
    }
    // msg.effort = NaN; // ignored by _reflexxes anyway...
    pthread_mutex_unlock(&nodeStateMutex);

    velocityGoalPublisher.publish(msg);
    rmlVelocityGoalPublisher.publish(vv);
}

/**
 * publishes a RML position goal inside a JointState message.
 * Only joint angles (position) and velocity are used,
 * effort is left empty and is ignored by RML anyway.
 */
void lwr_nk_teleop::publishPositionGoal()
{
    sensor_msgs::JointState msg;
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = "world";
    msg.header.seq = nk_teleop_seq++;  // FIXME: the 0-1-0-1 allows us to track header.seq in rqt_plot
    msg.name = jointNames;

    ros_fri_msgs::RMLPositionInputParameters ppp;
    ppp.header = msg.header;
    ppp.TargetPositionVector.resize(NUMBER_OF_JOINTS);
    ppp.TargetVelocityVector.resize(NUMBER_OF_JOINTS);
    ppp.MaxVelocityVector.resize(NUMBER_OF_JOINTS);
    ppp.MaxAccelerationVector.resize(NUMBER_OF_JOINTS);

    pthread_mutex_lock(&nodeStateMutex);
    msg.position = targetAngles;  // in radians
    // msg.velocity = targetVelocities;
    msg.velocity = zeroVelocities;
    // msg.effort = NaN;
    msg.effort.resize(7);
    msg.effort[6] = 0.02 * (nk_teleop_seq & 0x1);

    for (int j = 0; j < NUMBER_OF_JOINTS; j++)
    {
        ppp.TargetPositionVector[j] = msg.position[j];
        ppp.TargetVelocityVector[j] = msg.velocity[j];
        ppp.MaxVelocityVector[j] = jointVelocityLimits[j];
        ppp.MaxAccelerationVector[j] = jointAccelerationLimits[j];
    }

    pthread_mutex_unlock(&nodeStateMutex);

    positionGoalPublisher.publish(msg);
    rmlPositionGoalPublisher.publish(ppp);
}

/**
 * requests an emergency stop of the LWR robot.
 */
void lwr_nk_teleop::publishEmergencyStop(bool b)
{
    ROS_WARN("publishEmergencyStop: IMPLEMENT ME!");
    std_msgs::Bool msg;
    msg.data = b;
    emergencyStopPublisher.publish(msg);
}

void lwr_nk_teleop::publishCancelTrajectory()
{
    ROS_WARN("publishCancelTrajectory: IMPLEMENT ME!");
    /*
      actionlib_msgs::FollowJointTrajectoryActionGoal msg;
      msg.header.stamp = ros::Time:now();
      msg.goal_id = "HOW WOULD I KNOW?";
      cancelTrajectoryPublisher.publish( msg );
    */
}

/**
 * publishes a message with all current LED states.
 */
void lwr_nk_teleop::publishNKLEDs()
{
    sensor_msgs::Joy msg;
    msg.header.seq = 0;  // FIXME n_nk_leds++;
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = "nanokontrol";
    msg.buttons = nkLEDs;

    if (n_blink_callbacks & 0x1)
    {  // odd/even used for blink cycle
        std::map<int, int>::iterator it;
        for (it = nkBlinkMask.begin(); it != nkBlinkMask.end(); it++)
        {
            int index = it->first;
            int value = it->second;
            // ROS_INFO( "blinking LED %d value %d...", index, value );
            if (value == 1)
            {
                msg.buttons[index] = (msg.buttons[index] == 1) ? 0 : 1;
            }
        }
    }
    // msg.axes = ... NK2 has no motorfaders or motor knobs
    ledPublisher.publish(msg);
}

/**
 * switches to STOP state (and publishes a stop robot message),
 * then publishes a last LED message to switch all NK LEDs off.
 * We also usleep() a bit to have those messages published still.
 */
void lwr_nk_teleop::shutdown()
{
    setNodeState(STOP);

    for (int i = 0; i < NUMBER_NK_BUTTONS; i++)
    {
        nkLEDs[i] = 0;
    }
    publishNKLEDs();
    usleep(500 * 1000);
}

int lwr_nk_teleop::run()
{
    ROS_WARN("run: IMPLEMENT ME!");
    ros::spin();
}

// a local pointer to the ROS node, required for the signal handler
static lwr_nk_teleop* nk_ptr;

/*
 * cntl-c signal handler: ensure that the robot is stopped
 * before we exit.
 */
void SIGINT_handler(int signal)
{
    ROS_WARN("lwr_nk_teleop: received cntl-c (SIGINT), shutting down...");

    ROS_ERROR("FUCKFUCKFUCK 00000 !!!! ");
    if (nk_ptr != NULL)
    {
        nk_ptr->shutdown();
        usleep(2 * 1000 * 1000);
    }
    // ros::shutdown(); // this does NOT work as expected...
    ROS_ERROR("FUCKFUCKFUCK 11111 !!!! ");
    ros::shutdown();
    ROS_ERROR("FUCKFUCKFUCK 22222!!!! ");
    exit(-1);
    ROS_ERROR("FUCKFUCKFUCK 33333!!!! ");
}

int main(int argc, char** argv)
{
    // std::signal( SIGINT, SIGINT_handler );
    // ros::init( argc, argv, "lwr_nk_teleop", 1 ); // 1=no default SIGINT handler
    ros::init(argc, argv, "lwr_nk_teleop");  // 1=no default SIGINT handler
    ros::AsyncSpinner spinner(2);            // 2 threads
    spinner.start();

    lwr_nk_teleop nk_teleop;

    ros::spin();
    exit(0);
    // int status = nk_teleop.run();
    // exit ( status );
}
