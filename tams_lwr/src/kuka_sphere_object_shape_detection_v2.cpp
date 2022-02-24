/**
kuka_sphere_object_shape_detection.cpp

This class provides blind object shape detection. The tip of the KuKA LWR4+
drives along the surface of an object and iteratively builds up a 3D model
of the object with OctoMap/PCL in real time. Force measurements are used to
determine contact between tip of the robotic arm and the object as well as
to determine movement direction.
A sphere is installed as tip of the robotic arm to make calculations regarding
absolute position of contact point and movement direction simpler.

The object is situated in a so called "shape_detection_workspace". The tip of
the robotic arm searches for the object on a plane in absolute y-axis. Is the
boundary of the y-axis in the workspace reached, the tip moves in x-axis-
direction for 1cm to be followed by movement along the y-axis now in opposite
y-direction. Is an object found, the tip moves along the object on the y- and
z-axis.
The last joint of the KuKA LWR4+ is set to be orthogonal to the surface of
the object to not run into unwanted collisions with the object. The wanted
contactpoint is between tip (sphere) and object. The orientation of the last
joint needs is calculated upon position of the contactpoint on the sphere.

Maze form:
x-y-axes:           +     y-z-axes:
x: | & ^            +     y:  <--- & --->
   V   |            +
                    +     z: | & ^
y:  <--- & --->     +        V   |
                    +
Moving direction:   +     Moving direction:
<-----------        +     O: object
|                   +         vvvvv<
v                   +     <---vOOOO^----
----------->        +     |
           |        +     v   >vvvvv
           v        +     ----^OOOOv--->
<-----------        +


Actual robot motions are performed using the reflexxes motion library and
the KuKA fast research interface.

Notes:

The necessary FK and IK calculations are performed using the
MoveIt API; note that "robot_description" and also the corresponding
semantic description "robot_description_semantic" need to be on
the param server for this to work.
(But move_group is not necessary.)

See:

http://docs.ros.org/kinetic/api/moveit_tutorials/html/doc/pr2_tutorials/kinematics/src/doc/kinematic_model_tutorial.html
http://docs.ros.org/indigo/api/moveit_core/html/classmoveit_1_1core_1_1RobotState.html
//https://eigen.tuxfamily.org/dox-devel/group__TutorialGeometry.html

History:
2017.03.09 - sphere_object_shape_detection node adapted for KuKA LWR4+

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
#include <geometry_msgs/WrenchStamped.h>
#include <geometry_msgs/Vector3.h>

// MoveIt
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>

// OctoMap
#include <octomap/octomap.h>
#include <octomap/OcTree.h>
#include <octomap_msgs/conversions.h>

#include <eigen3/Eigen/Eigen>

// #include <pcl_conversions/pcl_conversions.h>
// #include <pcl/point_types.h>
// #include <pcl_ros/point_cloud.h>

#define NUMBER_OF_JOINTS 7
#define CYCLE_TIME_IN_SECONDS 0.01

// typedef pcl::PointCloud<pcl::PointXYZ> PCLCloud;

class kuka_sphere_object_shape_detection
{
public:
    kuka_sphere_object_shape_detection();
    ~kuka_sphere_object_shape_detection();
    int run();

    void jointStateCallback(const sensor_msgs::JointState jointStateMessage);
    void estimatedExternalWrenchCallback(const geometry_msgs::WrenchStamped externalWrench);
    void estimatedExternalTCPWrenchCallback(const geometry_msgs::WrenchStamped externalTCPWrench);

    void publishEmergencyStop(bool stop);
    void publishStopRobot(bool stop);
    void publishCancelTrajectory();
    void publishVelocityGoal();
    void publishPositionGoal();

    void publishStartGoal();
    void searchObjectGoal();

    bool lookupTransform(std::string root_frame, std::string target_frame, tf::StampedTransform& trafo);
    void printTransform(std::string prefix, tf::Transform trafo);
    void printTransform(std::string prefix, Eigen::Isometry3d trafo);

private:
    bool parseWorkspaceLimits(std::string tokens);
    bool parseShapeWorkspaceLimits(std::string tokens);
    void checkWorkspace(const tf::Vector3 spherePosition);
    void buildOctomap(tf::Vector3 PointOnSphere);
    void changeModeResetMotion();
    tf::Vector3 PointOnSphere();
    tf::Vector3 TangentVector();
    geometry_msgs::Vector3Stamped TranslationRule(tf::Vector3 forceVec);
    geometry_msgs::Vector3Stamped TranslationRuleB(tf::Vector3 arg1);
    tf::Vector3 reapproachObject();

    pthread_mutex_t nodeStateMutex;

    ros::NodeHandle nh;
    ros::NodeHandle nnh;
    bool verbose;

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

    tf::Vector3 externalWrenchVec;
    tf::Vector3 externalTCPWrenchVec;
    tf::Vector3 latestContactForce;

    // pcl::PointCloud<pcl::PointXYZ> *pc;
    octomap::OcTree* tree;

    std::map<std::string, double> workspaceLimits;                // "xmin" -> value, etc.
    std::map<std::string, double> shapeDetectionWorkspaceLimits;  // "xmin" -> value, etc.

    // ROS stuff
    ros::Subscriber jointStatesSubscriber;  // sensor_msgs::JointState
    ros::Subscriber externalWrenchSubscriber;
    ros::Subscriber externalTCPWrenchSubscriber;
    ros::Publisher emergencyStopPublisher;
    ros::Publisher stopRobotPublisher;
    ros::Publisher cancelTrajectoryPublisher;
    ros::Publisher positionGoalPublisher;  // joint state
    ros::Publisher velocityGoalPublisher;
    ros::Publisher rmlPositionGoalPublisher;  // RML messages
    ros::Publisher rmlVelocityGoalPublisher;
    ros::Publisher octomapPublisher;
    ros::Publisher pointcloudPublisher;

    tf::TransformListener* tfl;
    tf::TransformBroadcaster* tbr;

    tf::Transform startPosition;

    Eigen::Isometry3d world_base_transform;  // world -> lwr_arm_base_link
    Eigen::Isometry3d tool_transform;        // lwr_arm_7_link -> tcp

    robot_state::RobotStatePtr robot_state;
    robot_model::RobotModelPtr kinematic_model;
    robot_model_loader::RobotModelLoader robot_model_loader;
    const robot_state::JointModelGroup* joint_model_group;

    sensor_msgs::JointState lastJointStateMessage;  // received robot state

    ros::Duration robotTimeout;  // ROBOT_ERROR when joint_states dry out

    bool startPositionInitialized;
    bool leftMode;
    bool modeChange;
    bool currentYInitialized;
    bool latestContactForceInitialized;
    bool inside;
    bool firstItCMRM;
    // bool forceGoalPoseReached;

    int reapproachCounter;

    double FORCE_THRESHOLD;
    double currentY;
    double commandRate;

    long n_lwr_callbacks;
    long n_lwr_wrench_callbacks;
    long n_lwr_TCP_wrench_callbacks;
};

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

kuka_sphere_object_shape_detection::kuka_sphere_object_shape_detection()
{
    // node state and mutex
    //
    int s = pthread_mutex_init(&nodeStateMutex, NULL);
    if (s != 0)
    {
        ROS_ERROR("pthread_mutex_init failed.");
        exit(1);
    }

    ros::NodeHandle nnh("~");  // private node handle, private namespace

    nnh.param("command_rate", commandRate, 0.25);

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
    std::string velocity_limits, acceleration_limits, jerk_limits, workspace_limits, shape_detection_workspace_limits;

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

    nnh.param("workspace_limits", workspace_limits,
              std::string("-0.01 0.17 0.45 0.61 0.94 1.15"));  // xmin xmax ymin ymax zmin zmax
    parseWorkspaceLimits(workspace_limits);

    nnh.param("shape_detection_workspace_limits", shape_detection_workspace_limits,
              std::string("0.0 0.15 0.47 0.59 0.96 1.13"));  // xmin xmax ymin ymax zmin zmax
    parseShapeWorkspaceLimits(shape_detection_workspace_limits);

    ROS_INFO("joint position, velocity, acceleration limits initialized.");

    ROS_WARN("lwr_nk_teleop: robot limits after node initialization:");
    ROS_WARN("   limits   lower : upper   velocity  acceleration  jerk:");
    for (int j = 0; j < NUMBER_OF_JOINTS; j++)
    {
        ROS_WARN("   %.1d  %8.4lf : %8.4lf  %8.4lf  %8.4lf  %8.4lf", j, lowerJointLimits[j], upperJointLimits[j],
                 jointVelocityLimits[j], jointAccelerationLimits[j], jointJerkLimits[j]);
    }

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

    std::string tip = joint_model_group->getEndEffectorName();
    ROS_INFO("MoveIt: end effector %s", tip.c_str());

    // ROS publishers and subscribers
    //
    emergencyStopPublisher = nnh.advertise<std_msgs::Bool>("emergency_stop", 1);
    stopRobotPublisher = nnh.advertise<std_msgs::Bool>("stop_robot", 1);
    cancelTrajectoryPublisher = nnh.advertise<sensor_msgs::JointState>("cancel", 1);
    positionGoalPublisher = nnh.advertise<sensor_msgs::JointState>("joint_position_goal", 1);
    velocityGoalPublisher = nnh.advertise<sensor_msgs::JointState>("joint_velocity_goal", 1);

    // extra topics for RML-style commands
    rmlPositionGoalPublisher = nnh.advertise<ros_fri_msgs::RMLPositionInputParameters>("/lwr/jointPositionGoal", 1);
    rmlVelocityGoalPublisher = nnh.advertise<ros_fri_msgs::RMLVelocityInputParameters>("jointVelocityGoal", 1);

    jointStatesSubscriber = nnh.subscribe<sensor_msgs::JointState>(
        "/lwr/joint_states", 1, &kuka_sphere_object_shape_detection::jointStateCallback, this);
    externalWrenchSubscriber = nnh.subscribe<geometry_msgs::WrenchStamped>(
        "/lwr/estimatedExternalWrench", 1, &kuka_sphere_object_shape_detection::estimatedExternalWrenchCallback, this);
    externalTCPWrenchSubscriber = nnh.subscribe<geometry_msgs::WrenchStamped>(
        //"/lwr/estimatedExternalTcpWrench", 1,
        "/wireless_ft/wrench_3", 1, &kuka_sphere_object_shape_detection::estimatedExternalTCPWrenchCallback, this);

    octomapPublisher = nh.advertise<octomap_msgs::Octomap>("display_env", 1);
    // pointcloudPublisher = nh.advertise<PCLCloud>("pointcloud", 1);

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

    startPositionInitialized = false;
    leftMode = true;
    modeChange = false;
    currentYInitialized = false;
    latestContactForceInitialized = false;
    inside = true;
    firstItCMRM = true;
    // forceGoalPoseReached = true;

    tree = new octomap::OcTree(0.01);
    FORCE_THRESHOLD = 0.04;
    reapproachCounter = 0;

}  // end constructor

kuka_sphere_object_shape_detection::~kuka_sphere_object_shape_detection()
{
}

bool kuka_sphere_object_shape_detection::parseWorkspaceLimits(std::string tokens)
{
    double xmin, xmax, ymin, ymax, zmin, zmax;
    if (6 == sscanf(tokens.c_str(), "%lf %lf %lf %lf %lf %lf", &xmin, &xmax, &ymin, &ymax, &zmin, &zmax))
    {
        // ok so far, now some sanity checks
        assert(xmax > xmin);
        assert(xmin > -0.03);
        assert(xmax < 0.18);
        assert(ymax > ymin);
        assert(zmax > zmin);
        assert(zmin > 0.92);  // table height
        workspaceLimits["xmin"] = xmin;
        workspaceLimits["xmax"] = xmax;
        workspaceLimits["ymin"] = ymin;
        workspaceLimits["ymax"] = ymax;
        workspaceLimits["zmin"] = zmin;
        workspaceLimits["zmax"] = zmax;
    }
    else
    {
        ROS_ERROR("Failed to create workspace limits from given string '%s'", tokens.c_str());
        return false;
    }
    ROS_ERROR("Workspace limits x: %6.3lf .. %6.3lf y: %6.3lf %6.3lf z: %6.3lf %6.3lf", xmin, xmax, ymin, ymax, zmin,
              zmax);
    return true;
}

bool kuka_sphere_object_shape_detection::parseShapeWorkspaceLimits(std::string tokens)
{
    double xmin, xmax, ymin, ymax, zmin, zmax;
    if (6 == sscanf(tokens.c_str(), "%lf %lf %lf %lf %lf %lf", &xmin, &xmax, &ymin, &ymax, &zmin, &zmax))
    {
        // ok so far, now some sanity checks
        assert(xmax > xmin);
        assert(xmin > -0.01);
        assert(xmax < 0.17);
        assert(ymax > ymin);
        assert(zmax > zmin);
        assert(zmin > 0.94);  // table height
        shapeDetectionWorkspaceLimits["xmin"] = xmin;
        shapeDetectionWorkspaceLimits["xmax"] = xmax;
        shapeDetectionWorkspaceLimits["ymin"] = ymin;
        shapeDetectionWorkspaceLimits["ymax"] = ymax;
        shapeDetectionWorkspaceLimits["zmin"] = zmin;
        shapeDetectionWorkspaceLimits["zmax"] = zmax;
    }
    else
    {
        ROS_ERROR("Failed to create workspace limits from given string '%s'", tokens.c_str());
        return false;
    }
    ROS_ERROR("Workspace limits x: %6.3lf .. %6.3lf y: %6.3lf %6.3lf z: %6.3lf %6.3lf", xmin, xmax, ymin, ymax, zmin,
              zmax);
    return true;
}

void kuka_sphere_object_shape_detection::checkWorkspace(const tf::Vector3 spherePosition)
{
    bool insideShapeDetection = true;
    if (spherePosition.getX() < workspaceLimits["xmin"])
        inside = false;
    if (spherePosition.getX() > workspaceLimits["xmax"])
        inside = false;
    if (spherePosition.getY() < workspaceLimits["ymin"])
        inside = false;
    if (spherePosition.getY() > workspaceLimits["ymax"])
        inside = false;
    if (spherePosition.getZ() < workspaceLimits["zmin"])
        inside = false;
    if (spherePosition.getZ() > workspaceLimits["zmax"])
        inside = false;

    if (spherePosition.getX() < shapeDetectionWorkspaceLimits["xmin"])
    {
        insideShapeDetection = false;
        ROS_ERROR("x < xmin: modechange true");
        leftMode = true;
        modeChange = true;
        firstItCMRM = true;
        latestContactForceInitialized = false;
    }
    if (spherePosition.getX() > shapeDetectionWorkspaceLimits["xmax"])
    {
        insideShapeDetection = false;
        ROS_ERROR("x > xmax: modechange true");
        leftMode = false;
        modeChange = true;
        firstItCMRM = true;
        latestContactForceInitialized = false;
    }
    if (spherePosition.getY() < shapeDetectionWorkspaceLimits["ymin"])
        insideShapeDetection = false;
    if (spherePosition.getY() > shapeDetectionWorkspaceLimits["ymax"])
        insideShapeDetection = false;
    if (spherePosition.getZ() < shapeDetectionWorkspaceLimits["zmin"])
    {
        // insideShapeDetection = false;
        latestContactForceInitialized = false;
    }
    if (spherePosition.getZ() > shapeDetectionWorkspaceLimits["zmax"])
    {
        insideShapeDetection = false;
        latestContactForceInitialized = false;
    }

    // outside of workspace: stopRobot
    if (!inside)
    {
        ROS_ERROR("Outside allowed workspace. x y z %4.4lf %4.4lf %4.4lf", spherePosition.getX(), spherePosition.getY(),
                  spherePosition.getZ());
        publishStopRobot(true);
        return;
    }
    if (!insideShapeDetection)
    {
        ROS_ERROR("Outside allowed shape detection workspace.");
        ROS_ERROR("x y z: %4.4lf %4.4lf %4.4lf", spherePosition.getX(), spherePosition.getY(), spherePosition.getZ());
    }
}

void kuka_sphere_object_shape_detection::buildOctomap(const tf::Vector3 PointOnSphere)
{
    geometry_msgs::Vector3Stamped PoSphere;
    geometry_msgs::Vector3Stamped PInWorld;
    tf::StampedTransform stamped_transform;

    Eigen::Isometry3d spherePosition = robot_state->getGlobalLinkTransform("sphere");
    tf::Transform spherePositionTF;
    tf::transformEigenToTF(spherePosition, spherePositionTF);

    PoSphere.header.frame_id = "sphere";
    PoSphere.header.stamp = ros::Time::now();
    PoSphere.vector.x = PointOnSphere.getX();
    PoSphere.vector.y = PointOnSphere.getY();
    PoSphere.vector.z = PointOnSphere.getZ();
    try
    {
        tfl->waitForTransform("world", "sphere", ros::Time::now(), ros::Duration(1));
        tfl->lookupTransform("world", "sphere", ros::Time(0), stamped_transform);
        tfl->transformVector("world", ros::Time(0), PoSphere, "sphere", PInWorld);
    }
    catch (tf::TransformException& exception)
    {
        ROS_ERROR("Transform failed: %s", exception.what());
    }

    PInWorld.vector.x += spherePositionTF.getOrigin().getX();
    PInWorld.vector.y += spherePositionTF.getOrigin().getY();
    PInWorld.vector.z += spherePositionTF.getOrigin().getZ();
    octomap::point3d endpoint(PInWorld.vector.x, PInWorld.vector.y, PInWorld.vector.z);
    tree->updateNode(endpoint, true);
    tree->writeBinary("/homeL/9rau/simple_tree.bt");

    octomap_msgs::Octomap omap_msg;
    omap_msg.header.frame_id = "world";
    octomap_msgs::binaryMapToMsg(*tree, omap_msg);
    if (octomapPublisher.getNumSubscribers() > 0)
    {
        octomapPublisher.publish(omap_msg);
    }

    //   pcl::PointXYZ point;
    //   point = pcl::PointXYZ(PInWorld.vector.x, PInWorld.vector.y, PInWorld.vector.z);
    //   pc->push_back(point);
    //   if(pointcloudPublisher.getNumSubscribers() > 0)
    //   {
    //     pointcloudPublisher.publish(*pc);
    //   }

    //   ros::Publisher pub;
    //   pub = nh.advertise<sensor_msgs::PointCloud2> ("pcloud", 1);
    //   sensor_msgs::PointCloud2 shape;
    //   pcl::toROSMsg(*pc,shape);
    //   shape.header.frame_id="world";
    //   pub.publish (shape);
    // pcl::io::savePCDFile("/homeL/9rau/pcd.pcd",*pc);
}

void kuka_sphere_object_shape_detection::jointStateCallback(const sensor_msgs::JointState jointStateMessage)
{
    n_lwr_callbacks++;
    if (verbose && n_lwr_callbacks < 5)
    {
        ROS_INFO("NK teleop: received lwr joint state %ld", n_lwr_callbacks);
    }

    pthread_mutex_lock(&nodeStateMutex);

    unsigned int matched_lwr_joints = 0;
    for (unsigned int i = 0; i < jointStateMessage.name.size(); i++)
    {
        std::string name = jointStateMessage.name[i];
        int found = jointIndexMap.count(name);
        if (found > 0)
        {
            int lwrIndex = jointIndexMap[name];
            jointAngles[lwrIndex] = jointStateMessage.position[i];
            jointVelocities[lwrIndex] = jointStateMessage.velocity[i];
            jointTorques[lwrIndex] = jointStateMessage.effort[i];
            matched_lwr_joints++;
        }
        else if (verbose)
        {
            ROS_INFO("NK teleop: unmatched joint name '%s'", name.c_str());
        }
    }

    if (matched_lwr_joints == NUMBER_OF_JOINTS)
    {
        lastJointStateMessage = jointStateMessage;
    }
    else
    {
        ROS_ERROR("NK teleop: lwr joint state invalid/incomplete,");
        ROS_ERROR("NK teleop: ... only matched %d LWR joints.", matched_lwr_joints);
    }
    pthread_mutex_unlock(&nodeStateMutex);
}

void kuka_sphere_object_shape_detection::estimatedExternalWrenchCallback(
    const geometry_msgs::WrenchStamped externalWrench)
{
    n_lwr_wrench_callbacks++;
    if (verbose && n_lwr_wrench_callbacks < 5)
    {
        ROS_INFO("Received external wrench %ld", n_lwr_callbacks);
    }

    pthread_mutex_lock(&nodeStateMutex);

    externalWrenchVec[0] = externalWrench.wrench.force.x;
    externalWrenchVec[1] = externalWrench.wrench.force.y;
    externalWrenchVec[2] = externalWrench.wrench.force.z;

    pthread_mutex_unlock(&nodeStateMutex);
}

void kuka_sphere_object_shape_detection::estimatedExternalTCPWrenchCallback(
    const geometry_msgs::WrenchStamped externalTCPWrench)
{
    n_lwr_TCP_wrench_callbacks++;
    if (verbose && n_lwr_TCP_wrench_callbacks < 5)
    {
        ROS_INFO("Received external tcp wrench %ld", n_lwr_callbacks);
    }

    pthread_mutex_lock(&nodeStateMutex);

    externalTCPWrenchVec[0] = externalTCPWrench.wrench.force.x;
    externalTCPWrenchVec[1] = externalTCPWrench.wrench.force.y;
    externalTCPWrenchVec[2] = externalTCPWrench.wrench.force.z;

    pthread_mutex_unlock(&nodeStateMutex);
}

void kuka_sphere_object_shape_detection::printTransform(std::string prefix, tf::Transform trafo)
{
    tfScalar roll, pitch, yaw;
    trafo.getBasis().getEulerYPR(yaw, pitch, roll);
    ROS_INFO("%s: origin: %7.4lf %7.4lf %7.4lf quat: %7.4lf %7.4lf %7.4lf %7.4lf rpy: %6.3lf %6.3lf %6.3lf",
             prefix.c_str(), trafo.getOrigin().x(), trafo.getOrigin().y(), trafo.getOrigin().z(),
             trafo.getRotation().x(), trafo.getRotation().y(), trafo.getRotation().z(), trafo.getRotation().w(), roll,
             pitch, yaw);
}

void kuka_sphere_object_shape_detection::printTransform(std::string prefix, Eigen::Isometry3d affine3d)
{
    tf::Transform trafo;
    tf::transformEigenToTF(affine3d, trafo);
    printTransform(prefix, trafo);
}

bool kuka_sphere_object_shape_detection::lookupTransform(std::string root_frame, std::string target_frame,
                                                         tf::StampedTransform& trafo)
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

geometry_msgs::Vector3Stamped kuka_sphere_object_shape_detection::TranslationRule(tf::Vector3 forceVec)
{
    //  geometry_msgs::Vector3Stamped ruleTangent;
    //  ROS_ERROR("TVecStampFT values %4.4lf %4.4lf %4.4lf", forceVec.getX(), forceVec.getY(), forceVec.getZ());
    //  double mem;
    //  int vorzeichen;
    //  if (std::abs(forceVec.getY()) > std::abs(forceVec.getZ()) || std::abs(forceVec.getZ()) > std::abs(forceVec.getY()))
    //  {
    //    if (forceVec.getY() < 0 && forceVec.getZ() > 0 || forceVec.getY() > 0 && forceVec.getZ() < 0)
    //    {
    //      vorzeichen = -1;
    //    }
    //    else
    //    {
    //      vorzeichen = 1;
    //    }
    //    mem = forceVec.getZ();
    //    forceVec[2] = forceVec.getY() * vorzeichen;
    //    forceVec[1] = mem * vorzeichen;
    //  }
    //
    //  if (leftMode) //to the left
    //  {
    //    if (forceVec.getY() > 0 && forceVec.getZ() < 0) // ->v
    //    {
    //      forceVec[2] *= -1;
    //    }
    //    else if (forceVec.getY() > 0 && forceVec.getZ() > 0) // ->^
    //    {
    //      forceVec[1] *= -1;
    //    }
    //    else if (forceVec.getY() < 0 && forceVec.getZ() > 0) // ^<-
    //    {
    //      forceVec[2] *= -1;
    //    }
    //    else if (forceVec.getY() < 0 && forceVec.getZ() < 0) // v<-
    //    {
    //      forceVec[1] *= -1;
    //    }
    //    else { // move in in opposite direction of last contact force
    //      forceVec[0] *= -1;
    //      forceVec[1] *= -1;
    //      forceVec[2] *= -1;
    //    }
    //  }
    //  else if (!leftMode) //to the right
    //  {
    //    if (forceVec.getY() < 0 && forceVec.getZ() < 0) // v<-
    //    {
    //      forceVec[2] *= -1;
    //    }
    //    else if (forceVec.getY() < 0 && forceVec.getZ() > 0) // ^<-
    //    {
    //      forceVec[1] *= -1;
    //    }
    //    else if (forceVec.getY() > 0 && forceVec.getZ() > 0) // ->^
    //    {
    //      forceVec[2] *= -1;
    //    }
    //    else if (forceVec.getY() > 0 && forceVec.getZ() < 0) // ->v
    //    {
    //      forceVec[1] *= -1;
    //    }
    //    else { // move in in opposite direction of last contact force
    //      forceVec[0] *= -1;
    //      forceVec[1] *= -1;
    //      forceVec[2] *= -1;
    //    }
    //  }
    //  forceVec[0] = 0;
    //  ruleTangent.vector.x = forceVec[0];
    //  ruleTangent.vector.y = forceVec[1];
    //  ruleTangent.vector.z = forceVec[2];
    //  return ruleTangent;
}

geometry_msgs::Vector3Stamped kuka_sphere_object_shape_detection::TranslationRuleB(tf::Vector3 forceVec)
{
    geometry_msgs::Vector3Stamped ruleTangent;
    ROS_ERROR("TVecStampFT values %4.4lf %4.4lf %4.4lf", forceVec.getX(), forceVec.getY(), forceVec.getZ());
    double topForce, weakerForce, relativeWeakForce, mem;
    int vorzeichen;
    if (std::abs(forceVec.getX()) > std::abs(forceVec.getZ()) || std::abs(forceVec.getZ()) > std::abs(forceVec.getX()))
    {
        if ((forceVec.getX() < 0 && forceVec.getZ()) > 0 || (forceVec.getX() > 0 && forceVec.getZ() < 0))
        {
            vorzeichen = -1;
        }
        else
        {
            vorzeichen = 1;
        }
        mem = forceVec.getZ();
        forceVec[2] = forceVec.getX() * vorzeichen;
        forceVec[0] = mem * vorzeichen;

        if (std::abs(forceVec.getX()) > std::abs(forceVec.getZ()))
        {
            topForce = forceVec.getX();
            weakerForce = forceVec.getZ();
        }
        else
        {
            topForce = forceVec.getZ();
            weakerForce = forceVec.getX();
        }
        if (topForce < 0)
        {
            topForce *= -1;
        }
        relativeWeakForce = weakerForce / topForce;
    }

    if (leftMode)  // to the left
    {
        if (forceVec.getX() > 0 && forceVec.getZ() < 0)  // ->v
        {
            forceVec[0] = (topForce - FORCE_THRESHOLD) * 0.002;
            forceVec[2] = (topForce - FORCE_THRESHOLD) * relativeWeakForce * 0.002 * -1;
        }
        else if (forceVec.getX() > 0 && forceVec.getZ() > 0)  // ->^
        {
            forceVec[0] = (topForce - FORCE_THRESHOLD) * relativeWeakForce * 0.002 * -1;
            forceVec[2] = (topForce - FORCE_THRESHOLD) * 0.002;
        }
        else if (forceVec.getX() < 0 && forceVec.getZ() > 0)  // ^<-
        {
            forceVec[0] = (topForce - FORCE_THRESHOLD) * 0.002;
            forceVec[2] = (topForce - FORCE_THRESHOLD) * relativeWeakForce * 0.002 * -1;
        }
        else if (forceVec.getX() < 0 && forceVec.getZ() < 0)  // v<-
        {
            forceVec[0] = (topForce - FORCE_THRESHOLD) * relativeWeakForce * 0.002 * -1;
            forceVec[2] = (topForce - FORCE_THRESHOLD) * 0.002;
        }
        else
        {  // move in in opposite direction of last contact force
            forceVec[0] *= -0.002;
            forceVec[1] *= -0.002;
            forceVec[2] *= -0.002;
        }
    }
    else if (!leftMode)  // to the right
    {
        if (forceVec.getX() < 0 && forceVec.getZ() < 0)  // v<-
        {
            forceVec[0] = (topForce - FORCE_THRESHOLD) * 0.002;
            forceVec[2] = (topForce - FORCE_THRESHOLD) * relativeWeakForce * 0.002 * -1;
        }
        else if (forceVec.getX() < 0 && forceVec.getZ() > 0)  // ^<-
        {
            forceVec[0] = (topForce - FORCE_THRESHOLD) * relativeWeakForce * 0.002 * -1;
            forceVec[2] = (topForce - FORCE_THRESHOLD) * 0.002;
        }
        else if (forceVec.getX() > 0 && forceVec.getZ() > 0)  // ->^
        {
            forceVec[0] = (topForce - FORCE_THRESHOLD) * 0.002;
            forceVec[2] = (topForce - FORCE_THRESHOLD) * relativeWeakForce * 0.002 * -1;
        }
        else if (forceVec.getX() > 0 && forceVec.getZ() < 0)  // ->v
        {
            forceVec[0] = (topForce - FORCE_THRESHOLD) * relativeWeakForce * 0.002 * -1;
            forceVec[2] = (topForce - FORCE_THRESHOLD) * 0.002;
        }
        else
        {  // move in in opposite direction of last contact force
            forceVec[0] *= -0.002;
            forceVec[1] *= -0.002;
            forceVec[2] *= -0.002;
        }
    }

    ROS_ERROR("forceVec values after rule %4.4lf %4.4lf %4.4lf", forceVec.getX(), forceVec.getY(), forceVec.getZ());
    forceVec[1] = 0;
    ruleTangent.vector.x = forceVec[0];
    ruleTangent.vector.y = forceVec[1];
    ruleTangent.vector.z = forceVec[2];
    return ruleTangent;
}

tf::Vector3 kuka_sphere_object_shape_detection::PointOnSphere()
{
    geometry_msgs::Vector3Stamped TCPFVecStamped;
    geometry_msgs::Vector3Stamped SphereVec;
    tf::StampedTransform stamped_transform;
    TCPFVecStamped.header.frame_id = "wirelessATI";
    TCPFVecStamped.header.stamp = ros::Time::now();
    TCPFVecStamped.vector.x =
        externalTCPWrenchVec.getX() * -1;  // *1 = in direction of force; *-1 opposite direction of force
    TCPFVecStamped.vector.y = externalTCPWrenchVec.getY() * -1;
    TCPFVecStamped.vector.z = externalTCPWrenchVec.getZ() * -1;
    try
    {
        tfl->waitForTransform("sphere", "wirelessATI", ros::Time::now(), ros::Duration(1));  // kuka_fri_tcp
        tfl->lookupTransform("sphere", "wirelessATI", ros::Time(0), stamped_transform);
        tfl->transformVector("sphere", ros::Time(0), TCPFVecStamped, "wirelessATI", SphereVec);
    }
    catch (tf::TransformException& exception)
    {
        ROS_ERROR("Transform failed: %s", exception.what());
    }
    // get point in SphereFrame
    tf::Vector3 vec3normalized;
    vec3normalized[0] = SphereVec.vector.x;
    vec3normalized[1] = SphereVec.vector.y;
    vec3normalized[2] = SphereVec.vector.z;
    vec3normalized.normalize();

    // Position of point acting on sphere in Sphereframe;
    double radius = 0.005;  // 5mm
    tf::Vector3 PointOnSphere;
    PointOnSphere = (vec3normalized)*radius;
    ROS_ERROR("PointOnSphere x y z %4.4lf %4.4lf %4.4lf", PointOnSphere.getX(), PointOnSphere.getY(),
              PointOnSphere.getZ());

    buildOctomap(PointOnSphere);
    return PointOnSphere;
}

tf::Vector3 kuka_sphere_object_shape_detection::TangentVector()
{
    tf::Vector3 TVecT0;
    TVecT0[0] = 0;
    TVecT0[1] = 0;
    TVecT0[2] = 0;
    if (FORCE_THRESHOLD < externalTCPWrenchVec.getX() || FORCE_THRESHOLD < externalTCPWrenchVec.getY() ||
        FORCE_THRESHOLD < externalTCPWrenchVec.getZ() || -FORCE_THRESHOLD > externalTCPWrenchVec.getX() ||
        -FORCE_THRESHOLD > externalTCPWrenchVec.getY() || -FORCE_THRESHOLD > externalTCPWrenchVec.getZ())
    {
        geometry_msgs::Vector3Stamped TVecTCP;
        TVecTCP.header.frame_id = "wirelessATI";
        TVecTCP.header.stamp = ros::Time::now();
        TVecTCP.vector.x = externalTCPWrenchVec.getX();
        TVecTCP.vector.y = externalTCPWrenchVec.getY();
        TVecTCP.vector.z = externalTCPWrenchVec.getZ();
        tf::StampedTransform tcpToWorldStamp;
        geometry_msgs::Vector3Stamped TVecWorld;
        try
        {
            tfl->waitForTransform("world", "wirelessATI", ros::Time::now(), ros::Duration(1));
            tfl->lookupTransform("world", "wirelessATI", ros::Time(0), tcpToWorldStamp);
            tfl->transformVector("world", ros::Time(0), TVecTCP, "wirelessATI", TVecWorld);
        }
        catch (tf::TransformException& exception)
        {
            ROS_ERROR("Transform failed: %s", exception.what());
        }

        // for TranslationRule
        // tf::Vector3 TVecNorm;
        // TVecNorm[0] = TVecWorld.vector.x;
        // TVecNorm[1] = TVecWorld.vector.y;
        // TVecNorm[2] = TVecWorld.vector.z;
        // TVecNorm.normalize();

        // for TranslationRUleB
        tf::Vector3 TVec;
        TVec[0] = TVecWorld.vector.x;
        TVec[1] = TVecWorld.vector.y;
        TVec[2] = TVecWorld.vector.z;
        ROS_ERROR("externalTCPWrench in world %4.4lf %4.4lf %4.4lf", TVec[0], TVec[1], TVec[2]);

        // geometry_msgs::Vector3Stamped RuledTVecStampFT = TranslationRule(TVecNorm);
        geometry_msgs::Vector3Stamped RuledTVecStampFT = TranslationRuleB(TVec);
        ROS_ERROR("Bewegungsrichtung world %4.4lf %4.4lf %4.4lf", RuledTVecStampFT.vector.x, RuledTVecStampFT.vector.y,
                  RuledTVecStampFT.vector.z);
        geometry_msgs::Vector3Stamped TVecStamp;
        RuledTVecStampFT.header.frame_id = "world";
        RuledTVecStampFT.header.stamp = ros::Time::now();
        tf::StampedTransform sunriseToT0;
        try
        {
            tfl->waitForTransform("sphere", "world", ros::Time::now(), ros::Duration(1));
            tfl->lookupTransform("sphere", "world", ros::Time(0), sunriseToT0);
            tfl->transformVector("sphere", ros::Time(0), RuledTVecStampFT, "world", TVecStamp);
        }
        catch (tf::TransformException& exception)
        {
            ROS_ERROR("Transform failed: %s", exception.what());
        }
        TVecT0[0] = TVecStamp.vector.x;
        TVecT0[1] = TVecStamp.vector.y;
        TVecT0[2] = TVecStamp.vector.z;
    }
    return TVecT0;
}

void kuka_sphere_object_shape_detection::changeModeResetMotion()
{
    bool OneTimeBool;
    OneTimeBool = false;

    Eigen::Isometry3d spherePosition = robot_state->getGlobalLinkTransform("sphere");
    tf::Transform spherePositionTransformGoal;
    tf::transformEigenToTF(spherePosition, spherePositionTransformGoal);
    tf::Transform spherePositionTransform = spherePositionTransformGoal;
    tf::Quaternion quatRight(0.87, 0.01, -0.48, -0.016);
    if (firstItCMRM)
    {
        OneTimeBool = true;
        spherePositionTransformGoal.getOrigin().setY(spherePositionTransformGoal.getOrigin().getY() + 0.01);
        spherePositionTransformGoal.getOrigin().setZ(startPosition.getOrigin().getZ());
        firstItCMRM = false;
    }
    if (leftMode)
    {
        if (OneTimeBool)
        {
            // spherePositionTransformGoal.setRotation(startPosition.getRotation());
            spherePositionTransformGoal.getOrigin().setX(spherePositionTransformGoal.getOrigin().getX() + 0.02);
            OneTimeBool = false;
        }
    }
    else if (!leftMode)
    {
        if (OneTimeBool)
        {
            // spherePositionTransformGoal.setRotation(quatRight);
            spherePositionTransformGoal.getOrigin().setX(spherePositionTransformGoal.getOrigin().getX() - 0.02);
            OneTimeBool = false;
        }
    }

    // debugging
    Eigen::Isometry3d updatedSpherePosition2;
    updatedSpherePosition2 = robot_state->getGlobalLinkTransform("sphere");
    printTransform("current sphere position ", updatedSpherePosition2);
    printTransform("sphere position goal changeModeReset modus:", spherePositionTransformGoal);
    // end debugging

    Eigen::Isometry3d updatedSpherePose;
    tf::transformTFToEigen(spherePositionTransformGoal, updatedSpherePose);
    std::vector<double> ik_angles;
    ik_angles.resize(NUMBER_OF_JOINTS);
    robot_state->setJointGroupPositions(joint_model_group, jointAngles);

    ros::Time t3 = ros::Time::now();
    bool found_ik = robot_state->setFromIK(joint_model_group, updatedSpherePose, "sphere", 5, 0.02);
    ros::Time t4 = ros::Time::now();
    ROS_INFO("IK solution found %d, solver took %7.3lf msec", found_ik, 1000 * (t4 - t3).toSec());

    if (found_ik)
    {
        robot_state->copyJointGroupPositions(joint_model_group, ik_angles);
        for (int j = 0; j < NUMBER_OF_JOINTS; j++)
        {
            targetVelocities[j] = zeroVelocities[j];
            targetAngles[j] = ik_angles[j];
        }
        if (!OneTimeBool)
        {
            modeChange = false;  // change back to searchObjectGoal
        }
    }
    currentYInitialized = false;
    publishPositionGoal();
    ros::Duration(5).sleep();
}

tf::Vector3 kuka_sphere_object_shape_detection::reapproachObject()
{
    geometry_msgs::Vector3Stamped latestContactForceATI;
    geometry_msgs::Vector3Stamped latestContactForceWorld;
    latestContactForceATI.header.frame_id = "wirelessATI";
    latestContactForceATI.header.stamp = ros::Time::now();
    latestContactForceATI.vector.x = latestContactForce.getX();
    latestContactForceATI.vector.y = latestContactForce.getY();
    latestContactForceATI.vector.z = latestContactForce.getZ();
    tf::StampedTransform atiToWorld;
    try
    {
        tfl->waitForTransform("world", "wirelessATI", ros::Time::now(), ros::Duration(1));
        tfl->lookupTransform("world", "wirelessATI", ros::Time(0), atiToWorld);
        tfl->transformVector("world", ros::Time(0), latestContactForceATI, "wirelessATI", latestContactForceWorld);
    }
    catch (tf::TransformException& exception)
    {
        ROS_ERROR("Transform failed: %s", exception.what());
    }

    tf::Vector3 contactForce;
    contactForce[0] = latestContactForceWorld.vector.x;
    contactForce[1] = latestContactForceWorld.vector.y;
    contactForce[2] = latestContactForceWorld.vector.z;
    contactForce.normalize();
    contactForce[0] = 0;
    contactForce[1] = 0;  // /=200;
    contactForce[2] /= 50;
    // ROS_ERROR("reapproach direction %4.4lf %4.4lf %4.4lf", contactForce[0] *-1, contactForce[1] *-1, contactForce[2] *-1);
    return contactForce;
}

void kuka_sphere_object_shape_detection::searchObjectGoal()
{
    robot_state->update();  // is this even necessary?!
    Eigen::Isometry3d spherePosition = robot_state->getGlobalLinkTransform("sphere");

    tf::Transform spherePositionTransform;
    tf::transformEigenToTF(spherePosition, spherePositionTransform);
    tf::Vector3 workspaceVec;
    workspaceVec = spherePositionTransform.getOrigin();
    checkWorkspace(workspaceVec);

    if (!currentYInitialized)
    {
        currentY = spherePositionTransform.getOrigin().getY();
        currentYInitialized = true;
    }

    if (FORCE_THRESHOLD < externalTCPWrenchVec[0] || FORCE_THRESHOLD < externalTCPWrenchVec[1] ||
        FORCE_THRESHOLD < externalTCPWrenchVec[2] || -FORCE_THRESHOLD > externalTCPWrenchVec[0] ||
        -FORCE_THRESHOLD > externalTCPWrenchVec[1] || -FORCE_THRESHOLD > externalTCPWrenchVec[2])
    {
        robot_state->setJointGroupPositions(joint_model_group, jointAngles);

        // only for force orientation purposes
        geometry_msgs::Vector3Stamped forceOrientationATI;
        forceOrientationATI.header.frame_id = "wirelessATI";
        forceOrientationATI.header.stamp = ros::Time::now();
        forceOrientationATI.vector.x = externalTCPWrenchVec[0];
        forceOrientationATI.vector.y = externalTCPWrenchVec[1];
        forceOrientationATI.vector.z = externalTCPWrenchVec[2];
        tf::StampedTransform atiToWorldStamp;
        geometry_msgs::Vector3Stamped forceInWorld;
        try
        {
            tfl->waitForTransform("world", "wirelessATI", ros::Time::now(), ros::Duration(1));
            tfl->lookupTransform("world", "wirelessATI", ros::Time(0), atiToWorldStamp);
            tfl->transformVector("world", ros::Time(0), forceOrientationATI, "wirelessATI", forceInWorld);
        }
        catch (tf::TransformException& exception)
        {
            ROS_ERROR("Transform failed: %s", exception.what());
        }
        ROS_ERROR("Force in World: x y z: %4.4lf  %4.4lf  %4.4lf", forceInWorld.vector.x, forceInWorld.vector.y,
                  forceInWorld.vector.z);
        // end: only for force orientation purposes

        reapproachCounter = 0;
        latestContactForceInitialized = true;
        latestContactForce = externalTCPWrenchVec;
        tf::Vector3 VecSpherePoint = PointOnSphere();
        tf::Vector3 VecTangent = TangentVector();
        // tf::Vector3 SpherePoint = VecSpherePoint + VecTangent; //SphereFrame
        // ROS_ERROR("VecSpherePoint + VecTangent %4.4lf %4.4lf %4.4lf", SpherePoint[0], SpherePoint[1], SpherePoint[2]);

        // calculate SpherePoint vector to world frame for better understanding
        geometry_msgs::Vector3Stamped whatevA;
        geometry_msgs::Vector3Stamped whatevB;
        whatevA.header.frame_id = "sphere";
        whatevA.header.stamp = ros::Time::now();
        whatevA.vector.x = VecTangent.getX();
        whatevA.vector.y = VecTangent.getY();
        whatevA.vector.z = VecTangent.getZ();
        tf::StampedTransform sunriseToT0;
        try
        {
            tfl->waitForTransform("world", "sphere", ros::Time::now(), ros::Duration(1));
            tfl->lookupTransform("world", "sphere", ros::Time(0), sunriseToT0);
            tfl->transformVector("world", ros::Time(0), whatevA, "sphere", whatevB);
        }
        catch (tf::TransformException& exception)
        {
            ROS_ERROR("Transform failed: %s", exception.what());
        }
        // ROS_ERROR_STREAM("change in world end point " << whatevB.vector);

        // calculate rotation to make axis z point towards point on sphere
        // no rotation between T6 and SphereFrame in robot model
        Eigen::Quaternion<double> quat;
        Eigen::Vector3d sphereZ;

        sphereZ[0] = 0;
        sphereZ[1] = 0;
        sphereZ[2] = 1;
        Eigen::Vector3d PointOnSphereVec;
        PointOnSphereVec[0] = VecSpherePoint.getX();
        PointOnSphereVec[1] = VecSpherePoint.getY();
        PointOnSphereVec[2] = VecSpherePoint.getZ();
        quat.setFromTwoVectors(sphereZ, PointOnSphereVec);

        // comment in for orientation change
        tf::Quaternion newT0GoalQuat(quat.x(), quat.y(), quat.z(), quat.w());
        // spherePositionTransform.setRotation(newT0GoalQuat);

        whatevB.vector.y = currentY;
        tf::Vector3 whatevBOrigin;
        whatevBOrigin[0] = whatevB.vector.x;
        whatevBOrigin[1] = 0;
        whatevBOrigin[2] = whatevB.vector.z;
        spherePositionTransform.setOrigin(spherePositionTransform.getOrigin() + whatevBOrigin);  // world here

        // debugging
        Eigen::Isometry3d updatedSpherePosition2;
        updatedSpherePosition2 = robot_state->getGlobalLinkTransform("sphere");
        printTransform("current sphere position ", updatedSpherePosition2);
        printTransform("spherePosition Goal force modus:", spherePositionTransform);
        // debugging end

        Eigen::Isometry3d updatedSpherePose;
        tf::transformTFToEigen(spherePositionTransform, updatedSpherePose);
        std::vector<double> ik_angles;
        ik_angles.resize(NUMBER_OF_JOINTS);
        robot_state->setJointGroupPositions(joint_model_group, jointAngles);

        ros::Time t3 = ros::Time::now();
        bool found_ik = robot_state->setFromIK(joint_model_group, updatedSpherePose, "sphere", 5, 0.02);
        ros::Time t4 = ros::Time::now();
        ROS_INFO("IK solution found %d, solver took %7.3lf msec", found_ik, 1000 * (t4 - t3).toSec());

        if (found_ik)
        {
            robot_state->copyJointGroupPositions(joint_model_group, ik_angles);
            for (int j = 0; j < NUMBER_OF_JOINTS; j++)
            {
                targetVelocities[j] = zeroVelocities[j];
                targetAngles[j] = ik_angles[j];
            }
            // forceGoalPoseReached = false;
            publishPositionGoal();
        }
        else if (!found_ik)
        {
            ROS_ERROR("No IK found");
        }
    }

    else
    {
        // if (latestContactForceInitialized)
        // {
        //   if (reapproachCounter < 10)
        //   {
        //     spherePositionTransform.setOrigin(spherePositionTransform.getOrigin() - reapproachObject());
        //     reapproachCounter += 1;
        //   }
        //   else
        //   {
        //     spherePositionTransform.getOrigin().setX(startPosition.getOrigin().getX());
        //     spherePositionTransform.getOrigin().setZ(startPosition.getOrigin().getZ());
        //     latestContactForceInitialized = false;
        //     reapproachCounter = 0;
        //   }
        //   ROS_ERROR("reapproachCounter: %i", reapproachCounter);
        // }

        // reach goal triggered by force and then go on with "normal mode"
        // if(!forceGoalPoseReached)
        //{
        //  bool reached = true;
        //  for( int j=0; j < NUMBER_OF_JOINTS; j++ ) {
        //    if (!((jointAngles[j] - 0.05) <= targetAngles[j] <= (jointAngles[j] + 0.05)))
        //    {
        //      reached = false;
        //    }
        //  }
        //  if (reached)
        //  {
        //    forceGoalPoseReached = true;
        //  }
        //  else
        //  {
        //    return;
        //  }
        //}

        if (leftMode)
        {
            spherePositionTransform.getOrigin().setX(spherePositionTransform.getOrigin().getX() + 0.001);
        }
        else if (!leftMode)
        {
            spherePositionTransform.getOrigin().setX(spherePositionTransform.getOrigin().getX() - 0.001);
        }
        spherePositionTransform.getOrigin().setY(currentY);

        // debugging
        Eigen::Isometry3d updatedSpherePosition2;
        updatedSpherePosition2 = robot_state->getGlobalLinkTransform("sphere");
        printTransform("current sphere position ", updatedSpherePosition2);
        printTransform("sphere goal position normal mode", spherePositionTransform);
        // end debugging

        Eigen::Isometry3d updatedSpherePose;
        tf::transformTFToEigen(spherePositionTransform, updatedSpherePose);
        std::vector<double> ik_angles;
        ik_angles.resize(NUMBER_OF_JOINTS);
        robot_state->setJointGroupPositions(joint_model_group, jointAngles);
        bool found_ik = robot_state->setFromIK(joint_model_group, updatedSpherePose, "sphere", 1, 0.1);
        if (found_ik)
        {
            robot_state->copyJointGroupPositions(joint_model_group, ik_angles);
            for (int j = 0; j < NUMBER_OF_JOINTS; j++)
            {
                targetVelocities[j] = zeroVelocities[j];
                targetAngles[j] = ik_angles[j];
            }
            publishPositionGoal();
        }
        else
        {
            ROS_ERROR("No ik found");
        }
    }
}

void kuka_sphere_object_shape_detection::publishStartGoal()
{
    // 1.718039365067169, 1.0749976765306883, 1.1494410808961995, 1.4491305027889763, 0.24388065576387913,
    // -1.9058869195474655, -0.27576307740655404 (wsg angles: -0.02885, 0.02885 -> open?) double startAngles[] =
    // {1.72, 1.075, 1.15, 1.45, 0.244, -1.906, -0.276}; version 1 in y directions
    double startAngles[] = { -1.24, -0.64, 0.20, 1.695, 2.24, 1.48, -0.62 };  // version 2 in z directions

    sensor_msgs::JointState msg;
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = "world";
    // msg.header.seq = nk_teleop_seq++; // FIXME: the 0-1-0-1 allows us to track header.seq in rqt_plot
    msg.name = jointNames;

    ros_fri_msgs::RMLPositionInputParameters ppp;
    ppp.header = msg.header;
    ppp.TargetPositionVector.resize(NUMBER_OF_JOINTS);
    ppp.TargetVelocityVector.resize(NUMBER_OF_JOINTS);
    ppp.MaxVelocityVector.resize(NUMBER_OF_JOINTS);
    ppp.MaxAccelerationVector.resize(NUMBER_OF_JOINTS);

    pthread_mutex_lock(&nodeStateMutex);
    for (unsigned int i = 0; i < jointNames.size(); i++)
    {
        msg.position.push_back(startAngles[i]);
    }
    // msg.velocity = targetVelocities;
    msg.velocity = zeroVelocities;
    // msg.effort = NaN;
    // msg.effort.resize( 7 ); msg.effort[6] = 0.02;// * (nk_teleop_seq & 0x1);

    for (int j = 0; j < NUMBER_OF_JOINTS; j++)
    {
        ppp.TargetPositionVector[j] = msg.position[j];
        ppp.TargetVelocityVector[j] = msg.velocity[j];
        ppp.MaxVelocityVector[j] = jointVelocityLimits[j];
        ppp.MaxAccelerationVector[j] = jointAccelerationLimits[j];
    }

    pthread_mutex_unlock(&nodeStateMutex);

    Eigen::Isometry3d spherePosition = robot_state->getGlobalLinkTransform("sphere");
    printTransform("NOA: initial world->sphere", spherePosition);

    // ROS_ERROR(" jointAngles before startAngles %4.4lf %4.4lf %4.4lf %4.4lf %4.4lf %4.4lf",
    //           jointAngles[0], jointAngles[1], jointAngles[2], jointAngles[3],
    //           jointAngles[4], jointAngles[5]);

    positionGoalPublisher.publish(msg);
    rmlPositionGoalPublisher.publish(ppp);

    robot_state->setJointGroupPositions(joint_model_group, startAngles);
    spherePosition = robot_state->getGlobalLinkTransform("sphere");

    std::vector<double> joint_values;
    robot_state->copyJointGroupPositions(joint_model_group, joint_values);
    for (std::size_t i = 0; i < jointNames.size(); ++i)
    {
        ROS_INFO("Joint at startPosition %s: %f", jointNames[i].c_str(), joint_values[i]);
    }
    tf::transformEigenToTF(spherePosition, startPosition);
    startPositionInitialized = true;

    // wait for start position to be reached
    ros::Duration(7).sleep();
}

void kuka_sphere_object_shape_detection::publishStopRobot(bool stop)
{
    std_msgs::Bool msg;
    msg.data = stop;
    stopRobotPublisher.publish(msg);
}

void kuka_sphere_object_shape_detection::publishEmergencyStop(bool b)
{
    ROS_WARN("publishEmergencyStop: IMPLEMENT ME!");
    std_msgs::Bool msg;
    msg.data = b;
    emergencyStopPublisher.publish(msg);
}

void kuka_sphere_object_shape_detection::publishCancelTrajectory()
{
}

void kuka_sphere_object_shape_detection::publishPositionGoal()
{
    sensor_msgs::JointState msg;
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = "world";
    // msg.header.seq = nk_teleop_seq++; // FIXME: the 0-1-0-1 allows us to track header.seq in rqt_plot
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
    msg.effort[6] = 0.02;  // * (nk_teleop_seq & 0x1);

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

void kuka_sphere_object_shape_detection::publishVelocityGoal()
{
    sensor_msgs::JointState msg;
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = "world";
    // msg.header.seq = nk_teleop_seq++;
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

int kuka_sphere_object_shape_detection::run()
{
    ros::Rate loopRate(commandRate);
    ROS_INFO("loop rate is %lf: ", commandRate);

    // waiting until joint states are published
    unsigned int secs = 0;
    boost::shared_ptr<sensor_msgs::JointState const> msgPtr;
    while (ros::ok())
    {
        msgPtr = ros::topic::waitForMessage<sensor_msgs::JointState>("/lwr/joint_states", ros::Duration(1));
        if (msgPtr != NULL)
        {
            ROS_INFO("... joint states ok.");
            break;
        }
        secs++;
        ROS_INFO("... waiting for /joint_states (%u secs)...", secs);
    }

    // waiting until wrench is published
    secs = 0;
    boost::shared_ptr<geometry_msgs::WrenchStamped const> wrPtr;
    while (ros::ok())
    {
        wrPtr = ros::topic::waitForMessage<geometry_msgs::WrenchStamped>("/lwr/estimatedExternalTcpWrench",
                                                                         ros::Duration(1));
        if (wrPtr != NULL)
        {
            ROS_INFO("... /wrench ok.");
            break;
        }
        secs++;
        ROS_INFO("... waiting for /wrench (%u secs)...", secs);
    }

    // main loop

    ros::Rate rate(60.0);
    int counter;
    while (ros::ok())
    {
        rate.sleep();
        // counter++;
        // ROS_ERROR("%i %zu", counter, ros::Time::now().toNSec());
        ros::spinOnce();
        if (!startPositionInitialized)
        {
            ROS_ERROR("calling generate start goal.");
            publishStartGoal();
            // this sleeps for several(5) seconds
        }
        else if (startPositionInitialized)
        {
            if (inside)
            {
                if (!modeChange)
                {
                    searchObjectGoal();
                }
                else
                {
                    changeModeResetMotion();
                }
            }
        }
    };
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "kuka_sphere_object_shape_detection");  // 1=no default SIGINT handler

    kuka_sphere_object_shape_detection ksosd;
    ksosd.run();

    exit(EXIT_SUCCESS);
}
