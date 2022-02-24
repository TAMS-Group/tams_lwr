/** grav_comp_joints.cpp: joint-space gravity compensation mode
 * for the KuKA LWR, based on the estimated external torques.
 *
 * This program subscribes to joint_state and estimated external
 * joint torques from the KuKA LWR (controlled by ROS-FRI node)
 * and generates joint-space motion goals for gravity compensation.
 * Maximum speed can be configured, but should be suitably low
 * to avoid damage to the robot or humans around the robot.
 *
 * WARNING: this program relies on accurate payload information
 * for the LWR; with a wrong payload the estimated external
 * torques are off, resulting in unwanted robot motion.
 *
 * Note that this node respects the joint limits of the LWR,
 * but does NOT check for external collisions. Given that a
 * collision results in corresponding torque, the robot will
 * try to avoid collisions anyway.
 *
 * 2017.01 19 - linear velocity scaling
 * 2017.01.12 - created
 */

#include <math.h>
#include <vector>
#include <map>

#include <ros/ros.h>
#include <ros_fri_msgs/RMLPositionInputParameters.h>
#include <ros_fri_msgs/RMLVelocityInputParameters.h>
#include <sensor_msgs/JointState.h>

/*
std_msgs/Header header
  uint32 seq
  time stamp
  string frame_id
float64[] TargetPositionVector
float64[] TargetVelocityVector
float64[] MaxAccelerationVector
float64[] MaxVelocityVector
*/

#define NUMJ 7
#define DEG2RAD (M_PI / 180.0)

pthread_mutex_t mutex;

int n_joint_state_messages = 0;
int n_external_efforts_messages = 0;

std::vector<double> jointAngles;      // current LWR joint angles, radians
std::vector<double> jointVelocities;  // current LWR joint angles, radians
std::vector<double> jointEfforts;     // current LWR joint angles, radians

std::vector<double> effortThresholds;  // torque threadholds
std::vector<double> externalEfforts;   // estimated external joint torques
std::vector<double> goalVelocities;    // estimated external joint torques
std::vector<double> gains;             // gain factor: torque -> velocity

std::map<std::string, int> jointIndexMap;

double command_rate;
double averaging_weight;
std::string world_frame;
std::string tf_prefix;
std::string tool_frame;
std::string robot_frame;
std::string effort_thresholds_tokens;
std::string goal_velocities_tokens;
std::string gains_tokens;

/**
 * prints help/usage info and exists.
 */
int usage()
{
    ROS_INFO("Usage: lwr_grav_comp [params]\n");
    ROS_INFO("Example: FIXME \n");
    exit(1);
}

void parseAsVector(std::vector<double>& data, std::string tokens)
{
    double a, b, c, d, e, f, g;
    int n = sscanf(tokens.c_str(), "%lf %lf %lf %lf %lf %lf %lf", &a, &b, &c, &d, &e, &f, &g);
    if (n != NUMJ)
    {
        ROS_ERROR("Failed to parse initialization data '%s', found %d tokens.", tokens.c_str(), n);
        exit(1);
    }

    data.resize(NUMJ);
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

double clampedDeadbandLinear(double x, double deadband, double gain, double limit)
{
    if (fabs(x) <= deadband)
        return 0;
    double x2, x3;
    if (x >= deadband)
    {
        x2 = (x - deadband) * gain;
        x3 = clamp(x2, 0.0, limit);
    }
    if (x <= deadband)
    {
        x2 = (x + deadband) * gain;
        x3 = clamp(x2, -limit, 0.0);
    }
    ROS_INFO("clampedDeadbandLinear dead %8.4lf gain %8.4lf clamp %8.4lf x %8.4lf result %8.4lf", deadband, gain, limit,
             x, x3);
    return x3;
}

double exponentialAverage(double current, double previous, double weight)
{
    double result = current * weight + (1 - weight) * previous;
    ROS_INFO("exponentialAverage    prev %8.4lf wght %8.4lf                x %8.4lf result %8.4lf", previous, weight,
             current, result);
    return result;
}

void jointStateCallback(const sensor_msgs::JointState jointState)
{
    pthread_mutex_lock(&mutex);

    // ROS_WARN( "jointStateCallback %d", n_joint_state_messages );
    unsigned int matched_lwr_joints = 0;
    std::vector<double> tmpAngles;
    tmpAngles.resize(NUMJ);
    std::vector<double> tmpVelocities;
    tmpVelocities.resize(NUMJ);
    std::vector<double> tmpEfforts;
    tmpEfforts.resize(NUMJ);

    for (unsigned int i = 0; i < jointState.name.size(); i++)
    {
        std::string name = jointState.name[i];
        int found = jointIndexMap.count(name);
        // ROS_INFO( "parsing joint '%s' found %d", name.c_str(), found );
        if (found > 0)
        {
            int lwrIndex = jointIndexMap[name];
            tmpAngles[lwrIndex] = jointState.position[i];
            tmpVelocities[lwrIndex] = jointState.velocity[i];
            tmpEfforts[lwrIndex] = jointState.effort[i];
            matched_lwr_joints++;
        }
        else
        {  // ignore unknown joints
            ROS_INFO("lwr_grav_comp: unmatched joint name '%s'", name.c_str());
        }
    }

    if (matched_lwr_joints == NUMJ)
    {
        n_joint_state_messages++;
        jointAngles = tmpAngles;
        jointVelocities = tmpVelocities;
        jointEfforts = tmpEfforts;
    }
    pthread_mutex_unlock(&mutex);
}

void externalTorquesCallback(const sensor_msgs::JointState jointState)
{
    pthread_mutex_lock(&mutex);

    // ROS_WARN( "externalTorquesCallback %d", n_joint_state_messages );
    unsigned int matched_lwr_joints = 0;
    std::vector<double> tmpEfforts;
    tmpEfforts.resize(NUMJ);

    for (unsigned int i = 0; i < jointState.name.size(); i++)
    {
        std::string name = jointState.name[i];
        int found = jointIndexMap.count(name);
        // ROS_INFO( "parsing joint '%s' found %d", name.c_str(), found );
        if (found > 0)
        {
            int lwrIndex = jointIndexMap[name];
            tmpEfforts[lwrIndex] = jointState.effort[i];
            matched_lwr_joints++;
        }
        else
        {  // ignore unknown joints
            ROS_INFO("lwr_grav_comp: unmatched torques joint name '%s'", name.c_str());
        }
    }

    if (matched_lwr_joints == NUMJ)
    {
        n_external_efforts_messages++;
        externalEfforts = tmpEfforts;
    }
    pthread_mutex_unlock(&mutex);
}

int main(int argc, char** argv)
{
    // phase 1: create mutex and initialize ROS
    //
    if (0 != pthread_mutex_init(&mutex, NULL))
    {
        ROS_ERROR("pthread_mutex_init failed.");
        exit(1);
    }
    ros::init(argc, argv, "lwr_grav_comp", 1);

    // phase 2: create a NodeHandle and an async spinner.
    //
    ros::NodeHandle nh("grav_comp");
    ros::NodeHandle nnh("~");
    jointAngles.resize(NUMJ);
    jointVelocities.resize(NUMJ);
    jointEfforts.resize(NUMJ);

    externalEfforts.resize(NUMJ);
    goalVelocities.resize(NUMJ);

    jointIndexMap["lwr_arm_0_joint"] = 0;
    jointIndexMap["lwr_arm_1_joint"] = 1;
    jointIndexMap["lwr_arm_2_joint"] = 2;
    jointIndexMap["lwr_arm_3_joint"] = 3;
    jointIndexMap["lwr_arm_4_joint"] = 4;
    jointIndexMap["lwr_arm_5_joint"] = 5;
    jointIndexMap["lwr_arm_6_joint"] = 6;

    nh.param("command_rate", command_rate, 50.0);  // 10.0 Hz
    nh.param("tool_frame", tool_frame, std::string("tcp"));
    nh.param("robot_frame", robot_frame, std::string("lwr_arm_base_link"));
    nh.param("effort_thresholds", effort_thresholds_tokens, std::string("1 1 1 1 0.4 0.4 0.4"));  // Nm
    nh.param("gains", gains_tokens, std::string("0.2 0.2 0.4 0.4 0.4 1.0 1.0"));
    nh.param("goal_velocities", goal_velocities_tokens, std::string("0.5 0.5 0.5 0.5 0.5 0.6 0.6"));  // rad/s
    nh.param("averaging_weight", averaging_weight, 0.95);

    parseAsVector(effortThresholds, effort_thresholds_tokens);
    parseAsVector(goalVelocities, goal_velocities_tokens);
    parseAsVector(gains, gains_tokens);

    ROS_INFO("lwr_grav_comp: node params are:");
    ROS_INFO("command rate is %8.4f", command_rate);
    ROS_INFO("joint  torque deadband  gains   max velocities");
    for (int j = 0; j < NUMJ; j++)
    {
        ROS_INFO("%8d  %10.4lf  %10.4lf  %10.4lf", j, effortThresholds[j], gains[j], goalVelocities[j]);
    }

    // phase 3: subscribe to /lwr/joint_states and wait for
    // 10 messages, to ensure that the robot is up and running
    //
    ros::Subscriber jointStateSubscriber =
        nnh.subscribe<const sensor_msgs::JointState>("/lwr/joint_states", 1, &jointStateCallback);
    bool have_messages = false;
    double wait_time = 0;
    double max_wait_time = 5;
    while (!have_messages)
    {
        ros::spinOnce();
        if (n_joint_state_messages >= 10)
            break;  // ok, robot is alive
        usleep(50 * 1000);
        wait_time += 0.05;
        if (wait_time >= max_wait_time)
        {
            ROS_ERROR("lwr_grav_comp: no (valid) /lwr/joint_states messages after %lf secs.", max_wait_time);
            exit(2);
        }
    }
    ROS_INFO("lwr_grav_comp: subscribed to /lwr/joint_states.");

    ros::Subscriber externalEffortsSubscriber =
        nnh.subscribe<const sensor_msgs::JointState>("/lwr/estimatedExternalJointTorques", 1, &externalTorquesCallback);
    have_messages = false;
    wait_time = 0;
    while (!have_messages)
    {
        ros::spinOnce();
        if (n_external_efforts_messages >= 10)
            break;  // ok, robot is alive
        usleep(50 * 1000);
        wait_time += 0.05;
        if (wait_time >= max_wait_time)
        {
            ROS_ERROR("lwr_grav_comp: no (valid) /lwr/estimatedExternalJointTorques messages after %lf secs.",
                      max_wait_time);
            exit(2);
        }
    }
    ROS_INFO("lwr_grav_comp: subscribed to /lwr/estimatedExternalJointTorques");

    ros::Publisher jntVelGoalPublisher =
        nnh.advertise<ros_fri_msgs::RMLVelocityInputParameters>("/lwr/jointVelocityGoal", 1);
    usleep(500 * 1000);  // give roscore and subscribers a chance

    // phase 3: ros::spin. Read estimated torques, and try
    // to move away from all joints whose external torque
    // is higher than the given threshold.
    //
    ros::Rate rate(command_rate);
    bool done = false;
    unsigned long iteration = 0;
    std::vector<double> commandedVelocities;
    commandedVelocities.resize(NUMJ);

    while (!done && ros::ok())
    {
        for (int j = 0; j < NUMJ; j++)
        {
            /*
            if (externalEfforts[j] > effortThresholds[j]) {
              commandedVelocities[j] = -goalVelocities[j];
            }
            else if (externalEfforts[j] < -effortThresholds[j]) {
              commandedVelocities[j] = goalVelocities[j];
            }
            else {
             commandedVelocities[j] = 0.0;
            }
            */
            double v = -1 * clampedDeadbandLinear(externalEfforts[j], effortThresholds[j], gains[j], goalVelocities[j]);
            double w = exponentialAverage(v, commandedVelocities[j], averaging_weight);
            if (fabs(w) < 0.0001)
                w = 0.0;

            commandedVelocities[j] = w;
        }

        ROS_WARN("moving to target position...");
        ros_fri_msgs::RMLVelocityInputParameters rmlVelGoal;
        rmlVelGoal.header.seq = 1;
        rmlVelGoal.header.stamp = ros::Time::now();
        rmlVelGoal.header.frame_id = "lwr_base_link";  // FIXME
        rmlVelGoal.TargetVelocityVector.resize(NUMJ);
        rmlVelGoal.MaxAccelerationVector.resize(NUMJ);
        // rmlVelGoal.MaxVelocityVector.resize( NUMJ );

        for (int j = 0; j < NUMJ; j++)
        {
            rmlVelGoal.TargetVelocityVector[j] = commandedVelocities[j];
            rmlVelGoal.MaxAccelerationVector[j] = (j <= 3) ? 0.3 : 0.5;
            // rmlVelGoal.MaxVelocityVector[j] = 0.1;
        }

        ROS_INFO("vel goal is %8.2lf %8.2lf %8.2lf %8.2lf %8.2lf %8.2lf %8.2lf", rmlVelGoal.TargetVelocityVector[0],
                 rmlVelGoal.TargetVelocityVector[1], rmlVelGoal.TargetVelocityVector[2],
                 rmlVelGoal.TargetVelocityVector[3], rmlVelGoal.TargetVelocityVector[4],
                 rmlVelGoal.TargetVelocityVector[5], rmlVelGoal.TargetVelocityVector[6]);

        jntVelGoalPublisher.publish(rmlVelGoal);
        ros::spinOnce();
        rate.sleep();
    }

    exit(0);
}
