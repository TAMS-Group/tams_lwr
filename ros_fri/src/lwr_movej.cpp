/** lwr_movej.cpp: request LWR joint-position motion
 *
 * This program requests a joint-position motion on the KuKA LWR,
 * interpolated and controlled via Reflexxes-TypeII, but without
 * environment collision checks.
 *
 * NOTE / WARNING: this program does NOT perform any joint
 * limit checks; instead, this is supposed to be done by the
 * underlying motion controller (aka ros_fri / arm_controller ).
 *
 * 2017.01.03 - implement
 * 2016.06.29 - new
 */

#include <math.h>
#include <vector>
#include <map>

#include <ros/ros.h>
#include <ros_fri_msgs/RMLPositionInputParameters.h>
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

int n_joint_state_messages = 0;
std::vector<double> startAngles;
std::vector<double> startVelocities;
std::map<std::string, int> jointIndexMap;
pthread_mutex_t mutex;

/**
 * prints help/usage info and exists.
 */
int usage()
{
    printf("Usage: lwr_movej [rad|deg] [to|by] j1 j2 j3 j4 j5 j6 j7 \n");
    printf("where j1 .. j7 are joint-values in radians (or degrees)\n");
    printf("or the placeholder '.' to disable motion on that joint.\n");
    printf("Note that ros_fri (aka arm_controller) must be running.\n");
    printf("Example: lwr_movej rad to 0 0 0 0 0 0 0   (candle-position)\n");
    printf("Example: lwr_movej rad to _ _ _ _ _ .1.571 1.571  (only move j6 and j7)\n");
    printf("Example: lwr_movej deg by 10 _ _ -10 _ _ _   (values in degrees)\n");
    exit(1);
}

void jointStateCallback(const sensor_msgs::JointState jointState)
{
    pthread_mutex_lock(&mutex);

    // ROS_WARN( "jointStateCallback %d", n_joint_state_messages );
    unsigned int matched_lwr_joints = 0;
    for (unsigned int i = 0; i < jointState.name.size(); i++)
    {
        std::string name = jointState.name[i];
        int found = jointIndexMap.count(name);
        // ROS_INFO( "parsing joint '%s' found %d", name.c_str(), found );
        if (found > 0)
        {
            int lwrIndex = jointIndexMap[name];
            startAngles[lwrIndex] = jointState.position[i];
            startVelocities[lwrIndex] = jointState.velocity[i];
            matched_lwr_joints++;
        }
        else
        {  // ignore unknown joints
            ROS_INFO("lwr_movej: unmatched joint name '%s'", name.c_str());
        }
    }

    if (matched_lwr_joints == NUMJ)
    {
        n_joint_state_messages++;
    }

    pthread_mutex_unlock(&mutex);
}

int main(int argc, char** argv)
{
    if (0 != pthread_mutex_init(&mutex, NULL))
    {
        ROS_ERROR("pthread_mutex_init failed.");
        exit(1);
    }
    ros::init(argc, argv, "lwr_movej", 1);

    // ROS_ERROR( "lwr_movej: got %d command lne arguments:", argc );
    // for( int i=0; i < argc; i++ ) {
    // ROS_ERROR( "%d  '%s'", i, argv[i] );
    // }

    // phase 1: basic command line checks
    //
    if (argc < 10)
    {
        ROS_ERROR("lwr_movej: too few command line arguments.");
        usage();
    }
    bool move_to = false;
    bool move_rel = false;
    bool use_degrees = false;

    if (strcmp(argv[1], "rad") == 0)
    {
        use_degrees = false;
    }
    else if (strcmp(argv[1], "deg") == 0)
    {
        use_degrees = true;
    }
    else
    {
        ROS_ERROR("lwr_movej: need either 'rad' or 'deg' argument (radians or degrees).");
        usage();
    }

    if (strcmp(argv[2], "to") == 0)
    {
        move_to = true;
    }
    else if (strcmp(argv[2], "by") == 0)
    {
        move_rel = true;
    }
    else
    {
        ROS_ERROR("lwr_movej: need either 'to' or 'by' command.");
        usage();
    }

    if (argc > 10)
    {
        ROS_ERROR("lwr_movej: too many arguments.");
        usage();
    }

    // phase 2: extract joint values from command
    //
    // int NUMJ = 7;
    double target[NUMJ];
    bool wildcard[NUMJ];

    for (int j = 0; j < NUMJ; j++)
    {
        double val;
        int n;

        char* token = argv[j + 3];  // joint values start at index 3

        // ROS_INFO( "parsing joint value %d, token is '%s'", j, token );

        if (strcmp(token, "_") == 0)
        {  // '*' does NOT work in bash..
            wildcard[j] = true;
            target[j] = 0.0;
        }
        else if (1 == sscanf(token, "%lf", &val))
        {
            wildcard[j] = false;
            if (use_degrees)
                target[j] = val * DEG2RAD;
            else
                target[j] = val;
        }
        else
        {
            ROS_ERROR("lwr_movej: invalid argument '%s'", token);
            usage();
        }
    }

    // phase 3: create a NodeHandle and an async spinner.
    //
    ros::NodeHandle nh;
    startAngles.resize(NUMJ);
    startVelocities.resize(NUMJ);
    jointIndexMap["lwr_arm_0_joint"] = 0;
    jointIndexMap["lwr_arm_1_joint"] = 1;
    jointIndexMap["lwr_arm_2_joint"] = 2;
    jointIndexMap["lwr_arm_3_joint"] = 3;
    jointIndexMap["lwr_arm_4_joint"] = 4;
    jointIndexMap["lwr_arm_5_joint"] = 5;
    jointIndexMap["lwr_arm_6_joint"] = 6;

    // phase 4: subscribe to /lwr/joint_states and wait for
    // 10 messages, to ensure that the robot is up and running
    //
    ros::Subscriber sub = nh.subscribe<const sensor_msgs::JointState>("lwr/joint_states", 1, &jointStateCallback);
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
            ROS_ERROR("lwr_movej: no (valid) /lwr/joint_states messages after %lf secs.", max_wait_time);
            exit(2);
        }
    }

    // phase 4: now that we have current joint_state from the robot,
    // calculate full target angles (in radians).
    //
    pthread_mutex_lock(&mutex);
    for (int j = 0; j < NUMJ; j++)
    {
        double value = target[j];
        if (move_to)
        {
            if (wildcard[j])
                target[j] = startAngles[j];
            else
                target[j] = value;
        }
        else
        {  // move_rel
            if (wildcard[j])
                target[j] = startAngles[j];
            else
                target[j] = value + startAngles[j];
        }
        ROS_INFO("lwr_movej: joint %d %d %d  command %8.4lf start %8.4lf goal %8.4lf", j, move_to, wildcard[j], value,
                 startAngles[j], target[j]);
    }
    pthread_mutex_unlock(&mutex);

    // phase 5: publish the joint position goal
    //
    ros::Publisher jntPosGoalPublisher =
        nh.advertise<ros_fri_msgs::RMLPositionInputParameters>("/lwr/jointPositionGoal", 1);
    // "fake_lwr_jntPosGoal", 1 );
    usleep(500 * 1000);  // give roscore and subscribers a chance

    ros_fri_msgs::RMLPositionInputParameters rmlPosGoal;
    ROS_INFO("moving to target position...");
    {  // for( int n=0; n < 3; n++ ) {
        rmlPosGoal.header.seq = 1;
        rmlPosGoal.header.stamp = ros::Time::now();
        rmlPosGoal.header.frame_id = "lwr_arm_base_link";
        rmlPosGoal.TargetPositionVector.resize(NUMJ);
        rmlPosGoal.TargetVelocityVector.resize(NUMJ);
        rmlPosGoal.MaxAccelerationVector.resize(NUMJ);
        rmlPosGoal.MaxVelocityVector.resize(NUMJ);
        for (int i = 0; i < 7; i++)
        {
            rmlPosGoal.TargetPositionVector[i] = target[i];
            rmlPosGoal.TargetVelocityVector[i] = 0;
            rmlPosGoal.MaxAccelerationVector[i] = 0.3;
            rmlPosGoal.MaxVelocityVector[i] = 0.5;
        }
        // ROS_INFO( "... publishing jnt pos goal..." );
        jntPosGoalPublisher.publish(rmlPosGoal);
        ros::spinOnce();
        usleep(100 * 1000);
    }

    // phase 6: wait for execution?
    // NOTE: at the moment, not. Just spin for half a second.
    //
    double dt = 0.0;
    while (dt < 0.5)
    {
        ros::spinOnce();
        usleep(100 * 1000);
        dt += 0.1;
    }
    ROS_INFO("ok.");
    exit(0);
}
