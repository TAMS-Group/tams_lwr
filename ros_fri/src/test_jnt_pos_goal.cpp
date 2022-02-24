/** test_jnt_pos_goal: send joint-position goals to the LWR
 *
 * 2016.06.29 - catkinize fnh
 */

#include <ros/ros.h>
#include <ros_fri_msgs/RMLPositionInputParameters.h>

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

int main(int argc, char** argv)
{
    ros::init(argc, argv, "test_jnt_pos_goal", 1);
    ROS_WARN("test_jnt_pos_goal: staring...");

    ros::NodeHandle nh;
    ros::Publisher jntPosGoalPublisher = nh.advertise<ros_fri_msgs::RMLPositionInputParameters>("lwr_jntPosGoal", 1);

    ros::AsyncSpinner spinner(1);
    spinner.start();

    int NUMJ = 7;
    int seq_number = 0;
    ros_fri_msgs::RMLPositionInputParameters rmlPosGoal;
    while (ros::ok())
    {
        ROS_WARN("moving to candle position...");
        seq_number++;
        rmlPosGoal.header.seq = seq_number;
        rmlPosGoal.header.stamp = ros::Time::now();
        rmlPosGoal.header.frame_id = "lwr_base_link";
        rmlPosGoal.TargetPositionVector.resize(NUMJ);
        rmlPosGoal.TargetVelocityVector.resize(NUMJ);
        rmlPosGoal.MaxAccelerationVector.resize(NUMJ);
        rmlPosGoal.MaxVelocityVector.resize(NUMJ);
        for (int i = 0; i < 7; i++)
        {
            rmlPosGoal.TargetPositionVector[i] = 0;
            rmlPosGoal.TargetVelocityVector[i] = 0;
            rmlPosGoal.MaxAccelerationVector[i] = 0.5;
            rmlPosGoal.MaxVelocityVector[i] = 0.5;
        }
        ROS_INFO("... publishing jnt pos goal...");
        jntPosGoalPublisher.publish(rmlPosGoal);
        ROS_INFO("... sleeping 5 seconds...");
        usleep(5 * 1000 * 1000);

        ROS_WARN("moving to all-0.1 position...");
        seq_number++;
        rmlPosGoal.header.seq = seq_number;
        rmlPosGoal.header.stamp = ros::Time::now();
        rmlPosGoal.header.frame_id = "lwr_base_link";
        rmlPosGoal.TargetPositionVector.resize(NUMJ);
        rmlPosGoal.TargetVelocityVector.resize(NUMJ);
        rmlPosGoal.MaxAccelerationVector.resize(NUMJ);
        rmlPosGoal.MaxVelocityVector.resize(NUMJ);
        for (int i = 0; i < 7; i++)
        {
            rmlPosGoal.TargetPositionVector[i] = 0.1;
            rmlPosGoal.TargetVelocityVector[i] = 0;
            rmlPosGoal.MaxAccelerationVector[i] = 0.5;
            rmlPosGoal.MaxVelocityVector[i] = 0.5;
        }
        ROS_INFO("... publishing jnt pos goal...");
        jntPosGoalPublisher.publish(rmlPosGoal);
        ROS_INFO("... sleeping 5 seconds...");
        usleep(5 * 1000 * 1000);
    }
}
