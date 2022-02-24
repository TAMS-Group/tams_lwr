/** lwr_move_to_startPose
 * moves to start pose for kuka_sphere_object_shape_detection.cpp
 */

#include <ros/ros.h>
#include <ros_fri_msgs/RMLPositionInputParameters.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "lwr_move_to_startPose", 1);
    ROS_WARN("lwr_move_to_startPose: starting...");

    ros::NodeHandle nh;
    ros::Publisher jntPosGoalPublisher =
        nh.advertise<ros_fri_msgs::RMLPositionInputParameters>("/lwr/jointPositionGoal", 1);

    ros::AsyncSpinner spinner(1);
    spinner.start();

    double startAngles[] = { -1.24, -0.64, 0.203, 1.694, 2.24, 1.479, -0.619 };

    int NUMJ = 7;
    int seq_number = 0;
    ros_fri_msgs::RMLPositionInputParameters rmlPosGoal;
    while (ros::ok())
    {
        ROS_WARN("moving to start position...");
        seq_number++;
        rmlPosGoal.header.seq = seq_number;
        rmlPosGoal.header.stamp = ros::Time::now();
        rmlPosGoal.header.frame_id = "world";
        rmlPosGoal.TargetPositionVector.resize(NUMJ);
        rmlPosGoal.TargetVelocityVector.resize(NUMJ);
        rmlPosGoal.MaxAccelerationVector.resize(NUMJ);
        rmlPosGoal.MaxVelocityVector.resize(NUMJ);
        for (unsigned int i = 0; i < NUMJ; i++)
        {
            rmlPosGoal.TargetPositionVector.push_back(startAngles[i]);
        }
        for (int i = 0; i < 7; i++)
        {
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
