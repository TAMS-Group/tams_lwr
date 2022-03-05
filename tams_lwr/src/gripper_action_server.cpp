/** gripper_action_server.cpp - Actionlib server for the
 * Weiss/Schunk WSG-50 gripper with tactile fingers, using
 * control_msgs/GripperCommandAction
 *
 * Johannes Liebrecht --- 8liebrec@informatik.uni-hamburg.de
 */
#include <string>
#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <control_msgs/GripperCommandAction.h>

#include <wsg_50_common/Move.h>
#include <wsg_50_common/Conf.h>
#include <wsg_50_common/Status.h>

class GripperActionServer
{
protected:
    ros::NodeHandle nh_;
    // NodeHandle instance must be created before this line. Otherwise, strange error may occur.
    actionlib::SimpleActionServer<control_msgs::GripperCommandAction> as_;
    std::string action_name_;

    control_msgs::GripperCommand goal_;
    control_msgs::GripperCommandFeedback feedback_;
    control_msgs::GripperCommandResult result_;

    ros::Subscriber sub_;
    ros::ServiceClient client_move_;
    ros::ServiceClient client_release_;
    ros::ServiceClient client_set_force_;

public:
    GripperActionServer(std::string name) : as_(nh_, name, false), action_name_(name)
    {
        // register the goal and feedback callbacks
        as_.registerGoalCallback(boost::bind(&GripperActionServer::goalCB, this));
        as_.registerPreemptCallback(boost::bind(&GripperActionServer::preemptCB, this));

        client_move_ = nh_.serviceClient<wsg_50_common::Move>("wsg_50/movehold");
        client_release_ = nh_.serviceClient<wsg_50_common::Move>("wsg_50/release");
        client_set_force_ = nh_.serviceClient<wsg_50_common::Conf>("wsg_50/set_force");

        sub_ = nh_.subscribe<wsg_50_common::Status>("wsg_50/status", 1, &GripperActionServer::statusCB, this);

        as_.start();
    }

    void goalCB()
    {  // accept goal
        goal_ = as_.acceptNewGoal()->command;
        // call goal on wsg_50
        wsg_50_common::Conf conf_srv;
        // todo: change the max effort by topic
        conf_srv.request.val = 40; //goal_.max_effort;
        client_set_force_.call(conf_srv);
        wsg_50_common::Move move_srv;
        move_srv.request.width = std::abs(goal_.position * 1000 * 2);  // from m to mm
        move_srv.request.speed = 60;                    // max speed;
        if (move_srv.request.width > 80){  // assume target larger than 80 mm is open griper action
            client_release_.call(move_srv);
        }
        else{
            client_move_.call(move_srv);
        }
    }

    void preemptCB()
    {
        ROS_INFO("%s: Preempted", action_name_.c_str());
        // set the action state to preempted
        as_.setPreempted();
        // todo
        // wsg_50_x/stop: Stops a current action. Not working yet.
    }

    void statusCB(const wsg_50_common::Status status)
    {
        if (!as_.isActive())
        {
            return;
        }

        feedback_.position = status.width;
        feedback_.effort = status.force;
        result_.position = status.width;
        result_.effort = status.force;

        std::size_t found = status.status.find("Target Pos reached");
        if (found != std::string::npos)
        {
            feedback_.reached_goal = true;
            result_.reached_goal = true;
        }
        else
        {
            feedback_.reached_goal = false;
            result_.reached_goal = false;
        }

        found = status.status.find("Axis Stopped");
        if (found != std::string::npos)
        {
            feedback_.stalled = true;
            result_.stalled = true;
        }
        else
        {
            feedback_.stalled = false;
            result_.stalled = false;
        }

        as_.publishFeedback(feedback_);

        if (result_.reached_goal)
        {
            ROS_INFO("%s: Succeeded", action_name_.c_str());
            // set the action state to succeeded
            as_.setSucceeded(result_);
        }
        else
        {
            ROS_INFO("%s: Aborted", action_name_.c_str());
            // set the action state to aborted
            as_.setAborted(result_);
        }
    }

};  // class gripper_action_server

int main(int argc, char** argv)
{
    ros::init(argc, argv, "gripper_action_server", 1);  // 1 means no SigInt handler
    GripperActionServer gas(ros::this_node::getName());
    ros::spin();
    return 0;
}
