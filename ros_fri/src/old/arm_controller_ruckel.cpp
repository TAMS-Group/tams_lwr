// Johannes Liebrecht --- 8liebrec@informatik.uni-hamburg.de
#include <string>
#include <ros/ros.h>
#include <actionlib/server/action_server.h>
#include <actionlib/server/server_goal_handle.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <sensor_msgs/JointState.h>

#include <TypeIRML.h>
#include <FastResearchInterface.h>
#include <LinuxAbstraction.h>

#ifndef PI
#define PI 3.1415926535897932384626433832795
#endif

#ifndef RAD
#define RAD(A) ((A)*PI / 180.0)
#endif

#ifndef DEG
#define DEG(A) ((A)*180.0 / PI)
#endif

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
    int seq_counterFJTF_, seqCounterJointStates_;

    // Fri
    FastResearchInterface* FRI_;
    TypeIRML* RML_;
    TypeIRMLInputParameters* IP_;
    TypeIRMLOutputParameters* OP_;
    double CycleTime_;
    float JointValuesInRad_[LBR_MNJ];
    float TorqueValuesInRad_[LBR_MNJ];

    ros::Publisher pub_joints_;

public:
    lwr_action_server(std::string name)
        : as_(nh_, name, false), action_name_("lwr_action_server/joint_trajectory_action")
    {
        // register the goal and cancel callbacks
        as_.registerGoalCallback(boost::bind(&lwr_action_server::goalCB, this, _1));
        as_.registerCancelCallback(boost::bind(&lwr_action_server::cancelCB, this, _1));
        as_.start();

        pub_joints_ = nh_.advertise<sensor_msgs::JointState>("/joint_states", 1);

        goal_active_ = false;
        cancel_goal_ = false;
        seq_counterFJTF_ = 0;
        seqCounterJointStates_ = 0;

        FRI_ = new FastResearchInterface("/homeL/8liebrec/TAMS/FRILibrary/etc/980039-FRI-Driver.init");
        CycleTime_ = 0.02;
        RML_ = new TypeIRML(LBR_MNJ, CycleTime_);
        IP_ = new TypeIRMLInputParameters(LBR_MNJ);
        OP_ = new TypeIRMLOutputParameters(LBR_MNJ);

        memset(JointValuesInRad_, 0x0, LBR_MNJ * sizeof(float));
        memset(TorqueValuesInRad_, 0x0, LBR_MNJ * sizeof(float));

        int ResultValue = 0;
        ResultValue = FRI_->StartRobot(FastResearchInterface::JOINT_POSITION_CONTROL);
        if ((ResultValue != EOK) && (ResultValue != EALREADY))
        {
            ROS_INFO("lwr_action_server: An error occurred during starting up the robot...\n");
            exit(EXIT_FAILURE);
        }
    }

    ~lwr_action_server()
    {
        FRI_->StopRobot();
        delete RML_;
        delete IP_;
        delete OP_;
        delete FRI_;
    }

    void goalCB(GoalHandle gh)
    {
        if (!goal_active_)
        {
            // TODO check if order and joint names are correct else gh.setRejected();
            gh.setAccepted();

            // send each trajectory point to FRI
            Goal g = gh.getGoal();

            for (unsigned int i = 0; i < g->trajectory.points.size(); i++)
            {
                // execute trajectory point
                FRI_->GetMeasuredJointPositions(JointValuesInRad_);

                for (unsigned int j = 0; j < LBR_MNJ; j++)
                {
                    IP_->CurrentPosition->VecData[j] = (double)DEG(JointValuesInRad_[j]);
                    IP_->TargetPosition->VecData[j] = (double)DEG(g->trajectory.points[i].positions[j]);
                    // IP->TargetVelocity->VecData[j] = DEG(g->trajectory.points[i].velocities[j]);
                    IP_->MaxVelocity->VecData[j] = 50.0;
                    IP_->MaxAcceleration->VecData[j] = 50.0;  // DEG(g->trajectory.points[i].accelerations[j]);
                    IP_->SelectionVector->VecData[j] = true;
                }
                int ResultValue = TypeIRML::RML_WORKING;

                // target position-based trajectory generation.
                while ((FRI_->IsMachineOK()) && (ResultValue != TypeIRML::RML_FINAL_STATE_REACHED))
                {
                    FRI_->WaitForKRCTick();
                    ResultValue = RML_->GetNextMotionState_Position(*IP_, OP_);

                    //	if ((ResultValue != TypeIRML::RML_WORKING) && (ResultValue != TypeIRML::RML_FINAL_STATE_REACHED))
                    //	{
                    //		ROS_INFO("lwr_action_server: ERROR during trajectory generation (%d).", ResultValue);
                    //	}

                    for (unsigned int j = 0; j < LBR_MNJ; j++)
                    {
                        JointValuesInRad_[j] = RAD((double)(OP_->NewPosition->VecData[j]));
                    }

                    FRI_->SetCommandedJointPositions(JointValuesInRad_);

                    // prepare for next iteration
                    *(IP_->CurrentPosition) = *(OP_->NewPosition);
                    *(IP_->CurrentVelocity) = *(OP_->NewVelocity);

                    // check cancel request
                    if (cancel_goal_)
                    {
                        ROS_INFO("lwr_action_server: Goal canceled\n");
                        gh.setAborted();
                        cancel_goal_ = false;
                        return;
                    }
                }
                ROS_INFO("FINEEEEEEEEEEEEEEEEEEE");

                // Trajectorypoint executed, now publish Feedback and Jointstates
                // pubish JointStates with velocity
                FRI_->GetMeasuredJointPositions(JointValuesInRad_);
                FRI_->GetMeasuredJointTorques(TorqueValuesInRad_);
                //	FRI_->WaitForKRCTick();	//TODO test if needed???

                sensor_msgs::JointState js;
                js.header.seq = seqCounterJointStates_;
                js.header.stamp = ros::Time::now();
                js.header.frame_id = "calib_lwr_arm_base_link";
                js.name = g->trajectory.joint_names;

                for (int j = 0; j < LBR_MNJ; j++)
                {
                    js.position.push_back(JointValuesInRad_[j]);
                    js.velocity.push_back(RAD(IP_->CurrentVelocity->VecData[j]));
                    js.effort.push_back(TorqueValuesInRad_[j]);
                }

                pub_joints_.publish(js);
                seqCounterJointStates_++;

                // fill trajectory_msgs/JointTrajectoryPoints
                trajectory_msgs::JointTrajectoryPoint actual;
                trajectory_msgs::JointTrajectoryPoint desired;
                trajectory_msgs::JointTrajectoryPoint error;
                for (unsigned int j = 0; j < LBR_MNJ; j++)
                {
                    actual.positions.push_back((double)JointValuesInRad_[j]);
                    desired.positions.push_back(g->trajectory.points[i].positions[j]);
                    error.positions.push_back(g->trajectory.points[i].positions[j] - JointValuesInRad_[j]);

                    actual.velocities.push_back(RAD(IP_->CurrentVelocity->VecData[j]));
                    desired.velocities.push_back(g->trajectory.points[i].velocities[j]);
                    error.velocities.push_back(g->trajectory.points[i].velocities[j] -
                                               RAD(IP_->CurrentVelocity->VecData[j]));

                    // TODO read FRI acceleration
                    actual.accelerations.push_back(NAN);
                    desired.accelerations.push_back(g->trajectory.points[i].accelerations[j]);
                    error.accelerations.push_back(NAN);

                    actual.time_from_start.sec = RML_->GetExecutionTime();
                    desired.time_from_start = g->trajectory.points[j].time_from_start;
                    error.time_from_start.sec = g->trajectory.points[j].time_from_start.sec - RML_->GetExecutionTime();
                }

                // fill control_msgs/FollowJointTrajectoryFeedback
                control_msgs::FollowJointTrajectoryFeedback fjtf;
                fjtf.header.seq = seq_counterFJTF_;
                fjtf.header.stamp = ros::Time::now();
                fjtf.header.frame_id = "calib_lwr_arm_base_link";
                fjtf.joint_names = g->trajectory.joint_names;
                fjtf.actual = actual;
                fjtf.desired = desired;
                fjtf.error = error;
                gh.publishFeedback(fjtf);
                seq_counterFJTF_++;
            }
            // Trajectory finished all was fine Todo publish Result??..
            gh.setSucceeded();
        }
        else
        {
            gh.setRejected();
        }
    }

    void cancelCB(GoalHandle gh)
    {
        // cancel FRI
        cancel_goal_ = true;
        // cancel Goalhandle
        goal_active_ = false;
        gh.setCanceled();
    }

    void publishJointStatesWithoutActiveGoal()
    {
        pthread_mutex_lock(&mutex);
        std::string names[] = { "lwr_arm_0_joint", "lwr_arm_1_joint", "lwr_arm_2_joint", "lwr_arm_3_joint",
                                "lwr_arm_4_joint", "lwr_arm_5_joint", "lwr_arm_6_joint" };

        FRI_->GetMeasuredJointPositions(JointValuesInRad_);
        FRI_->GetMeasuredJointTorques(TorqueValuesInRad_);
        sensor_msgs::JointState js;

        js.header.seq = seqCounterJointStates_;
        js.header.stamp = ros::Time::now();
        js.header.frame_id = "calib_lwr_arm_base_link";

        for (int i = 0; i < LBR_MNJ; i++)
        {  // ROS_INFO("names %f value \n",JointValuesInRad_[i]);
            js.name.push_back(names[i]);
            js.position.push_back(JointValuesInRad_[i]);
            js.velocity.push_back(0);
            js.effort.push_back(TorqueValuesInRad_[i]);
        }

        pub_joints_.publish(js);
        seqCounterJointStates_++;
        pthread_mutex_unlock(&mutex);
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "lwr_action_server");
    lwr_action_server lwr_action_server(ros::this_node::getName());
    //  important
    ros::AsyncSpinner spinner(1);
    spinner.start();
    ros::Rate loop_rate(5);  // 10Hz

    while (ros::ok())
    {
        lwr_action_server.publishJointStatesWithoutActiveGoal();
        ros::spinOnce();
        loop_rate.sleep();
    }
}

/*	Analysing KRL Value.......
bool *KRLBoolValues;
int *KRLIntValues;
float *KRLFloatValues;
FRI_->GetKRLBoolValues (KRLBoolValues);
FRI_->GetKRLIntValues (KRLIntValues);
FRI_->GetKRLFloatValues (KRLFloatValues);
for (unsigned int j = 0; j < 16; j++)
{
    ROS_INFO("KRLBoolValues(%d).", KRLBoolValues[i]);
}
for (unsigned int j = 0; j < 16; j++)
{
    ROS_INFO("KRLIntValues(%d).", KRLIntValues[i]);
}
for (unsigned int j = 0; j < 16; j++)
{
    ROS_INFO("KRLFloatValues(%f).", KRLFloatValues[i]);
}
*/
