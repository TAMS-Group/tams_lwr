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

    FastResearchInterface* FRI;

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

        // FRI_ = new FastResearchInterface("/homeL/8liebrec/TAMS/FRILibrary/etc/980039-FRI-Driver.init");

        RML_ = new TypeIRML(LBR_MNJ, CycleTime_);
        IP_ = new TypeIRMLInputParameters(LBR_MNJ);
        OP_ = new TypeIRMLOutputParameters(LBR_MNJ);
        CycleTime_ = 0.02;
        memset(JointValuesInRad_, 0x0, LBR_MNJ * sizeof(float));

        unsigned int ControlScheme = FastResearchInterface::JOINT_POSITION_CONTROL;
        FRI = new FastResearchInterface("/homeL/8liebrec/TAMS/FRILibrary/etc/980039-FRI-Driver.init");
        FRI->StartRobot(ControlScheme);
    }

    void goalCB(GoalHandle gh)
    {
        unsigned int i = 0;

        int ResultValue = 0;

        float JointValuesInRad[7];

        double CycleTime = 0.02;

        TypeIRML* RML;

        TypeIRMLInputParameters* IP;

        TypeIRMLOutputParameters* OP;

        RML = new TypeIRML(7, CycleTime);

        IP = new TypeIRMLInputParameters(7);

        OP = new TypeIRMLOutputParameters(7);

        memset(JointValuesInRad, 0x0, 7 * sizeof(float));

        FRI->GetMeasuredJointPositions(JointValuesInRad);

        for (i = 0; i < 7; i++)
        {
            IP->CurrentPosition->VecData[i] = (double)DEG(JointValuesInRad[i]);
            IP->TargetPosition->VecData[i] = (double)0.2;
            IP->MaxVelocity->VecData[i] = (double)50.0;
            IP->MaxAcceleration->VecData[i] = (double)50.0;
            IP->SelectionVector->VecData[i] = true;
        }

        ResultValue = TypeIRML::RML_WORKING;

        while ((FRI->IsMachineOK()) && (ResultValue != TypeIRML::RML_FINAL_STATE_REACHED))
        {
            FRI->WaitForKRCTick();

            ResultValue = RML->GetNextMotionState_Position(*IP, OP);

            for (i = 0; i < 7; i++)
            {
                JointValuesInRad[i] = RAD((double)(OP->NewPosition->VecData[i]));
            }

            FRI->SetCommandedJointPositions(JointValuesInRad);

            *(IP->CurrentPosition) = *(OP->NewPosition);
            *(IP->CurrentVelocity) = *(OP->NewVelocity);
        }
        ROS_INFO("FINEEEEEEEEEEEEEEE");
        FRI->StopRobot();
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
        // string names[] = {"A1", "A2", "E1", "A3", "A4", "A5", "A6"};
        std::string names[] = { "lwr_arm_0_joint", "lwr_arm_1_joint", "lwr_arm_2_joint", "lwr_arm_3_joint",
                                "lwr_arm_4_joint", "lwr_arm_5_joint", "lwr_arm_6_joint" };  // TODO check

        FRI->GetMeasuredJointPositions(JointValuesInRad_);
        FRI->GetMeasuredJointTorques(TorqueValuesInRad_);
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
    ros::Rate loop_rate(10);  // 10Hz

    while (ros::ok())
    {
        lwr_action_server.publishJointStatesWithoutActiveGoal();
        ros::spinOnce();
        loop_rate.sleep();
    }
}
