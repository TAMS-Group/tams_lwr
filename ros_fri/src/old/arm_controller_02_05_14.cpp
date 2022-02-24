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
    bool goal_active_, cancel_goal_, executeTrajectory_;
    int seq_counterFJTF_, seqCounterJointStates_;
    GoalHandle agh_;

    // Fri
    FastResearchInterface* FRI_;

    double Position_old_[LBR_MNJ], Position_new_[LBR_MNJ], Position_delta_[LBR_MNJ];
    ros::Time time_old_, time_new_, time_delta_;
    double velocity_new_[LBR_MNJ], velocity_old_[LBR_MNJ];
    double acceleration_[LBR_MNJ];

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

        memset(JointValuesInRad_, 0x0, LBR_MNJ * sizeof(float));
        memset(TorqueValuesInRad_, 0x0, LBR_MNJ * sizeof(float));

        int ResultValue = 0;
        ResultValue = FRI_->StartRobot(FastResearchInterface::JOINT_POSITION_CONTROL);
        if ((ResultValue != EOK) && (ResultValue != EALREADY))
        {
            ROS_INFO("lwr_action_server: An error occurred during starting up the robot...\n");
            exit(EXIT_FAILURE);
        }

        FRI_->GetMeasuredJointPositions(JointValuesInRad_);
        for (unsigned int j = 0; j < LBR_MNJ; j++)
        {
            Position_old_[j] = JointValuesInRad_[j];
        }
        time_old_ = ros::Time::now();
        FRI_->WaitForKRCTick();
        FRI_->GetMeasuredJointPositions(JointValuesInRad_);
        for (unsigned int j = 0; j < LBR_MNJ; j++)
        {
            Position_new_[j] = JointValuesInRad_[j];
            Position_delta_[j] = Position_new_[j] - Position_old_[j];
        }
        time_new_ = ros::Time::now();
        time_delta_.sec = time_new_.sec - time_old_.sec;
        time_delta_.nsec = time_new_.nsec - time_old_.nsec;
    }

    ~lwr_action_server()
    {
        FRI_->StopRobot();
        delete FRI_;
    }

    void goalCB(GoalHandle gh)
    {
        if (!goal_active_)
        {
            // TODO check if order and joint names are correct else gh.setRejected();
            gh.setAccepted();
            agh_ = gh;
            goal_active_ = true;
        }
        else
        {
            gh.setRejected();
        }
    }

    void executeTrajectory()
    {
        if (goal_active_)
        {
            // send each trajectory point to FRI
            Goal g = agh_.getGoal();
            ros::Duration time_from_start;
            time_from_start.sec = 0;
            time_from_start.nsec = 0;
            ros::Time time_start;

            time_start = ros::Time::now();
            for (unsigned int i = 0; i < g->trajectory.points.size(); i++)
            {
                FRI_->WaitForKRCTick();

                if (!FRI_->IsMachineOK())
                {
                    ROS_INFO("lwr_action_server: ERROR the machine is not ready anymore.");
                    agh_.setAborted();
                    exit(EXIT_FAILURE);
                }
                for (unsigned int j = 0; j < LBR_MNJ; j++)
                {
                    JointValuesInRad_[j] = g->trajectory.points[i].positions[j];
                }
                FRI_->SetCommandedJointPositions(JointValuesInRad_);

                publishJointStates();

                time_from_start = (ros::Time::now() - time_start);
                publishFeedback(g, i, time_from_start);

                // check cancel request
                if (cancel_goal_)
                {
                    ROS_INFO("lwr_action_server: Goal canceled\n");
                    agh_.setCanceled();
                    cancel_goal_ = false;
                    stopLwr(g);
                    return;
                }
            }
            // Trajectory finished all was fine Todo publish Result??..
            agh_.setSucceeded();
            goal_active_ = false;
        }
    }

    void cancelCB(GoalHandle gh)
    {
        // cancel FRI
        cancel_goal_ = true;
        // cancel Goalhandle
        goal_active_ = false;
    }

    void stopLwr(Goal g)
    {
        double CycleTime_ = 0.01;
        TypeIRML* RML = new TypeIRML(LBR_MNJ, CycleTime_);
        TypeIRMLInputParameters* IP = new TypeIRMLInputParameters(LBR_MNJ);
        TypeIRMLOutputParameters* OP = new TypeIRMLOutputParameters(LBR_MNJ);

        // load first position
        FRI_->GetMeasuredJointPositions(JointValuesInRad_);
        for (unsigned int j = 0; j < LBR_MNJ; j++)
        {
            IP->CurrentPosition->VecData[j] = (double)DEG(JointValuesInRad_[j]);
            IP->CurrentVelocity->VecData[j] = (double)DEG(velocity_new_[j]);
            IP->TargetVelocity->VecData[j] = (double)0.0;
            IP->MaxVelocity->VecData[j] = (double)DEG(0.5);
            IP->MaxAcceleration->VecData[j] = (double)DEG(0.2);
            IP->SelectionVector->VecData[j] = true;
        }
        // execute Trajectory
        int ResultValue = TypeIRML::RML_WORKING;

        // target position-based trajectory generation.
        while ((FRI_->IsMachineOK()) && (ResultValue != TypeIRML::RML_FINAL_STATE_REACHED))
        {
            FRI_->WaitForKRCTick();
            ResultValue = RML->GetNextMotionState_Velocity(*IP, OP);

            if ((ResultValue != TypeIRML::RML_WORKING) && (ResultValue != TypeIRML::RML_FINAL_STATE_REACHED))
            {
                ROS_INFO("lwr_action_server: ERROR during trajectory generation (%d).", ResultValue);
            }

            for (unsigned int j = 0; j < LBR_MNJ; j++)
            {
                JointValuesInRad_[j] = RAD((double)(OP->NewPosition->VecData[j]));
            }
            FRI_->SetCommandedJointPositions(JointValuesInRad_);

            // prepare for next iteration
            *(IP->CurrentPosition) = *(OP->NewPosition);
            *(IP->CurrentVelocity) = *(OP->NewVelocity);
        }
        delete RML;
        delete IP;
        delete OP;
    }

    void publishFeedback(Goal g, unsigned int counter, ros::Duration time_from_start)
    {
        FRI_->GetMeasuredJointTorques(TorqueValuesInRad_);
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
            actual.positions.push_back((double)JointValuesInRad_[j]);
            desired.positions.push_back(g->trajectory.points[counter].positions[j]);
            error.positions.push_back(fabs(g->trajectory.points[counter].positions[j]) - fabs(JointValuesInRad_[j]));

            actual.velocities.push_back(velocity_new_[j]);
            desired.velocities.push_back(g->trajectory.points[counter].velocities[j]);
            error.velocities.push_back(fabs(g->trajectory.points[counter].velocities[j]) - fabs(velocity_new_[j]));

            actual.accelerations.push_back(acceleration_[j]);
            desired.accelerations.push_back(g->trajectory.points[counter].accelerations[j]);
            error.accelerations.push_back(fabs(g->trajectory.points[counter].accelerations[j]) - fabs(acceleration_[j]));

            actual.effort.push_back(TorqueValuesInRad_[j]);
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

    void publishJointStates()
    {
        pthread_mutex_lock(&mutex);
        std::string names[] = { "lwr_arm_0_joint", "lwr_arm_1_joint", "lwr_arm_2_joint", "lwr_arm_3_joint",
                                "lwr_arm_4_joint", "lwr_arm_5_joint", "lwr_arm_6_joint" };

        FRI_->GetMeasuredJointPositions(JointValuesInRad_);
        FRI_->GetMeasuredJointTorques(TorqueValuesInRad_);

        time_old_ = time_new_;
        time_new_ = ros::Time::now();

        for (unsigned int j = 0; j < LBR_MNJ; j++)
        {
            Position_old_[j] = Position_new_[j];
            Position_new_[j] = JointValuesInRad_[j];
            velocity_old_[j] = velocity_new_[j];
            velocity_new_[j] = (Position_new_[j] - Position_old_[j]) / (time_new_.toSec() - time_old_.toSec());
        }

        sensor_msgs::JointState js;

        js.header.seq = seqCounterJointStates_;
        js.header.stamp = ros::Time::now();
        js.header.frame_id = "calib_lwr_arm_base_link";

        for (int i = 0; i < LBR_MNJ; i++)
        {  // ROS_INFO("names %f value \n",JointValuesInRad_[i]);
            js.name.push_back(names[i]);
            js.position.push_back(JointValuesInRad_[i]);
            js.velocity.push_back(velocity_new_[i]);
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
        lwr_action_server.publishJointStates();
        lwr_action_server.executeTrajectory();
        ros::spinOnce();
        loop_rate.sleep();
    }
}

/*
  //load first position
   FRI_->GetMeasuredJointPositions(JointValuesInRad_);
   for (unsigned int j = 0; j < LBR_MNJ; j++)
   {
    IP_->CurrentPosition->VecData[j] = (double) DEG(JointValuesInRad_[j]);
    IP_->CurrentVelocity->VecData[j] = (double) 0.0;
    IP_->MaxVelocity->VecData[j] = (double) DEG(0.2);
    IP_->MaxAcceleration->VecData[j] = (double) DEG(0.2);
    IP_->SelectionVector->VecData[j] = true;
   }
  //execute Trajectory
  for (unsigned int i = 0; i < g->trajectory.points.size(); i++)
    {
    for (unsigned int j = 0; j < LBR_MNJ; j++)
    {
      //IP_->TargetPosition->VecData[j]	= (double) DEG(g->trajectory.points[i].positions[j]);
      IP_->TargetVelocity->VecData[j] = (double) DEG(g->trajectory.points[i].velocities[j]);
    }
    int ResultValue	= TypeIRML::RML_WORKING;

      //target position-based trajectory generation.
    while ((FRI_->IsMachineOK()) && (ResultValue != TypeIRML::RML_FINAL_STATE_REACHED))
    {
      FRI_->WaitForKRCTick();
      //ResultValue	= RML_->GetNextMotionState_Position(*IP_, OP_);
      ResultValue	= RML_->GetNextMotionState_Velocity(*IP_, OP_);

      if ((ResultValue == TypeIRML::RML_FINAL_STATE_REACHED) and (i < (g->trajectory.points.size()-1)))
      {
        break;
      }

      if ((ResultValue != TypeIRML::RML_WORKING) && (ResultValue != TypeIRML::RML_FINAL_STATE_REACHED))
      {
        ROS_INFO("lwr_action_server: ERROR during trajectory generation (%d).", ResultValue);
      }

      for (unsigned int j = 0; j < LBR_MNJ; j++)
      {
        JointValuesInRad_[j] = RAD((double) (OP_->NewPosition->VecData[j]));
      }
      FRI_->SetCommandedJointPositions(JointValuesInRad_);
  ROS_INFO("SEND::::::::::");
      //prepare for next iteration
      *(IP_->CurrentPosition)	= *(OP_->NewPosition);
      *(IP_->CurrentVelocity)	= *(OP_->NewVelocity);

      //check cancel request
      if (cancel_goal_)
      {
        ROS_INFO("lwr_action_server: Goal canceled\n");
        gh.setAborted();
        cancel_goal_ = false;
        return;
      }
    }

   ROS_INFO("FINEEEEEEEEEEEEEEEEEEE");
*/
