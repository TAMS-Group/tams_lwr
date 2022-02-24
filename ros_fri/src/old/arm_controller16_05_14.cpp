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

#include <ros_fri/LWRStates.h>
#include <ros_fri/MotionCompletion.h>

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
    int seq_counterFJTF_, seqCounterJointStates_, seqCounterLWRStates_;
    GoalHandle agh_;

    // Fri
    FastResearchInterface* FRI_;

    double Position_old_[LBR_MNJ], Position_new_[LBR_MNJ], Position_delta_[LBR_MNJ];
    ros::Time time_old_, time_new_, time_delta_;
    double velocity_new_[LBR_MNJ], velocity_old_[LBR_MNJ];
    double acceleration_[LBR_MNJ];

    float commandedJointValuesInRad_[LBR_MNJ];
    float measuredJointValuesInRad_[LBR_MNJ];
    float TorqueValuesInRad_[LBR_MNJ];
    float EstimatedExternalJointTorquesInRad_[LBR_MNJ];

    float EstimatedExternalCartForcesAndTorques_[12];
    float DriveTemperatues_[LBR_MNJ];

    // Msg
    ros_fri::MotionCompletion mc_;

    ros::Publisher pub_joints_;
    ros::Publisher pub_motionCompletion_;
    ros::Publisher pub_lwrStates_;

    // TODO TEST
    double CycleTime_;
    TypeIRML* RML;
    TypeIRMLInputParameters* IP;
    TypeIRMLOutputParameters* OP;

public:
    lwr_action_server(std::string name)
        : as_(nh_, name, false), action_name_("lwr_action_server/joint_trajectory_action")
    {
        // register the goal and cancel callbacks
        as_.registerGoalCallback(boost::bind(&lwr_action_server::goalCB, this, _1));
        as_.registerCancelCallback(boost::bind(&lwr_action_server::cancelCB, this, _1));
        as_.start();

        pub_joints_ = nh_.advertise<sensor_msgs::JointState>("/joint_states", 1);
        pub_motionCompletion_ = nh_.advertise<ros_fri::MotionCompletion>("/motion_completion", 1);
        pub_lwrStates_ = nh_.advertise<ros_fri::LWRStates>("/lwr_states", 1);

        goal_active_ = false;
        cancel_goal_ = false;
        seq_counterFJTF_ = 0;
        seqCounterJointStates_ = 0;
        seqCounterLWRStates_ = 0;
        FRI_ = new FastResearchInterface("/homeL/8liebrec/TAMS/FRILibrary/etc/980039-FRI-Driver.init");

        memset(commandedJointValuesInRad_, 0x0, LBR_MNJ * sizeof(float));
        memset(measuredJointValuesInRad_, 0x0, LBR_MNJ * sizeof(float));
        memset(TorqueValuesInRad_, 0x0, LBR_MNJ * sizeof(float));
        memset(EstimatedExternalJointTorquesInRad_, 0x0, LBR_MNJ * sizeof(float));
        memset(EstimatedExternalCartForcesAndTorques_, 0x0, LBR_MNJ * sizeof(float));
        memset(DriveTemperatues_, 0x0, LBR_MNJ * sizeof(float));

        int ResultValue = 0;

        ResultValue = FRI_->StartRobot(FastResearchInterface::JOINT_POSITION_CONTROL);

        //	ROS_INFO("MODE:%d", FRI_->GetFRIMode());
        //	FRI_->SetKRLIntValue(15, 30); //$FRI_FRM_INT[15]
        //	FRI_->WaitForKRCTick();
        //	ROS_INFO("MODE:%d", FRI_->GetFRIMode());

        //	ResultValue	= FRI_->StartRobot(FastResearchInterface::JOINT_IMPEDANCE_CONTROL );
        if ((ResultValue != EOK) && (ResultValue != EALREADY))
        {
            ROS_INFO("lwr_action_server: An error occurred during starting up the robot...\n");
            exit(EXIT_FAILURE);
        }

        FRI_->GetMeasuredJointPositions(measuredJointValuesInRad_);
        for (unsigned int j = 0; j < LBR_MNJ; j++)
        {
            Position_old_[j] = measuredJointValuesInRad_[j];
        }
        time_old_ = ros::Time::now();
        FRI_->WaitForKRCTick();
        FRI_->GetMeasuredJointPositions(measuredJointValuesInRad_);
        for (unsigned int j = 0; j < LBR_MNJ; j++)
        {
            Position_new_[j] = measuredJointValuesInRad_[j];
            Position_delta_[j] = Position_new_[j] - Position_old_[j];
        }
        time_new_ = ros::Time::now();
        time_delta_.sec = time_new_.sec - time_old_.sec;
        time_delta_.nsec = time_new_.nsec - time_old_.nsec;

        // TODO TEST
        CycleTime_ = 0.01;
        RML = new TypeIRML(LBR_MNJ, CycleTime_);
        IP = new TypeIRMLInputParameters(LBR_MNJ);
        OP = new TypeIRMLOutputParameters(LBR_MNJ);
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

            float CommandedTorquesInNm[LBR_MNJ], CommandedStiffness[LBR_MNJ], CommandedDamping[LBR_MNJ];

            for (unsigned int i = 0; i < LBR_MNJ; i++)
            {
                CommandedStiffness[i] = (float)200.0;
                CommandedDamping[i] = (float)0.7;
                CommandedTorquesInNm[i] = (float)0.0;
            }

            FRI_->SetCommandedJointStiffness(CommandedStiffness);
            FRI_->SetCommandedJointDamping(CommandedDamping);
            FRI_->SetCommandedJointTorques(CommandedTorquesInNm);

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
                    commandedJointValuesInRad_[j] = g->trajectory.points[i].positions[j];
                }

                FRI_->SetCommandedJointPositions(commandedJointValuesInRad_);

                publishJointStates();

                time_from_start = (ros::Time::now() - time_start);
                publishFeedback(g, i, time_from_start);

                mc_.stateOfExecution = (i + 1) * 100 / g->trajectory.points.size();
                pub_motionCompletion_.publish(mc_);

                // TODO TEST
                publishLWRStates();

                // check cancel request
                if (cancel_goal_)
                {
                    ROS_INFO("lwr_action_server: Goal canceled\n");
                    agh_.setCanceled();
                    cancel_goal_ = false;
                    stopLwr(g);
                    return;
                }

                // check measured and commanded JointValues and stop motion if necessary
                if (i > 0)
                {
                    for (unsigned int j = 0; j < LBR_MNJ; j++)
                    {
                        //	ROS_INFO("Don't touch me )) -- %f", fabs(commandedJointValuesInRad_[j] - measuredJointValuesInRad_[j]));
                        if (fabs(commandedJointValuesInRad_[j] - measuredJointValuesInRad_[j]) > 0.015)
                        {
                            ROS_INFO("lwr_action_server ERROR: The differenz between measured and commanded "
                                     "jointvalues is higher "
                                     "then threshold!");
                            // cancel Goalhandle
                            goal_active_ = false;
                            stopLwr(g);
                            agh_.setAborted();
                            return;
                        }
                    }
                }
            }
            // Trajectory finished all was fine //TODO publish Result??..
            agh_.setSucceeded();
            goal_active_ = false;
        }
    }

    /*
     void executeTrajectory()
    {

      if (goal_active_)
      {
        //send each trajectory point to FRI
          Goal g = agh_.getGoal();
          ros::Duration time_from_start;
        time_from_start.sec = 0;
        time_from_start.nsec = 0;
        ros::Time time_start ;

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
              commandedJointValuesInRad_[j] = g->trajectory.points[i].positions[j];
            }
            FRI_->SetCommandedJointPositions(commandedJointValuesInRad_);

            publishJointStates();

            time_from_start = (ros::Time::now() - time_start);
            publishFeedback(g, i, time_from_start);


            mc_.stateOfExecution = (i+1)*100 / g->trajectory.points.size();
            pub_motionCompletion_.publish(mc_);

            //TODO TEST
            publishLWRStates();

            //check cancel request
            if (cancel_goal_)
            {
              ROS_INFO("lwr_action_server: Goal canceled\n");
              agh_.setCanceled();
              cancel_goal_ = false;
              stopLwr(g);
              return;
            }

            //check measured and commanded JointValues and stop motion if necessary
            if (i > 0)
            {
              for (unsigned int j = 0; j < LBR_MNJ; j++)
              {
              //	ROS_INFO("Don't touch me )) -- %f", fabs(commandedJointValuesInRad_[j] -
    measuredJointValuesInRad_[j])); if (fabs(commandedJointValuesInRad_[j] - measuredJointValuesInRad_[j]) > 0.015)
                {
                  ROS_INFO("lwr_action_server ERROR: The differenz between measured and commanded jointvalues is higher
    then threshold!");
                   //cancel Goalhandle
                    goal_active_ = false;
                  stopLwr(g);
                  agh_.setAborted();
                  return;
                }
              }
            }


            //check EstimatedExternalJointTorques and abort execution
            FRI_->GetEstimatedExternalJointTorques(EstimatedExternalJointTorquesInRad_);
            for (unsigned int j = 0; j < LBR_MNJ; j++)
             {

                ROS_INFO("GetEstimatedExternalJointTorques (%f).", EstimatedExternalJointTorquesInRad_[j]);

          if (EstimatedExternalJointTorquesInRad_[j] > 0.1)	//$TORQUE_AXIS_EST
              {
                ROS_INFO("lwr_action_server: ERROR JointPositionOffset for joint(0-6): j is to high! (%f).",
    EstimatedExternalJointTorquesInRad_[j]); agh_.setAborted(); stopLwr(g); return;
              }

             }


          }
        //Trajectory finished all was fine //TODO publish Result??..
        agh_.setSucceeded();
        goal_active_ = false;
      }
    }
    */

    void cancelCB(GoalHandle gh)
    {
        // cancel FRI
        cancel_goal_ = true;
        // cancel Goalhandle
        goal_active_ = false;
    }

    void stopLwr(Goal g)
    {
        // load first position
        // FRI_->GetMeasuredJointPositions(measuredJointValuesInRad_);
        for (unsigned int j = 0; j < LBR_MNJ; j++)
        {
            IP->CurrentPosition->VecData[j] = (double)DEG(measuredJointValuesInRad_[j]);
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
                commandedJointValuesInRad_[j] = RAD((double)(OP->NewPosition->VecData[j]));
            }
            FRI_->SetCommandedJointPositions(commandedJointValuesInRad_);

            // prepare for next iteration
            *(IP->CurrentPosition) = *(OP->NewPosition);
            *(IP->CurrentVelocity) = *(OP->NewVelocity);
        }
    }

    void publishFeedback(Goal g, unsigned int counter, ros::Duration time_from_start)
    {
        FRI_->GetMeasuredJointPositions(measuredJointValuesInRad_);
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
            actual.positions.push_back((double)measuredJointValuesInRad_[j]);
            desired.positions.push_back(g->trajectory.points[counter].positions[j]);
            error.positions.push_back(fabs(g->trajectory.points[counter].positions[j]) -
                                      fabs(measuredJointValuesInRad_[j]));

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

        FRI_->GetMeasuredJointPositions(measuredJointValuesInRad_);
        FRI_->GetMeasuredJointTorques(TorqueValuesInRad_);

        time_old_ = time_new_;
        time_new_ = ros::Time::now();

        for (unsigned int j = 0; j < LBR_MNJ; j++)
        {
            Position_old_[j] = Position_new_[j];
            Position_new_[j] = measuredJointValuesInRad_[j];
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
            js.position.push_back(measuredJointValuesInRad_[i]);
            js.velocity.push_back(velocity_new_[i]);
            js.effort.push_back(TorqueValuesInRad_[i]);
        }

        pub_joints_.publish(js);
        seqCounterJointStates_++;
        pthread_mutex_unlock(&mutex);
    }

    void publishLWRStates()
    {
        ros_fri::LWRStates lwrStates;
        lwrStates.header.seq = seqCounterLWRStates_;
        lwrStates.header.stamp = ros::Time::now();
        lwrStates.header.frame_id = "calib_lwr_arm_base_link";

        FRI_->GetEstimatedExternalJointTorques(EstimatedExternalJointTorquesInRad_);
        FRI_->GetEstimatedExternalCartForcesAndTorques(EstimatedExternalCartForcesAndTorques_);
        FRI_->GetDriveTemperatures(DriveTemperatues_);

        for (unsigned int j = 0; j < LBR_MNJ; j++)
        {
            lwrStates.EstimatedExternalJointTorques.push_back(EstimatedExternalJointTorquesInRad_[j]);
            lwrStates.DriveTemperatures.push_back(DriveTemperatues_[j]);
        }

        for (unsigned int j = 0; j < 12; j++)
        {
            lwrStates.EstimatedExternalCartForcesAndTorques.push_back(EstimatedExternalCartForcesAndTorques_[j]);
        }

        // lwrStates_.CompleteRobotStateAndInformation	= FRI_->GetCompleteRobotStateAndInformation();
        pub_lwrStates_.publish(lwrStates);
        seqCounterLWRStates_++;
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "lwr_action_server");
    lwr_action_server lwr_action_server(ros::this_node::getName());
    //  important
    ros::AsyncSpinner spinner(1);
    spinner.start();
    ros::Rate loop_rate(10);  // 100Hz

    while (ros::ok())
    {
        lwr_action_server.publishJointStates();
        lwr_action_server.executeTrajectory();
        lwr_action_server.publishLWRStates();
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
