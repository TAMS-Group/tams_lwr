// ros_control interface for the KUKA LWR robot, using "Fast Research Interface"
// - 2022, Hongzhuo Liang
// - 2018, Philipp Ruppel

#include <controller_manager/controller_manager.h>
#include <hardware_interface/force_torque_sensor_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <ros/node_handle.h>
#include <RMLPositionInputParameters.h>
#include <RMLPositionOutputParameters.h>
#include <FastResearchInterface.h>
#include <LinuxAbstraction.h>
#include <ros/callback_queue.h>

class LWR : public hardware_interface::RobotHW
{
    // joint names
    std::vector<std::string> joint_names = {
        "lwr_arm_0_joint", "lwr_arm_1_joint", "lwr_arm_2_joint", "lwr_arm_3_joint",
        "lwr_arm_4_joint", "lwr_arm_5_joint", "lwr_arm_6_joint",
    };

    // ros interface for sending back measured positions / velocities / efforts
    hardware_interface::JointStateInterface joint_state_interface;

    // ros position control interface
    hardware_interface::PositionJointInterface joint_position_interface;
    std::vector<double> cmd, pos, vel, eff, prev;
    std::vector<float> tmp;

    // the fri interface for communicating with the robot
    std::unique_ptr<FastResearchInterface> fri;

    // maximum joint-space difference between two robot commands
    double max_step_size = 0.0004;

    std::vector<double> lower_joint_limits = {
        -2.90, -2.00, -2.90, -2.00, -2.90, -2.00, -2.90,
    };
    std::vector<double> upper_joint_limits = {
        +2.90, +2.00, +2.90, +2.00, +2.90, +2.00, +2.90,
    };

    std::vector<double> joint_diff_weights = {
        1, 1, 1, 1, 0.4, 0.4, 0.25,
    };

    // force/torque interface for the end-effector
    hardware_interface::ForceTorqueSensorInterface force_torque_interface;
    std::vector<double> force{ 3, 0.0 }, torque{ 3, 0.0 };
    std::vector<float> tmp_force_torque{ 6, 0.0 };

    double max_joint_step = 1.0 / 100;
    ros::Time time_old_, time_new_;
    double Position_old_[LBR_MNJ], Position_new_[LBR_MNJ];
public:
    LWR()
    {
        // create robot interface
        ROS_INFO("creating fast research interface");
        fri.reset(new FastResearchInterface("/opt/FRILibrary/etc/980039-FRI-Driver.init"));
        ROS_INFO("fast research interface created");

        // usleep(1000 * 1000);

        ROS_INFO("fri mode %i", fri->GetFRIMode());
        ROS_INFO("power %i", fri->IsRobotArmPowerOn());
        ROS_INFO("any drive error %i", fri->DoesAnyDriveSignalAnError());
        ROS_INFO("any drive warning %i", fri->DoesAnyDriveSignalAWarning());

        // init buffers
        cmd.resize(joint_names.size(), 0.0);
        pos.resize(joint_names.size(), 0.0);
        vel.resize(joint_names.size(), 0.0);
        eff.resize(joint_names.size(), 0.0);
        tmp.resize(joint_names.size(), 0.0);
        prev.resize(joint_names.size(), 0.0);

        // init all joints
        for (size_t i = 0; i < joint_names.size(); i++)
        {
            // register joint state interface
            joint_state_interface.registerHandle(
                hardware_interface::JointStateHandle(joint_names[i], &pos[i], &vel[i], &eff[i]));

            // register position control interface
            joint_position_interface.registerHandle(
                hardware_interface::JointHandle(joint_state_interface.getHandle(joint_names[i]), &cmd[i]));
        }

        // init end-effector force/torque interface
        force_torque_interface.registerHandle(
            hardware_interface::ForceTorqueSensorHandle("tool_force_torque", "tcp", force.data(), torque.data()));

        // register interfaces
        registerInterface(&joint_state_interface);
        registerInterface(&joint_position_interface);
        registerInterface(&force_torque_interface);
        time_new_ = ros::Time::now();
    }

    // start robot
    // needs to be called before sending any commands to the robot
    bool start()
    {
        // start robot and set error flag
        ROS_INFO("starting robot");
        {
            auto result = fri->StartRobot(FastResearchInterface::JOINT_POSITION_CONTROL, 5);
            if (result != EOK)
            {
                ROS_WARN_STREAM("failed to start robot error " << result << " try again...");
                // try again
                auto result = fri->StartRobot(FastResearchInterface::JOINT_POSITION_CONTROL, 5);
                if (result != EOK)
                {
                    ROS_ERROR_STREAM("failed to start robot error " << result << " stop robot...");
                    return false;
                }
            }
            else
            {
                ROS_INFO("robot started");
            }
        }
        // usleep(1000000);
        if (!(fri->IsMachineOK()))
        {
            ROS_ERROR_STREAM("machine is not ok");
            return false;
        }

        ROS_INFO("fri mode %i", fri->GetFRIMode());
        ROS_INFO("fri control mode %i", fri->GetCurrentControlScheme());
        ROS_INFO("power %i", fri->IsRobotArmPowerOn());
        ROS_INFO("any drive error %i", fri->DoesAnyDriveSignalAnError());
        ROS_INFO("any drive warning %i", fri->DoesAnyDriveSignalAWarning());
        ROS_INFO_STREAM("cycle time " << fri->GetFRICycleTime());
        ROS_INFO_STREAM("current communication timing quality " << fri->GetCommunicationTimingQuality());

        for (auto& f : tmp)
        {
            f = 0.0;
        }
        fri->GetMeasuredJointPositions(tmp.data());
        for (auto& f : tmp)
        {
            if (!std::isfinite(f) || f == 0.0)
            {
                ROS_ERROR_STREAM("failed to get joint positions");
                return false;
            }
        }
        cmd.assign(tmp.begin(), tmp.end());
        prev = cmd;

        return true;
    }

    void stop()
    {
        // stop robot
        ROS_INFO("stopping robot");
        puts("stopping robot");
        fri->StopRobot();
        ROS_INFO("robot stopped");
        puts("robot stopped");
    }

    ~LWR()
    {
        stop();
    }

    // checks if everything is ok with the robot or if we should shut down
    // bool ok() { return robot_ok; }

    bool wait()
    {
        if (!(fri->IsMachineOK()))
        {
            return false;
        }
        return (EOK == fri->WaitForKRCTick(100));
    }

    // read current state and send commands
    bool read()
    {
        // check if robot is ok
        if (!(fri->IsMachineOK()))
        {
            return false;
        }

        // get positions and convert from float to double
        fri->GetMeasuredJointPositions(tmp.data());
        pos.assign(tmp.begin(), tmp.end());

        // get efforts and convert from float to double
        // fri->GetMeasuredJointTorques(tmp.data());
        fri->GetEstimatedExternalJointTorques(tmp.data());
        eff.assign(tmp.begin(), tmp.end());

        // calculate velocities
        time_old_ = time_new_;
        time_new_ = ros::Time::now();
        for (unsigned int j = 0; j < LBR_MNJ; j++)
        {
            Position_old_[j] = Position_new_[j];
            Position_new_[j] = pos[j];
            vel[j] = (Position_new_[j] - Position_old_[j]) / (time_new_.toSec() - time_old_.toSec());
        }

        // read tool force/torque
        fri->GetEstimatedExternalCartForcesAndTorques(tmp_force_torque.data());
        force[0] = tmp_force_torque[0];
        force[1] = tmp_force_torque[1];
        force[2] = tmp_force_torque[2];
        torque[0] = tmp_force_torque[3];
        torque[1] = tmp_force_torque[4];
        torque[2] = tmp_force_torque[5];

        return true;
    }

    bool write()
    {
        // check if robot is ok
        if (!(fri->IsMachineOK()))
        {
            ROS_ERROR_STREAM("robot is not ok");
            return false;
        }

        bool cmd_all_zero = true;
        for (auto& f : cmd)
        {
            if (f != 0.0)
            {
                cmd_all_zero = false;
            }
        }
        if (cmd_all_zero)
        {
            ROS_ERROR_STREAM("all command positions are zero");
            return false;
        }
        for (auto& f : cmd)
        {
            if (!std::isfinite(f))
            {
                ROS_ERROR_STREAM("command positions not finite");
                return false;
            }
        }

        for (size_t i = 0; i < cmd.size(); i++)
        {
            if (std::abs(cmd[i] - prev[i]) > max_joint_step)
            {
                ROS_ERROR_STREAM("step for joint " << i << " is " << std::abs(cmd[i] - prev[i]) << "which is larger than max_joint_step: "  << max_joint_step);
                return false;
            }
        }

        tmp.assign(cmd.begin(), cmd.end());
        for (auto& f : tmp)
        {
            if (!std::isfinite(f))
            {
                ROS_ERROR_STREAM("command positions not finite");
                return false;
            }
        }

        fri->SetCommandedJointPositions(tmp.data());
        prev.assign(cmd.begin(), cmd.end());
        return true;
    }
};

int main(int argc, char** argv)
{
    // init ros node
    ros::init(argc, argv, "ros_fri_roscontrol" /*, ros::init_options::NoSigintHandler*/);

    ros::NodeHandle node_handle;

    // ros::CallbackQueue queue;
    // node_handle.setCallbackQueue(&queue);

    ROS_INFO("ros fri roscontrol");

    {
        // robot interface for sending commands to the robot and receiving joint feedback
        LWR robot;

        // ros controller manager for our robot, need to periodically call update on it
        controller_manager::ControllerManager cm(&robot);

        // start robot, needs to try twice until it starts
        if (!robot.start())
        {
            ROS_ERROR_STREAM("failed to start robot");
            return -1;
        }

        // async spinner running in the background
        ros::AsyncSpinner spinner(2);
        spinner.start();

        ros::Time ros_last_time = ros::Time::now();

        // control loop
        while (ros::ok())
        {
            // read joint states from robot
            bool read_ok = robot.read();

            if (!read_ok)
            {
                ROS_ERROR_STREAM("failed to read lwr robot state");
                break;
            }

            if (read_ok)
            {
                // get ros time
                ros::Time ros_time = ros::Time::now();
                ros::Duration elapsed_time = ros_time - ros_last_time;
                ros_last_time = ros_time;

                // update controller manager
                cm.update(ros_time, elapsed_time);

                // send commands to robot
                if (!robot.write())
                {
                    ROS_ERROR_STREAM("failed to send command");
                    break;
                }
            }

            // sleep
            // TODO: update rate ?
            robot.wait();
        }

        robot.stop();

        usleep(100000);
    }

    ros::shutdown();

    ROS_INFO("shutting down");
    puts("shutting down");
}
