// ros_control interface for the KUKA LWR robot, using "Fast Research Interface"
// - 2018, Philipp Ruppel

#include <controller_interface/controller.h>
#include <controller_manager/controller_manager.h>
#include <hardware_interface/force_torque_sensor_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <pluginlib/class_list_macros.h>
#include <ros/node_handle.h>

#include <RMLPositionFlags.h>
#include <RMLPositionInputParameters.h>
#include <RMLPositionOutputParameters.h>
#include <RMLVelocityInputParameters.h>
#include <RMLVelocityOutputParameters.h>
#include <ReflexxesAPI.h>

#include <FastResearchInterface.h>
#include <LinuxAbstraction.h>

#include <time.h>

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
    std::vector<double> cmd, pos, vel, eff, out;
    std::vector<float> tmp;

    // the fri interface for communicating with the robot
    std::unique_ptr<FastResearchInterface> fri;

    // if the robot has been started successfully
    bool robot_ok = false;

    // maximum joint-space difference between two robot commands
    double max_step_size = 0.002;

    std::vector<double> lower_joint_limits = {
        -2.90, -2.00, -2.90, -2.00, -2.90, -2.00, -2.90,
    };
    std::vector<double> upper_joint_limits = {
        +2.90, +2.00, +2.90, +2.00, +2.90, +2.00, +2.90,
    };

    // force/torque interface for the end-effector
    hardware_interface::ForceTorqueSensorInterface force_torque_interface;
    std::vector<double> force{ 3, 0.0 }, torque{ 3, 0.0 };
    std::vector<float> tmp_force_torque{ 6, 0.0 };

public:
    LWR()
    {
        // create robot interface
        ROS_INFO("creating fast research interface");
        fri.reset(new FastResearchInterface("/opt/FRILibrary/etc/980039-FRI-Driver.init"));
        ROS_INFO("fast research interface created");

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
        out.resize(joint_names.size(), 0.0);

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
    }

    // start robot
    // needs to be called before sending any commands to the robot
    void start()
    {
        ROS_INFO("fri mode %i", fri->GetFRIMode());
        ROS_INFO("power %i", fri->IsRobotArmPowerOn());
        ROS_INFO("any drive error %i", fri->DoesAnyDriveSignalAnError());
        ROS_INFO("any drive warning %i", fri->DoesAnyDriveSignalAWarning());

        // start robot and set error flag
        ROS_INFO("starting robot");
        {
            auto result = fri->StartRobot(FastResearchInterface::JOINT_IMPEDANCE_CONTROL);
            // fri->StartRobot(FastResearchInterface::JOINT_POSITION_CONTROL);
            if ((result != EOK) && (result != EALREADY))
            {
                ROS_ERROR("failed to start robot");
                robot_ok = false;
            }
            else
            {
                ROS_INFO("robot started");
                robot_ok = true;
            }
        }
        if (!robot_ok)
        {
            return;
        }

        // ros::Duration(1.0).sleep();

        ROS_INFO("fri mode %i", fri->GetFRIMode());
        ROS_INFO("power %i", fri->IsRobotArmPowerOn());
        ROS_INFO("any drive error %i", fri->DoesAnyDriveSignalAnError());
        ROS_INFO("any drive warning %i", fri->DoesAnyDriveSignalAWarning());

        // get start positions
        for (auto& f : tmp)
        {
            f = 0.0;
        }
        fri->GetCommandedJointPositions(tmp.data());
        for (auto& f : tmp)
        {
            ROS_INFO("%f", f);
            if (!std::isfinite(f) || f == 0.0)
            {
                ROS_INFO("failed to get joint positions");
                robot_ok = false;
                return;
            }
        }
        out.assign(tmp.begin(), tmp.end());

        // set stiffness
        for (auto& f : tmp)
        {
            f = 300;
        }
        /*tmp[0] = 150;
        tmp[1] = 150;
        tmp[2] = 150;
        tmp[3] = 150;
        tmp[4] = 50;
        tmp[5] = 50;
        tmp[6] = 50;*/
        /*for (size_t i = 0; i < tmp.size(); i++) {
          tmp[i] = 75.0 - i * 50.0 / (tmp.size() - 1);
        }*/
        fri->SetCommandedJointStiffness(tmp.data());

        // set damping
        for (auto& f : tmp)
        {
            // f = 0.8;
            f = 0.85;
        }
        // tmp[6] = 5.0;
        fri->SetCommandedJointDamping(tmp.data());
    }

    ~LWR()
    {
        // stop robot
        ROS_INFO("stopping robot");
        puts("stopping robot");
        fri->StopRobot();
        ROS_INFO("robot stopped");
        puts("robot stopped");

        // cleanup / delete robot interface
        /*ROS_INFO("deleting fast research interface");
        puts("deleting fast research interface");
        fri.reset();
        ROS_INFO("fast research interface deleted");
        puts("fast research interface deleted");*/

        // Yup, this is a memory leak,
        // but the fri destructor apparently has some syncrhonization bugs,
        // so without leaking the fri interface, this node would never shut down.
        // Doesn't really seem to be a problem through - if we make sure that we
        // call StopRobot (see above) !
        fri.release();
    }

    // checks if everything is ok with the robot or if we should shut down
    bool ok()
    {
        return robot_ok;
    }

    // read current state / joint feedback from robot
    void read()
    {
        // check if robot is ok
        if (!fri->IsMachineOK())
        {
            // robot_ok = false;
            ROS_ERROR("robot not ok");
            return;
        }
        /*if (!robot_ok) {
          ROS_ERROR("robot not ok");
          return;
        }*/

        // get positions and convert from float to double
        fri->GetMeasuredJointPositions(tmp.data());
        pos.assign(tmp.begin(), tmp.end());

        // get efforts and convert from float to double
        // fri->GetMeasuredJointTorques(tmp.data());
        fri->GetEstimatedExternalJointTorques(tmp.data());
        eff.assign(tmp.begin(), tmp.end());

        // TODO: velocities

        // read tool force/torque
        fri->GetEstimatedExternalCartForcesAndTorques(tmp_force_torque.data());
        force[0] = tmp_force_torque[0];
        force[1] = tmp_force_torque[1];
        force[2] = tmp_force_torque[2];
        torque[0] = tmp_force_torque[3];
        torque[1] = tmp_force_torque[4];
        torque[2] = tmp_force_torque[5];
    }

    // interpolate between a and b
    double mix(double a, double b, double f)
    {
        return b * f + a * (1.0 - f);
    }

    // send commands to robot
    void write()
    {
        // ROS_INFO("write");

        // check if robot is ok
        if (!fri->IsMachineOK())
        {
            robot_ok = false;
            ROS_ERROR("robot not ok");
            return;
        }
        /*if (!robot_ok) {
          ROS_ERROR("robot not ok");
          return;
        }*/

        bool cmdAllZero = true;
        for (auto& f : cmd)
        {
            if (f != 0.0)
            {
                cmdAllZero = false;
            }
        }

        for (auto& f : cmd)
        {
            // ROS_INFO("cmd %f", f);
            if (!std::isfinite(f))
            {
                ROS_INFO("invalid joint positions");
                robot_ok = false;
                return;
            }
        }

        if (cmdAllZero)
        {
            ROS_INFO("cmd all zero, ignored");
        }
        else
        {
            for (size_t joint_index = 0; joint_index < joint_names.size(); joint_index++)
            {
                cmd[joint_index] = std::max(cmd[joint_index], lower_joint_limits[joint_index]);
                cmd[joint_index] = std::min(cmd[joint_index], upper_joint_limits[joint_index]);
            }

            // don't move more than max_step_size
            double factor = 1.0;
            double largest_difference = 0.0;
            for (size_t joint_index = 0; joint_index < joint_names.size(); joint_index++)
            {
                largest_difference = std::max(largest_difference, std::abs(out[joint_index] - cmd[joint_index]));
            }
            if (largest_difference > max_step_size)
            {
                factor = max_step_size / largest_difference;
            }

            // update position command
            for (size_t joint_index = 0; joint_index < joint_names.size(); joint_index++)
            {
                out[joint_index] = (float)mix(out[joint_index], cmd[joint_index], factor);
            }
        }

        /*// apply limits
        for (size_t joint_index = 0; joint_index < joint_names.size();
             joint_index++) {
          out[joint_index] =
              std::max(out[joint_index], lower_joint_limits[joint_index]);
          out[joint_index] =
              std::min(out[joint_index], upper_joint_limits[joint_index]);
        }*/

        // send position command
        tmp.assign(out.begin(), out.end());

        for (auto& f : out)
        {
            // ROS_INFO("out %f", f);
            if (!std::isfinite(f))
            {
                ROS_INFO("invalid joint positions");
                robot_ok = false;
                return;
            }
        }

        fri->SetCommandedJointPositions(tmp.data());
    }
};

int main(int argc, char** argv)
{
    // init ros node
    ros::init(argc, argv, "ros_fri_lwr" /*, ros::init_options::NoSigintHandler*/);
    ros::NodeHandle node_handle;

    // async spinner running in the background
    ros::AsyncSpinner spinner(4);
    spinner.start();

    // robot interface for sending commands to the robot and receiving joint
    // feedback
    LWR robot;

    // ros controller manager for our robot, need to periodically call update on
    // it
    controller_manager::ControllerManager cm(&robot);

    ros::Duration(1.0).sleep();

    // start robot
    robot.start();

    // accurate time measurement
    // timespec last_time;
    // clock_gettime(CLOCK_MONOTONIC, &last_time);

    ros::Time ros_last_time = ros::Time::now();

    // control loop
    while (ros::ok() && robot.ok())
    {  // shutdown cleanly

        // read joint states from robot
        robot.read();

        /*// accurate time measurement
        timespec current_time;
        clock_gettime(CLOCK_MONOTONIC, &current_time);

        // compute elapsed time
        ros::Duration elapsed_time(current_time.tv_sec - last_time.tv_sec +
                                   (current_time.tv_nsec - last_time.tv_nsec) *
                                       (1.0 / 1000000000.0));
        ros::Duration elapsed_time = current_time - last_time;
        last_time = current_time;*/

        // get ros time
        ros::Time ros_time = ros::Time::now();
        ros::Duration elapsed_time = ros_time - ros_last_time;
        ros_last_time = ros_time;

        // update controller manager
        cm.update(ros_time, elapsed_time);

        // send commands to robot
        robot.write();

        // sleep
        // TODO: update rate ?
        usleep(1000);
        // usleep(200);
    }

    ROS_INFO("shutting down");
    puts("shutting down");
}
