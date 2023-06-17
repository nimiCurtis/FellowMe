#include <fellowme_base/fellowme_hw_interface.h>

// ROS parameter loading
#include <rosparam_shortcuts/rosparam_shortcuts.h>

//#include <std_msgs/Float32.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Empty.h>

#include <iomanip>
 
namespace fellowme_base
{
    FellowmeHWInterface::FellowmeHWInterface(ros::NodeHandle &nh, urdf::Model *urdf_model)
        : name_("hardware_interface")
        , nh_(nh)
    { 
        // Initialization of the robot's resources (joints, sensors, actuators) and
        // interfaces can be done here or inside init().
        // E.g. parse the URDF for joint names & interfaces, then initialize them
        // Check if the URDF model needs to be loaded
        if (urdf_model == NULL)
            loadURDF(nh, "robot_description");
        else
            urdf_model_ = urdf_model;

        // Load rosparams
        ros::NodeHandle rpnh(nh_, name_);
        std::size_t error = 0;
        // Code API of rosparam_shortcuts:
        // http://docs.ros.org/en/noetic/api/rosparam_shortcuts/html/namespacerosparam__shortcuts.html#aa6536fe0130903960b1de4872df68d5d
        error += !rosparam_shortcuts::get(name_, rpnh, "joints", joint_names_);
        error += !rosparam_shortcuts::get(name_, nh_, "mobile_base_controller/wheel_radius", wheel_radius_);
        error += !rosparam_shortcuts::get(name_, nh_, "mobile_base_controller/linear/x/max_velocity", max_velocity_);
        // Get additional parameters from the fellowme_base/config/base.yaml which is stored on the parameter server
        error += !rosparam_shortcuts::get(name_, nh_, "gain", gain_);
        error += !rosparam_shortcuts::get(name_, nh_, "trim", trim_);
        error += !rosparam_shortcuts::get(name_, nh_, "motor_constant", motor_constant_);
        error += !rosparam_shortcuts::get(name_, nh_, "pwm_limit", pwm_limit_);
        error += !rosparam_shortcuts::get(name_, nh_, "debug/hardware_interface", debug_);
        rosparam_shortcuts::shutdownIfError(name_, error);

        wheel_diameter_ = 2.0 * wheel_radius_;
        //max_velocity_ = 0.2; // m/s
        // ros_control RobotHW needs velocity in rad/s but in the config its given in m/s
        max_velocity_ = linearToAngular(max_velocity_);


        ROS_INFO_STREAM("mobile_base_controller/wheel_radius: " << wheel_radius_);
        ROS_INFO_STREAM("mobile_base_controller/linear/x/max_velocity: " << max_velocity_);
        ROS_INFO_STREAM("pwm_limit: " << pwm_limit_);

        //Setup publisher for angular wheel joint velocity commands
        pub_wheel_cmd_velocities_ = nh_.advertise<fellowme_msgs::WheelsCmdStamped>("wheel_cmd_velocities", 10);

        // Setup publisher to reset wheel encoders (used during first launch of the hardware interface)
        // Setup subscriber for the wheel encoders
        sub_encoder_ticks_ = nh_.subscribe("fellowme_base/encoders/ticks", 10, &FellowmeHWInterface::encoderTicksCallback, this);
        sub_measured_joint_states_ = nh_.subscribe("fellowme_base/measured_joint_states", 10, &FellowmeHWInterface::measuredJointStatesCallback, this);

        // Initialize the hardware interface
        init(nh_, nh_);

        // Wait for encoder messages being published
        isReceivingMeasuredJointStates(ros::Duration(10));
    }

 
    bool FellowmeHWInterface::init(ros::NodeHandle &root_nh, ros::NodeHandle &robot_hw_nh)
    {
        ROS_INFO("Initializing Fellowme Hardware Interface ...");
        num_joints_ = joint_names_.size();
        ROS_INFO("Number of joints: %d", (int)num_joints_);
        std::array<std::string, NUM_JOINTS> motor_names = {"left_motor", "right_motor"};
        for (unsigned int i = 0; i < num_joints_; i++)
        {
            // Create a JointStateHandle for each joint and register them with the 
            // JointStateInterface.
            hardware_interface::JointStateHandle joint_state_handle(joint_names_[i],
                                                                    &joint_positions_[i], 
                                                                    &joint_velocities_[i],
                                                                    &joint_efforts_[i]);
            joint_state_interface_.registerHandle(joint_state_handle);

            // Create a JointHandle (read and write) for each controllable joint
            // using the read-only joint handles within the JointStateInterface and 
            // register them with the JointVelocityInterface.
            hardware_interface::JointHandle joint_handle(joint_state_handle, &joint_velocity_commands_[i]);
            velocity_joint_interface_.registerHandle(joint_handle);

            // Initialize joint states with zero values
            joint_positions_[i] = 0.0;
            joint_velocities_[i] = 0.0;
            joint_efforts_[i] = 0.0; // unused with diff_drive_controller

            joint_velocity_commands_[i] = 0.0;

            // Initialize encoder_ticks_ to zero because receiving meaningful
            // tick values from the microcontroller might take some time
            encoder_ticks_[i] = 0.0;
            measured_joint_states_[i].angular_position_ = 0.0;
            measured_joint_states_[i].angular_velocity_ = 0.0;
            measured_joint_states_[i].effort_ = 0.0;


            // Initialize the pid controllers for the motors using the robot namespace
            // std::string pid_namespace = "pid/" + motor_names[i];
            // ROS_INFO_STREAM("pid namespace: " << pid_namespace);
            // ros::NodeHandle nh(root_nh, pid_namespace);
            // // TODO implement builder pattern to initialize values otherwise it is hard to see which parameter is what.
            // pids_[i].init(nh, 0.8, 0.35, 0.5, 0.01, 3.5, -3.5, false, max_velocity_, -max_velocity_);
            // pids_[i].setOutputLimits(-max_velocity_, max_velocity_);
        }

        // Register the JointStateInterface containing the read only joints
        // with this robot's hardware_interface::RobotHW.
        registerInterface(&joint_state_interface_);

        // Register the JointVelocityInterface containing the read/write joints
        // with this robot's hardware_interface::RobotHW.
        registerInterface(&velocity_joint_interface_);

        ROS_INFO("... Done Initializing Fellowme Hardware Interface");

        return true;
    }

    void FellowmeHWInterface::read(const ros::Time& time, const ros::Duration& period)
    {
        //ROS_INFO_THROTTLE(1, "Read");
        ros::Duration elapsed_time = period;

        // Read from robot hw (motor encoders)
        // Fill joint_state_* members with read values
        for (std::size_t i = 0; i < num_joints_; ++i)
        {
            joint_positions_[i] = measured_joint_states_[i].angular_position_;
            joint_velocities_[i] = measured_joint_states_[i].angular_velocity_;
            joint_efforts_[i] = measured_joint_states_[i].effort_; // unused with diff_drive_controller
        }

        if (debug_)
        {
            const int width = 10;
            const char sep = ' ';
            std::stringstream ss;
            ss << std::left << std::setw(width) << std::setfill(sep) << "Read" << std::left << std::setw(width) << std::setfill(sep) << "ticks" << std::left << std::setw(width) << std::setfill(sep) << "angle" << std::left << std::setw(width) << std::setfill(sep) << "velocity" << std::endl;
            ss << std::left << std::setw(width) << std::setfill(sep) << "j0:" << std::left << std::setw(width) << std::setfill(sep) << encoder_ticks_[0] << std::left << std::setw(width) << std::setfill(sep) << joint_positions_[0] << std::left << std::setw(width) << std::setfill(sep) << joint_velocities_[0] << std::endl;
            ss << std::left << std::setw(width) << std::setfill(sep) << "j1:" << std::left << std::setw(width) << std::setfill(sep) << encoder_ticks_[1] << std::left << std::setw(width) << std::setfill(sep) << joint_positions_[1] << std::left << std::setw(width) << std::setfill(sep) << std::setfill(sep) << joint_velocities_[1];
            ROS_INFO_STREAM(std::endl << ss.str());
            //printState();
        }
    }

    void FellowmeHWInterface::write(const ros::Time& time, const ros::Duration& period)
    {
        ros::Duration elapsed_time = period;
        // Write to robot hw
        // joint velocity commands from ros_control's RobotHW are in rad/s

        // adjusting k by gain and trim
        double motor_constant_right_inv = (gain_ + trim_) / motor_constant_;
        double motor_constant_left_inv = (gain_ - trim_) / motor_constant_;


        joint_velocity_commands_[0] = joint_velocity_commands_[0] * motor_constant_left_inv;
        joint_velocity_commands_[1] = joint_velocity_commands_[1] * motor_constant_right_inv;


        // Publish the desired (commanded) angular wheel joint velocities
        fellowme_msgs::WheelsCmdStamped wheel_cmd_msg;
        wheel_cmd_msg.header.stamp = ros::Time::now();
        for (int i = 0; i < NUM_JOINTS; ++i)
        {
            wheel_cmd_msg.wheels_cmd.angular_velocities.joint.push_back(joint_velocity_commands_[i]);
        }

        pub_wheel_cmd_velocities_.publish(wheel_cmd_msg);

        if (debug_)
        {
            const int width = 10;
            const char sep = ' ';
            std::stringstream ss;
            // Header
            ss << std::left << std::setw(width) << std::setfill(sep) << "Write"
            << std::left << std::setw(width) << std::setfill(sep) << "velocity"
            << std::endl;

            ROS_INFO_STREAM(std::endl << ss.str());
        }
    }

    bool FellowmeHWInterface::isReceivingMeasuredJointStates(const ros::Duration &timeout)
    {
        ROS_INFO("Get number of measured joint states publishers");

        ros::Time start = ros::Time::now();
        int num_publishers = sub_measured_joint_states_.getNumPublishers();
        ROS_INFO("Waiting for measured joint states being published...");
        while ((num_publishers == 0) && (ros::Time::now() < start + timeout))
        {
            ros::Duration(0.1).sleep();
            num_publishers = sub_measured_joint_states_.getNumPublishers();
        }
        if (num_publishers == 0)
        {
            ROS_ERROR("No measured joint states publishers. Timeout reached.");
        }
        else
        {
            ROS_INFO_STREAM("Number of measured joint states publishers: " << num_publishers);
        }

        return (num_publishers > 0);
    }

    void FellowmeHWInterface::loadURDF(const ros::NodeHandle &nh, std::string param_name)
    {
        std::string urdf_string;
        urdf_model_ = new urdf::Model();

        // search and wait for robot_description on param server
        while (urdf_string.empty() && ros::ok())
        {
            std::string search_param_name;
            if (nh.searchParam(param_name, search_param_name))
            {
                ROS_INFO_STREAM_NAMED(name_, "Waiting for model URDF on the ROS param server at location: " <<
                                        nh.getNamespace() << search_param_name);
                nh.getParam(search_param_name, urdf_string);
            }
            else
            {
                ROS_INFO_STREAM_NAMED(name_, "Waiting for model URDF on the ROS param server at location: " <<
                                        nh.getNamespace() << param_name);
                nh.getParam(param_name, urdf_string);
            }

            usleep(100000);
        }

        if (!urdf_model_->initString(urdf_string))
            ROS_ERROR_STREAM_NAMED(name_, "Unable to load URDF model");
        else
            ROS_DEBUG_STREAM_NAMED(name_, "Received URDF from param server");
    }

    void FellowmeHWInterface::printState()
    {
        // WARNING: THIS IS NOT REALTIME SAFE
        // FOR DEBUGGING ONLY, USE AT YOUR OWN ROBOT's RISK!
        ROS_INFO_STREAM_THROTTLE(1, std::endl << printStateHelper());
    }

    std::string FellowmeHWInterface::printStateHelper()
    {
        std::stringstream ss;
        std::cout.precision(15);

        for (std::size_t i = 0; i < num_joints_; ++i)
        {
            ss << "j" << i << ": " << std::fixed << joint_positions_[i] << "\t ";
            ss << std::fixed << joint_velocities_[i] << "\t ";
            ss << std::fixed << joint_efforts_[i] << std::endl;
        }
        return ss.str();
    }

    std::string FellowmeHWInterface::printCommandHelper()
    {
        std::stringstream ss;
        std::cout.precision(15);
        ss << "    position     velocity         effort  \n";
        for (std::size_t i = 0; i < num_joints_; ++i)
        {
            ss << std::fixed << joint_velocity_commands_[i] << "\t ";
        }
        return ss.str();
    }


    /// Process updates from encoders
    void FellowmeHWInterface::encoderTicksCallback(const fellowme_msgs::EncodersStamped::ConstPtr& msg_encoder)
    {
        /// Update current encoder ticks in encoders array
        encoder_ticks_[0] = msg_encoder->encoders.ticks[0];
        encoder_ticks_[1] = msg_encoder->encoders.ticks[1];
        ROS_DEBUG_STREAM_THROTTLE(1, "Left encoder ticks: " << encoder_ticks_[0]);
        ROS_DEBUG_STREAM_THROTTLE(1, "Right encoder ticks: " << encoder_ticks_[1]);
    }

    void FellowmeHWInterface::measuredJointStatesCallback(const sensor_msgs::JointState::ConstPtr& msg_joint_states)
    {
        /// Update current encoder ticks in encoders array
        for (std::size_t i = 0; i < num_joints_; ++i)
        {
            measured_joint_states_[i].angular_position_ = msg_joint_states->position[i];
            measured_joint_states_[i].angular_velocity_ = msg_joint_states->velocity[i];
            measured_joint_states_[i].effort_ = msg_joint_states->effort[i];
        }
        //ROS_DEBUG_STREAM_THROTTLE(1, "Left encoder ticks: " << encoder_ticks_[0]);
        //ROS_DEBUG_STREAM_THROTTLE(1, "Right encoder ticks: " << encoder_ticks_[1]);
    }


    // double FellowmeHWInterface::ticksToAngle(const int &ticks) const
    // {
    //     // Convert number of encoder ticks to angle in radians
    //     double angle = (double)ticks * (2.0*M_PI / encoder_resolution_);
    //     ROS_DEBUG_STREAM_THROTTLE(1, ticks << " ticks correspond to an angle of " << angle);
	//     return angle;
    // }

    // double FellowmeHWInterface::normalizeAngle(double &angle) const
    // {
    //     // https://stackoverflow.com/questions/11498169/dealing-with-angle-wrap-in-c-code
    //     angle = fmod(angle, 2.0*M_PI);

    //     if (angle < 0)
    //         angle += 2.0*M_PI;

    //     ROS_DEBUG_STREAM_THROTTLE(1, "Normalized angle: " << angle);
    //     return angle;
    // }


    double FellowmeHWInterface::linearToAngular(const double &distance) const
    {
        return distance / wheel_diameter_ * 2.0;
    }

    double FellowmeHWInterface::angularToLinear(const double &angle) const
    {
        return angle * wheel_diameter_ / 2.0;
    }

};