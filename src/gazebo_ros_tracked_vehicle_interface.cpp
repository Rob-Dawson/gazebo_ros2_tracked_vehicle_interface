// Tidle ~ is no longer supported in ROS2 and thus has been changed

#include "gazebo_ros_tracked_vehicle_interface.hpp"

using std::placeholders::_1;
namespace gazebo
{

    // Tracks
    enum
    {
        RIGHT,
        LEFT,
    };
    GazeboRosTrackedVehicleInterface::GazeboRosTrackedVehicleInterface()
        : ModelPlugin()
    {
        printf("Hello World");
        msgs::Set(cmd_vel.mutable_linear(), ignition::math::Vector3d::Zero);
        msgs::Set(cmd_vel.mutable_angular(), ignition::math::Vector3d::Zero);
    }

    // GazeboRosTrackedVehicleInterface::~GazeboRosTrackedVehicleInterface()
    // {
    //     alive_ = false;

    // }
    template <typename Type>
    Type GazeboRosTrackedVehicleInterface::params(const sdf::ElementPtr &_sdf, const std::string &topicName, const Type &defaultName)
    {
        if (_sdf->HasElement(topicName))
        {
            return _sdf->Get<Type>(topicName);
        }
        else
        {
            return defaultName;
            RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "Parameter Not Found " << command_ros_topic_.c_str());
        }
    }

    template <typename Type>
    Type GazeboRosTrackedVehicleInterface::params(const sdf::ElementPtr &_sdf, const std::string &paramName, const std::map<std::string, Type> &options, const Type &defaultValue)
    {
        if (_sdf->HasElement(paramName))
        {
            std::string value = _sdf->Get<std::string>(paramName);
            auto it = options.find(value);
            if (it != options.end())
            {
                return it->second;
            }
            else
            {
                RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "Invalid value '%s' for parameter '%s'. Using default.", value.c_str(), paramName.c_str());
            }
        }
        return defaultValue;
    }



    void GazeboRosTrackedVehicleInterface::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
    {

        this->m_parent = _parent;
        ros_node = gazebo_ros::Node::Get(_sdf);

        command_ros_topic_ = params<std::string>(_sdf, "commandROSTopic", "cmd_vel_ros");
        command_ign_topic_ = params<std::string>(_sdf, "commandIGNTopic", "cmd_vel_ign");
        odometry_topic_ = params<std::string>(_sdf, "odometryTopic", "odom");
        odometry_frame_ = params<std::string>(_sdf, "odometryFrame", "odom");
        robot_base_frame_ = params<std::string>(_sdf, "robotBaseFrame", "base_footprint");
        track_speed_topic_ = params<std::string>(_sdf, "trackSpeedTopic", "tracks_speed");

        this->m_ignition_node.reset(new transport::Node());
        this->m_ignition_node->Init(_parent->GetWorld()->Name());

        publishOdomTF_ = params<bool>(_sdf, "publishOdomTF", true);
        track_separation_ = params<double>(_sdf, "tracks_separation", 0.34);
        update_rate = params<double>(_sdf, "updateRate", 100.0);

        RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "commandROSTopic " << command_ros_topic_.c_str());
        RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "commandIGNTopic " << command_ign_topic_.c_str());
        RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "odometryTopic " << odometry_topic_.c_str());
        RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "odometryFrame " << odometry_frame_.c_str());
        RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "robotBaseFrame " << robot_base_frame_.c_str());
        RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "trackSpeedTopic " << track_speed_topic_.c_str());
        RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "publishOdomTF: " << (publishOdomTF_ ? "true" : "false"));
        RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "tracks_separation " << track_separation_);
        RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "updateRate " << update_rate);

        // Choose where the odometry information comes from
        // Need to test this and see how it works
        std::map<std::string, OdomSource> odomOptions;
        odomOptions["encoder"] = ENCODER;
        odomOptions["world"] = WORLD;

        odom_source = params(_sdf, "odometrySource", odomOptions, WORLD);
        this->publish_tf_ = true;
        if (!_sdf->HasElement("publishTf"))
        {
            RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "GazeboRosTrackedVehicleInterface Plugin (ns = %s) missing <publishTf>, defaults to %d" << this->robot_namespace_.c_str() << this->publish_tf_);
        }
        else
        {
            this->publish_tf_ = _sdf->GetElement("publishTf")->Get<bool>();
        }
        if (this->update_rate > 0.0)
            this->update_period = 1.0 / this->update_rate;
        else
            this->update_period = 0.0;
        last_update_time = m_parent->GetWorld()->SimTime();

        // Initialize velocity stuff
        track_speed[RIGHT] = 0;
        track_speed[LEFT] = 0;

        alive_ = true;

        // /* SETUP OF ROS AND IGNITION PUB SUBS & CO */
        transformBroadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(ros_node);

        // auto callback = std::bind(&GazeboRosTrackedVehicleInterface::cmdVelCallback, this, _1);
        // auto subscription = ros_node->create_subscription<geometry_msgs::msg::Twist>(command_ros_topic_, 1, callback);

        cmd_vel_ros = ros_node->create_subscription<geometry_msgs::msg::Twist>(
            command_ros_topic_, 10,
            [this](const geometry_msgs::msg::Twist::SharedPtr msg)
            {
                this->cmdVelCallback(msg);
            });
        if (this->publish_tf_)
        {
            RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), odometry_topic_);

            odom = ros_node->create_publisher<nav_msgs::msg::Odometry>(odometry_topic_, 10);
            RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), odometry_topic_);
        }
 // listen to the update event (broadcast every simulation iteration)
        this->update_connection = gazebo::event::Events::ConnectWorldUpdateBegin(
            [this](const gazebo::common::UpdateInfo &info)
            {
                this->OnUpdate(info);
            });

        this->tracks_vel_subscriber_ign = this->m_ignition_node->Subscribe<msgs::Vector2d, GazeboRosTrackedVehicleInterface>(track_speed_topic_, &GazeboRosTrackedVehicleInterface::OnTrackVelMsg, this);


        // tracks_vel_subscriber_ign = ros_node->create_subscription<msgs::Vector2d, GazeboRosTrackedVehicleInterface>(track_speed_topic_, &GazeboRosTrackedVehicleInterface::OnTrackVelMsg, this);
// ROS 2 Subscriber

        // ROS PUB SUB & CO

        this->cmd_vel_publisher_ign = this->m_ignition_node->Advertise<msgs::Twist>(command_ign_topic_);

        // if(_sdf->HasElement("commandROSTopic"))
        // {
        //     command_ros_topic_ = _sdf->Get<std::string>("commandROSTopic");
        //     RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "Parameter Found: " << command_ros_topic_.c_str());

        // }
        // else
        // {
        //     command_ros_topic_ = "cmd_vel";
        //     RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "Parameter Not Found " << command_ros_topic_.c_str());
        // }

        // if(_sdf->HasElement("commandIGNTopic"))
        // {
        //     command_ign_topic_ = _sdf->Get<std::string>("commandIGNTopic");
        // }
        // else
        // {
        //     command_ign_topic_ = "cmd_vel_ign";
        // }

        // RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "Parameter: " << command_ros_topic_.c_str());
        // gazebo_ros->getParameter<std::string> ( command_ros_topic_, "commandROSTopic", "cmd_vel_ros" );
        if (!rclcpp::ok())
        {

            RCLCPP_FATAL(rclcpp::get_logger("rclcpp"), "A ROS node for Gazebo has not been initialized, unable to load plugin.");
            return;
        }
        RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "Hello EVERYONE FROM ROS2!");
    }

    void GazeboRosTrackedVehicleInterface::cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr &msg)
    {
        cmd_vel.mutable_linear()->set_x(msg->linear.x);
        cmd_vel.mutable_angular()->set_z(msg->angular.z);
                RCLCPP_INFO_STREAM(rclcpp::get_logger("tracked_interface"), "Inside");

        this->cmd_vel_publisher_ign->Publish(cmd_vel);
    }

    void GazeboRosTrackedVehicleInterface::OnUpdate(const gazebo::common::UpdateInfo &info)
    {
        current_time_gazebo = m_parent->GetWorld()->SimTime();
        seconds_since_last_update = (current_time_gazebo - last_update_time).Double();
        if(seconds_since_last_update > update_period)
        {
            if(this->publish_tf_)
            {
                publishOdom(seconds_since_last_update);
            }
            last_update_time+= common::Time(update_period);
        }
    }

    void GazeboRosTrackedVehicleInterface::OnTrackVelMsg(ConstVector2dPtr &msg)
    {
        track_speed[LEFT] = msg->x();
        track_speed[RIGHT] = msg->y();
        RCLCPP_INFO_STREAM(rclcpp::get_logger("tracked_interface"), "TRACK_SPEED: " << msg->y());
    }

    void GazeboRosTrackedVehicleInterface::publishOdom(double time_step)
    {
        current_time_ros = ros_node->now();
        RCLCPP_INFO_STREAM(rclcpp::get_logger("tracked_interface"), "Time is: " << current_time_ros.seconds());
    }

}
