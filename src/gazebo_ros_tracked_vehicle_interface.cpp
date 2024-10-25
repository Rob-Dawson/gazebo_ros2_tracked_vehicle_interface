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

        odom_source = params(_sdf, "odometrySource", odomOptions, ENCODER);
        RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "Odom Source " << odom_source);

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

            odom_publisher = ros_node->create_publisher<nav_msgs::msg::Odometry>(odometry_topic_, 10);
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
        if (odom_source == ENCODER)
        {
            // RCLCPP_FATAL(rclcpp::get_logger("rcl1cpp"), "Encoder");

            UpdateOdometryEncoder();
        }

        common::Time current_time = m_parent->GetWorld()->SimTime();
        seconds_since_last_update = (current_time - last_update_time).Double();
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

    void GazeboRosTrackedVehicleInterface::UpdateOdometryEncoder()
    {
        double vl = track_speed[LEFT];
        double vr = track_speed[RIGHT];

        common::Time current_time = m_parent->GetWorld()->SimTime();
        double seconds_since_last_update = (current_time - last_odom_update_).Double();
        last_odom_update_ = current_time;
        // RCLCPP_INFO_STREAM(rclcpp::get_logger("track_interface"), seconds_since_last_update);
        // RCLCPP_INFO_STREAM(rclcpp::get_logger("track_interface"), current_time);

        double b = track_separation_;
        //SL = Displacement Left => Distance travelled Left
        //SR = Displacement Right
        //SSUM = Total Displacement = Total distance travelled
        //SDIFF = Displacement Difference = How much the robot has rotated
        //dx = Change in X position
        //dy = Change in Y position
        //dtheta = Change in angular rotation
        double sl = vl*seconds_since_last_update;
        double sr = vr * seconds_since_last_update;
        double ssum= sl+sr;
        double sdiff = sr-sl;
        double dx = (ssum) / 2.0 * cos(pose_encoder_.theta + (sdiff)/(2.0*b));
        double dy = (ssum) / 2.0 * sin (pose_encoder_.theta + (sdiff)/(2.0*b));
        double dtheta = sdiff/b;

        //Updating robots position
        pose_encoder_.x += dx;
        pose_encoder_.y += dy;
        pose_encoder_.theta += dtheta;

        double w = dtheta/seconds_since_last_update;
        double v = sqrt(dx*dx+dy*dy)/seconds_since_last_update;
        tf2::Quaternion qt;
        tf2::Vector3 vt;
        qt.setRPY(0,0,pose_encoder_.theta);
        vt = tf2::Vector3(pose_encoder_.x, pose_encoder_.y,0);

        odom.pose.pose.position.x = vt.x();
        odom.pose.pose.position.y = vt.y();
        odom.pose.pose.position.z = vt.z();

        odom.pose.pose.orientation.x = qt.x();
        odom.pose.pose.orientation.y = qt.y();
        odom.pose.pose.orientation.z = qt.z();
        odom.pose.pose.orientation.w = qt.w();

        odom.twist.twist.linear.x=v;
        odom.twist.twist.linear.y=0;
        odom.twist.twist.angular.z=w;


    }

    void GazeboRosTrackedVehicleInterface::publishOdom(double time_step)
    {
        current_time_ros = ros_node->now();
        // RCLCPP_INFO_STREAM(rclcpp::get_logger("tracked_interface"), "Time is: " << current_time_ros.seconds());

        tf2::Quaternion qt;
        tf2::Vector3 vt;

        std::string odom_frame = "tracked_robot_2/"+odometry_frame_;
        std::string base_footprint_frame = "tracked_robot_2/"+robot_base_frame_;

        if (odom_source == ENCODER)
        {
            // RCLCPP_INFO_STREAM(rclcpp::get_logger("tracked_interface"), "Encoder: " );
            qt = tf2::Quaternion(odom.pose.pose.orientation.x, odom.pose.pose.orientation.y, odom.pose.pose.orientation.z, odom.pose.pose.orientation.w);
            vt = tf2::Vector3(odom.pose.pose.position.x, odom.pose.pose.position.y, odom.pose.pose.position.z);
        }
        if(odom_source == WORLD)
        {
        ignition::math::Pose3d pose = m_parent->WorldPose();
        // RCLCPP_INFO_STREAM(rclcpp::get_logger("tracked_interface"), pose);

        qt = tf2::Quaternion(pose.Rot().X(), pose.Rot().Y(), pose.Rot().Z(), pose.Rot().W());
        vt = tf2::Vector3(pose.Pos().X(), pose.Pos().Y(), pose.Pos().Z());

        odom.pose.pose.position.x = vt.x();
        odom.pose.pose.position.y = vt.y();
        odom.pose.pose.position.z = vt.z();

        odom.pose.pose.orientation.x = qt.x();
        odom.pose.pose.orientation.y = qt.y();
        odom.pose.pose.orientation.z = qt.z();
        odom.pose.pose.orientation.w = qt.w();

        ignition::math::Vector3d linear;
        linear = m_parent->WorldLinearVel();
        odom.twist.twist.angular.z = m_parent->WorldAngularVel().Z();

        float yaw = pose.Rot().Yaw();
        odom.twist.twist.linear.x = cosf(yaw) * linear.X() + sinf(yaw) * linear.X();
        odom.twist.twist.linear.y = cosf(yaw) * linear.Y() + sinf(yaw) * linear.Y();
        }

        if(publishOdomTF_)
        {
            geometry_msgs::msg::TransformStamped transform_stamped;
            transform_stamped.header.stamp = current_time_ros;
            transform_stamped.header.frame_id = odom_frame;
            transform_stamped.child_frame_id = base_footprint_frame;

            transform_stamped.transform.translation.x = vt.getX();
            transform_stamped.transform.translation.y = vt.getY();
            transform_stamped.transform.translation.z = vt.getZ();

            transform_stamped.transform.rotation.x = qt.x();
            transform_stamped.transform.rotation.y = qt.y();
            transform_stamped.transform.rotation.z = qt.z();
            transform_stamped.transform.rotation.z = qt.w();

            transformBroadcaster->sendTransform(transform_stamped);

        }


        odom.pose.covariance[0] = 0.00001;
        odom.pose.covariance[7] = 0.00001;
        odom.pose.covariance[14] = 1000000000000.0;
        odom.pose.covariance[21] = 1000000000000.0;
        odom.pose.covariance[28] = 1000000000000.0;
        odom.pose.covariance[35] = 0.001;

        odom.header.stamp = current_time_ros;
        odom.header.frame_id = odom_frame;
        odom.child_frame_id = base_footprint_frame;
        odom_publisher->publish(odom);
    }

}
