#ifndef gazebo_ros_tracked_vehicle_interface
#define gazebo_ros_tracked_vehicle_interface

//ROS 2 Includes
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

//Gazebo Includes
#include <gazebo/gazebo.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/World.hh> // Include the header for World class
#include <gazebo/transport/transport.hh> // Include for transport
#include <gazebo_ros/node.hpp>  // This includes the Gazebo ROS Node class

#include <gazebo/common/common.hh>
#include <gazebo/physics/Model.hh>
#include "gazebo/msgs/MessageTypes.hh"
#include "gazebo/msgs/twist.pb.h"


#include <boost/thread.hpp>
#include <boost/bind.hpp>

#include <ignition/math/Vector3.hh>


namespace gazebo
{
    class GazeboRosTrackedVehicleInterface : public ModelPlugin
    {

        enum OdomSource
        {
            ENCODER = 0,
            WORLD = 1,
        };


        public:
            GazeboRosTrackedVehicleInterface();
            // ~GazeboRosTrackedVehicleInterface();
            void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);

            template<typename Type>
            Type params(const sdf::ElementPtr &_sdf, const std::string &paramName, const Type &defaultValue);
            
            template<typename Type>
            Type params(const sdf::ElementPtr &_sdf, const std::string &paramName, const std::map<std::string, Type> &options, const Type &defaultValue);

            // std::string params(const sdf::ElementPtr &_sdf, const std::string &topicName, const std::string &defaultName);
            // bool params(const sdf::ElementPtr &_sdf, const std::string &topicName, const bool &defaultName);
            // double params(const sdf::ElementPtr &_sdf, const std::string &topicName, const double &defaultName);

            // void Reset();
        private:
            gazebo_ros::Node::SharedPtr ros_node;
            rclcpp::Node::SharedPtr node;
            transport::NodePtr m_ignition_node;
            physics::ModelPtr m_parent;
            physics::WorldPtr m_world;

            // global variables for storing parameters
            std::string robot_namespace_;
            std::string command_ros_topic_;
            std::string command_ign_topic_;
            std::string odometry_topic_;
            std::string odometry_frame_;
            std::string track_speed_topic_;
            std::string robot_base_frame_;

            std::string topicName;


            double track_separation_;

            // Flags
            bool publish_tf_;
            bool publishOdomTF_;
            bool alive_;

            //ROS 2 Publisher
            rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom;
            rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_ros;
            
            std::shared_ptr<tf2_ros::TransformBroadcaster> transformBroadcaster;


            
            // ignition transport
            transport::PublisherPtr cmd_vel_publisher_ign;
            transport::SubscriberPtr tracks_vel_subscriber_ign;
            event::ConnectionPtr update_connection;
            double update_rate;
            double update_period;
            //Maybe use chrono????
            common::Time last_update_time;

            OdomSource odom_source;

            // void onTrackVelMSg (ConstVector2dPtr &msg);
            // void cmdVellCallback(const geometry_msgs::msg::TwistStamped &msg);

            geometry_msgs::msg::Pose pose_encoder_;
            msgs::Twist cmd_vel;
            double track_speed[2];

    };
    GZ_REGISTER_MODEL_PLUGIN(GazeboRosTrackedVehicleInterface);

}
#endif // WORLD_PLUGIN_TUTORIAL_H