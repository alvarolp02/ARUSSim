/**
 * @file arussim_node.hpp
 * @brief Header file for the ARUS Team's simulator node (ARUSim).
 * 
 * This file declares the Simulator class, which simulates the behavior of a vehicle 
 * in a racing environment, including sensor data generation, state updates, and 
 * visualization in RViz.
 */

#include <rclcpp/rclcpp.hpp>

#include "custom_msgs/msg/state.hpp"
#include "custom_msgs/msg/cmd.hpp"

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h> 

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl/io/pcd_io.h>
#include <iostream>
#include "ConeXYZColorScore.h"

#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>

/**
 * @class Simulator
 * @brief The Simulator class is responsible for simulating the vehicle dynamics, 
 * sensor data, and managing communication with ROS nodes.
 * 
 * The Simulator class handles the simulation of a vehicle's position, orientation, 
 * and velocity, along with generating and broadcasting sensor data. It also publishes 
 * visualization data to RViz and listens for control commands.
 */

class Simulator : public rclcpp::Node
{
  public:
  /**
     * @brief Constructor for the Simulator class.
     * 
     * This initializes the Simulator node, declares ROS parameters, sets up publishers, 
     * subscribers, and timers, and loads necessary resources like the track and vehicle mesh.
     */
    Simulator();

  private:
    double x_ = 0;
    double y_ = 0;
    double yaw_ = 0;
    double vx_ = 0;
    double vy_ = 0;
    double r_ = 0;

    std::string kTrackName;
    double kFrictionCoef;
    double kStateUpdateRate;
    double kMass;
    double kWheelBase;
    double kFOV;
    double kSensorRate;
    double kNoise;
    double kMinPerceptionX;

    rclcpp::Clock::SharedPtr clock_;
    rclcpp::Time time_last_cmd_;
    double input_acc_;
    double input_delta_;

    visualization_msgs::msg::Marker marker_;
    pcl::PointCloud<ConeXYZColorScore> track_;

    /**
     * @brief Callback function for the slow timer.
     * 
     * This method is called at regular intervals to update and publish sensor data, 
     * such as the track and perception point clouds.
     */
    void on_slow_timer();

    /**
     * @brief Callback function for the fast timer.
     * 
     * This method is called at regular intervals to update the vehicle's state and 
     * broadcast its transform to the ROS TF system.
     */
    void on_fast_timer();

    /**
     * @brief Callback for receiving control commands.
     * 
     * This method processes incoming control commands (acceleration and steering angle) 
     * to update the vehicle's dynamics.
     * 
     * @param msg The control command message.
     */
    void cmd_callback(const custom_msgs::msg::Cmd::SharedPtr msg);
    
    /**
     * @brief Callback for receiving teleportation commands from RViz.
     * 
     * This method teleports the vehicle to a new pose as commanded via RViz.
     * 
     * @param msg The pose message received from RViz.
     */
    void rviz_telep_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);
    
    /**
     * @brief Updates the vehicle's state based on the current input and model equations.
     * 
     * This method computes the vehicle's new position, orientation, and velocity using 
     * basic kinematic equations and the received control inputs.
     */
    void update_state();

    /**
     * @brief Broadcasts the vehicle's current pose to the ROS TF system.
     * 
     * This method sends the vehicle's transform to the TF tree so that other nodes 
     * can track its position and orientation.
     */
    void broadcast_transform();


    rclcpp::TimerBase::SharedPtr slow_timer_;
    rclcpp::TimerBase::SharedPtr fast_timer_;
    rclcpp::Subscription<custom_msgs::msg::Cmd>::SharedPtr cmd_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr rviz_telep_sub_;
    rclcpp::Publisher<custom_msgs::msg::State>::SharedPtr state_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr track_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr perception_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};