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


class Simulator : public rclcpp::Node
{
  public:
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

    void onSlowTimer();
    void onFastTimer();
    void onCmd(const custom_msgs::msg::Cmd::SharedPtr msg);
    void onRvizTelep(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);
    void updateState();
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