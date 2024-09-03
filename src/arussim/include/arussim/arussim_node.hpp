#include "rclcpp/rclcpp.hpp"

#include "custom_msgs/msg/state.hpp"
#include "custom_msgs/msg/cmd.hpp"

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2/LinearMath/Quaternion.h"


class Simulator : public rclcpp::Node
{
  public:
    Simulator();

  private:
    float x_ = 0;
    float y_ = 0;
    float yaw_ = 0;
    float vx_ = 0;
    float vy_ = 0;
    float r_ = 0;

    rclcpp::Clock::SharedPtr clock_;
    rclcpp::Time time_last_cmd_;
    float input_acc_;
    float input_delta_;

    visualization_msgs::msg::Marker marker_;

    void onTimer();
    void onCmd(const custom_msgs::msg::Cmd::SharedPtr msg);
    void updateState();
    void broadcast_transform();

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<custom_msgs::msg::Cmd>::SharedPtr subscription_;
    rclcpp::Publisher<custom_msgs::msg::State>::SharedPtr state_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};