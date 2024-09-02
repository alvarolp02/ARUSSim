#include "rclcpp/rclcpp.hpp"

#include "custom_msgs/msg/state.hpp"


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

    void timer_callback();
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<custom_msgs::msg::State>::SharedPtr publisher_;
};