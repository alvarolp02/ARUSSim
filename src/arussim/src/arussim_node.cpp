#include "arussim/arussim_node.hpp"

Simulator::Simulator() : Node("simulator")
{
    publisher_ = this->create_publisher<custom_msgs::msg::State>("/arussim/state", 10);
    timer_ = this->create_wall_timer(std::chrono::milliseconds(500), std::bind(&Simulator::timer_callback, this));
}

void Simulator::timer_callback()
{
    auto message = custom_msgs::msg::State();
    message.x = x_;
    message.y = y_;
    message.yaw = yaw_;
    message.vx = vx_;
    message.vy = vy_;
    message.r = r_;
    publisher_->publish(message);
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Simulator>());
  rclcpp::shutdown();
  return 0;
}