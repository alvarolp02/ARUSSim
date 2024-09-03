#include "arussim/arussim_node.hpp"

const float L = 1.5; // Distance between axles
const float m = 270.0; //Mass of the vehicle
const float fc = 0.5; //Coefficient of friction

Simulator::Simulator() : Node("simulator")
{   
    clock_ = std::make_shared<rclcpp::Clock>(RCL_SYSTEM_TIME);
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    state_pub_ = this->create_publisher<custom_msgs::msg::State>("/arussim/state", 10);
    perception_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/arussim/perception", 10);
    marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/arussim/vehicle_visualization", 10);
    timer_ = this->create_wall_timer(std::chrono::milliseconds(10), std::bind(&Simulator::onTimer, this));
    subscription_ = this->create_subscription<custom_msgs::msg::Cmd>("/arussim/cmd", 1, std::bind(&Simulator::onCmd, this, std::placeholders::_1));
    
    marker_.header.frame_id = "arussim/vehicle_cog";
    marker_.header.stamp = clock_->now();
    marker_.ns = "arussim";
    marker_.id = 0;
    marker_.type = visualization_msgs::msg::Marker::MESH_RESOURCE;
    marker_.mesh_resource = "package://arussim/meshes/whole_car.stl";
    marker_.action = visualization_msgs::msg::Marker::ADD;
    marker_.pose.position.x = 1.0;
    marker_.pose.position.y = 0;
    marker_.pose.position.z = 0;
    marker_.pose.orientation.x = 0;
    marker_.pose.orientation.y = 0;
    marker_.pose.orientation.z = 0;
    marker_.pose.orientation.w = 1;
    marker_.scale.x = 0.001;
    marker_.scale.y = 0.001;
    marker_.scale.z = 0.001;
    marker_.color.r = 60.0/255.0;
    marker_.color.g = 55.0/255.0;
    marker_.color.b = 70.0/255.0;
    marker_.color.a = 1.0;
}

void Simulator::onTimer()
{   
    updateState();
    
    auto message = custom_msgs::msg::State();
    message.x = x_;
    message.y = y_;
    message.yaw = yaw_;
    message.vx = vx_;
    message.vy = vy_;
    message.r = r_;
    state_pub_->publish(message);

    broadcast_transform();
    marker_.header.stamp = clock_->now();
    marker_pub_->publish(marker_);


    auto map_cloud = std::make_shared<pcl::PointCloud<PointXYZColorScore>>();
    for (int i = 0; i < 10; i++)
    {
        PointXYZColorScore point;
        point.x = 3*i;
        point.y = 1.5;
        point.z = 0;
        point.color = 0;
        point.score = 1.0;
        map_cloud->push_back(point);
    }
    for (int i = 0; i < 10; i++)
    {
        PointXYZColorScore point;
        point.x = 3*i;
        point.y = -1.5;
        point.z = 0;
        point.color = 1;
        point.score = 1.0;
        map_cloud->push_back(point);
    }

    sensor_msgs::msg::PointCloud2 map_msg;
    pcl::toROSMsg(*map_cloud,map_msg);
    map_msg.header.stamp = clock_->now();
    map_msg.header.frame_id="arussim/world";
    perception_pub_->publish(map_msg);

    pcl::io::savePCDFileASCII("pointcloud.pcd", *map_cloud);
}

void Simulator::onCmd(const custom_msgs::msg::Cmd::SharedPtr msg)
{
    input_acc_ = msg->acc;
    input_delta_ = msg->delta;
    time_last_cmd_ = clock_->now();
}

void Simulator::updateState()
{
    rclcpp::Time current_time = clock_->now();
    if((current_time - time_last_cmd_).seconds() > 0.2 && vx_ != 0)
    {
        input_acc_ = vx_ > 0 ? -fc*9.8 : fc*9.8;
    }

    float dt = 0.01;

    // Model equations
    float x_dot = vx_ * std::cos(yaw_);
    float y_dot = vx_ * std::sin(yaw_);
    float yaw_dot = vx_ / L * std::tan(input_delta_);
    
    // Update state
    x_ += x_dot * dt;
    y_ += y_dot * dt;
    yaw_ += yaw_dot * dt;
    vx_ += input_acc_ * dt;
}

void Simulator::broadcast_transform()
  {
    // Create a TransformStamped message
    geometry_msgs::msg::TransformStamped transform_stamped;

    // Set the frame ID and child frame ID
    transform_stamped.header.stamp = clock_->now();
    transform_stamped.header.frame_id = "arussim/world";
    transform_stamped.child_frame_id = "arussim/vehicle_cog";

    // Set the translation (x, y, z)
    transform_stamped.transform.translation.x = x_;
    transform_stamped.transform.translation.y = y_;
    transform_stamped.transform.translation.z = 0.0;

    // Set the rotation (using a quaternion)
    tf2::Quaternion q;
    q.setRPY(0, 0, yaw_); // Roll, Pitch, Yaw in radians
    transform_stamped.transform.rotation.x = q.x();
    transform_stamped.transform.rotation.y = q.y();
    transform_stamped.transform.rotation.z = q.z();
    transform_stamped.transform.rotation.w = q.w();

    // Broadcast the transform
    tf_broadcaster_->sendTransform(transform_stamped);
  }

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Simulator>());
  rclcpp::shutdown();
  return 0;
}