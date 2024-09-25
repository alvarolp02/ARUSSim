#include "arussim/arussim_node.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <random>


std::string track = "FSG.pcd";
double fc = 0.5;
double state_update_rate = 100;
double mass = 200;
double L = 1.5;
double fov = 20;
double sensor_rate = 10;
double noise = 0.01;
double min_perception_x = -1;


Simulator::Simulator() : Node("simulator")
{   
    this->declare_parameter<std::string>("track", track);
    this->declare_parameter<double>("friction_coef", fc);
    this->declare_parameter<double>("state_update_rate", state_update_rate);
    this->declare_parameter<double>("mass", mass);
    this->declare_parameter<double>("wheel_base", L);
    this->declare_parameter<double>("sensor.fov_radius", fov);
    this->declare_parameter<double>("sensor.pub_rate", sensor_rate);
    this->declare_parameter<double>("sensor.noise_sigma", noise);
    this->declare_parameter<double>("sensor.cut_cones_below_x", min_perception_x);

    this->get_parameter("track", track);
    this->get_parameter("friction_coef", fc);
    this->get_parameter("state_update_rate", state_update_rate);
    this->get_parameter("mass", mass);
    this->get_parameter("wheel_base", L);
    this->get_parameter("sensor.fov_radius", fov);
    this->get_parameter("sensor.pub_rate", sensor_rate);
    this->get_parameter("sensor.noise_sigma", noise);
    this->get_parameter("sensor.cut_cones_below_x", min_perception_x);


    clock_ = std::make_shared<rclcpp::Clock>(RCL_SYSTEM_TIME);
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    state_pub_ = this->create_publisher<custom_msgs::msg::State>("/arussim/state", 10);
    track_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/arussim/track", 10);
    perception_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/arussim/perception", 10);
    marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/arussim/vehicle_visualization", 1);
    slow_timer_ = this->create_wall_timer(std::chrono::milliseconds((int)(1000/sensor_rate)), std::bind(&Simulator::onSlowTimer, this));
    fast_timer_ = this->create_wall_timer(std::chrono::milliseconds((int)(1000/state_update_rate)), std::bind(&Simulator::onFastTimer, this));
    cmd_sub_ = this->create_subscription<custom_msgs::msg::Cmd>("/arussim/cmd", 1, std::bind(&Simulator::onCmd, this, std::placeholders::_1));
    rviz_telep_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>("/initialpose", 1, std::bind(&Simulator::onRvizTelep, this, std::placeholders::_1));


    // Load the car mesh
    marker_.header.frame_id = "arussim/vehicle_cog";
    marker_.header.stamp = clock_->now();
    marker_.ns = "arussim";
    marker_.id = 0;
    marker_.type = visualization_msgs::msg::Marker::MESH_RESOURCE;
    marker_.mesh_resource = "package://arussim/resources/meshes/whole_car.stl";
    marker_.action = visualization_msgs::msg::Marker::ADD;
    marker_.pose.position.x = 1.0;
    marker_.pose.position.y = 0;
    marker_.pose.position.z = 0;
    marker_.scale.x = 0.001;
    marker_.scale.y = 0.001;
    marker_.scale.z = 0.001;
    marker_.color.r = 60.0/255.0;
    marker_.color.g = 55.0/255.0;
    marker_.color.b = 70.0/255.0;
    marker_.color.a = 1.0;
    marker_.lifetime = rclcpp::Duration::from_seconds(0.0);

    // Load the track pointcloud
    std::string package_path = ament_index_cpp::get_package_share_directory("arussim");
    std::string filename = package_path+"/resources/tracks/"+track;
    if (pcl::io::loadPCDFile<ConeXYZColorScore>(filename, track_) == -1)
    {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Couldn't read file %s", filename.c_str());
        return;
    }
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Loaded %ld data points from %s", track_.points.size(), filename.c_str());

}

void Simulator::onSlowTimer()
{   
    // Update track
    sensor_msgs::msg::PointCloud2 track_msg;
    pcl::toROSMsg(track_, track_msg);
    track_msg.header.stamp = clock_->now();
    track_msg.header.frame_id="arussim/world";
    track_pub_->publish(track_msg);

    // Random noise generation
    std::random_device rd; 
    std::mt19937 gen(rd());
    std::normal_distribution<> dist(0.0, noise);

    auto perception_cloud = pcl::PointCloud<ConeXYZColorScore>();
    for (auto &point : track_.points)
    {
        double d = std::sqrt(std::pow(point.x - x_, 2) + std::pow(point.y - y_, 2));
        if (d < fov)
        {
            ConeXYZColorScore p;
            p.x = (point.x - x_)*std::cos(yaw_) + (point.y - y_)*std::sin(yaw_) + dist(gen);
            p.y = -(point.x - x_)*std::sin(yaw_) + (point.y - y_)*std::cos(yaw_) + dist(gen);
            p.z = 0.0;
            p.color = point.color;
            p.score = 1.0;
            if (p.x > min_perception_x) {
                perception_cloud.push_back(p);
            }
        }
    }

    sensor_msgs::msg::PointCloud2 perception_msg;
    pcl::toROSMsg(perception_cloud, perception_msg);
    perception_msg.header.stamp = clock_->now();
    perception_msg.header.frame_id="arussim/vehicle_cog";
    perception_pub_->publish(perception_msg);
}

void Simulator::onFastTimer()
{   
    // Update state and broadcast transform
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
    
    // Update vehicle marker
    marker_.header.stamp = clock_->now();
    marker_pub_->publish(marker_);
}

void Simulator::onCmd(const custom_msgs::msg::Cmd::SharedPtr msg)
{
    input_acc_ = msg->acc;
    input_delta_ = msg->delta;
    time_last_cmd_ = clock_->now();
}

void Simulator::onRvizTelep(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
{
    tf2::Quaternion q(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
    tf2::Matrix3x3 mat(q);
    double roll, pitch, yaw;
    mat.getRPY(roll, pitch, yaw);

    x_ = msg->pose.pose.position.x;
    y_ = msg->pose.pose.position.y;
    yaw_ = yaw;
    vx_ = 0;
    vy_ = 0;
    r_ = 0;
}

void Simulator::updateState()
{
    rclcpp::Time current_time = clock_->now();
    if((current_time - time_last_cmd_).seconds() > 0.2 && vx_ != 0)
    {
        input_acc_ = vx_ > 0 ? -fc*9.8 : fc*9.8;
    }

    double dt = 0.01;

    // Model equations
    double x_dot = vx_ * std::cos(yaw_);
    double y_dot = vx_ * std::sin(yaw_);
    double yaw_dot = vx_ / L * std::tan(input_delta_);
    
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