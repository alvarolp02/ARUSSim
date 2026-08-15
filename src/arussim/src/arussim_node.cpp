/**
 * @file arussim_node.cpp
 * @brief Simulator node for the ARUS Team (ARUSsim)
 */

#include "arussim/arussim_node.hpp"
#include <cstdlib>
#include <csignal>
#include <filesystem>
#include <sys/wait.h>
#include <unistd.h>

/**
 * @class Simulator
 * @brief Simulator class for the ARUS Team (ARUSsim).
 *
 * This class simulates the behavior of a vehicle in a racing environment,
 * handling the simulation of state updates, sensor data generation, and broadcasting
 * transforms to visualize the vehicle and track in RViz.
 */
Simulator::Simulator() : Node("simulator")
{
    this->declare_parameter<std::string>("track", "FSG");
    this->declare_parameter<std::string>("simulation_car", "ART25D_2WD");
    this->declare_parameter<double>("state_update_rate", 1000);
    this->declare_parameter<double>("controller_rate", 100);
    this->declare_parameter<std::string>("simulation_mode", "default");
    this->declare_parameter<bool>("use_gss", false);
    this->declare_parameter<double>("vehicle.COG_front_dist", 1.9);
    this->declare_parameter<double>("vehicle.COG_back_dist", -1.0);
    this->declare_parameter<double>("vehicle.car_width", 0.8);
    this->declare_parameter<double>("cone.height", 0.325);
    this->declare_parameter<double>("cone.width", 0.228);
    this->declare_parameter<double>("sensor.lidar_fov", 120.0);
    this->declare_parameter<double>("sensor.lidar_height", 0.64);
    this->declare_parameter<double>("sensor.lidar_min_dist", 30.0);
    this->declare_parameter<double>("sensor.lidar_max_dist", 3.0);
    this->declare_parameter<double>("sensor.lidar_mu_time", 2.3156);
    this->declare_parameter<double>("sensor.lidar_sigma_time", 0.3460);
    this->declare_parameter<double>("sensor.camera_fov", 10);
    this->declare_parameter<double>("sensor.pub_rate", 10);
    this->declare_parameter<double>("sensor.noise_position_lidar_perception", 0.01);
    this->declare_parameter<double>("sensor.noise_prob_lidar_perception", 0.2);
    this->declare_parameter<double>("sensor.noise_lidar_color", 0.01);
    this->declare_parameter<double>("sensor.noise_camera_perception", 0.01);
    this->declare_parameter<double>("sensor.noise_camera_color", 0.01);
    this->declare_parameter<double>("sensor.cut_cones_below_x", -1);
    this->declare_parameter<double>("sensor.position_lidar_x", 1.5);
    this->declare_parameter<double>("sensor.position_camera_x", 0.0);
    this->declare_parameter<double>("sensor.camera_color_probability", 0.85);
    this->declare_parameter<double>("sensor.perception_delay_mu", 2.3156);
    this->declare_parameter<double>("sensor.perception_delay_sigma", 0.3460);
    this->declare_parameter<bool>("csv_state", false);
    this->declare_parameter<bool>("csv_vehicle_dynamics", false);
    this->declare_parameter<bool>("debug", false);

    this->get_parameter("track", kTrackName);
    this->get_parameter("simulation_car", kSimulationCar);
    this->get_parameter("state_update_rate", kStateUpdateRate);
    this->get_parameter("controller_rate", kControllerRate);
    this->get_parameter("simulation_mode", kSimulationMode);
    this->get_parameter("use_gss", kUseGSS);
    this->get_parameter("vehicle.COG_front_dist", kCOGFrontDist);
    this->get_parameter("vehicle.COG_back_dist", kCOGBackDist);
    this->get_parameter("vehicle.car_width", kCarWidth);
    this->get_parameter("cone.height", kConeHeight);
    this->get_parameter("cone.width", kConeWidth);
    this->get_parameter("sensor.lidar_fov", kLidarFOV);
    this->get_parameter("sensor.lidar_height", kLidarHeight);
    this->get_parameter("sensor.lidar_min_dist", kMinLidarDistance);
    this->get_parameter("sensor.lidar_max_dist", kMaxLidarDistance);
    this->get_parameter("sensor.lidar_mu_time", kLidarMuTime);
    this->get_parameter("sensor.lidar_sigma_time", kLidarSigmaTime);
    this->get_parameter("sensor.camera_fov", kCameraFOV);
    this->get_parameter("sensor.pub_rate", kSensorRate);
    this->get_parameter("sensor.noise_position_lidar_perception", kNoiseLidarPosPerception);
    this->get_parameter("sensor.noise_prob_lidar_perception", kNoiseLidarProbPerception);
    this->get_parameter("sensor.noise_lidar_color", kNoiseLidarColor);
    this->get_parameter("sensor.noise_camera_perception", kNoiseCameraPerception);
    this->get_parameter("sensor.noise_camera_color", kNoiseCameraColor);
    this->get_parameter("sensor.cut_cones_below_x", kMinPerceptionX);
    this->get_parameter("sensor.position_lidar_x", kPosLidarX);
    this->get_parameter("sensor.position_camera_x", kPosCameraX);
    this->get_parameter("sensor.camera_color_probability", kCameraColorProbability);
    this->get_parameter("sensor.perception_delay_mu", kDelayMu);
    this->get_parameter("sensor.perception_delay_sigma", kDelaySigma);
    this->get_parameter("csv_state", kCSVState);
    this->get_parameter("csv_vehicle_dynamics", kCSVVehicleDynamics);
    this->get_parameter("debug", kDebug);

    clock_ = std::make_shared<rclcpp::Clock>(RCL_SYSTEM_TIME);
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    state_pub_ = this->create_publisher<arussim_msgs::msg::State>(
        "/arussim/state", 10);
    ground_truth_pub_ = this->create_publisher<common_msgs::msg::State>(
        "/arussim/ground_truth", 10);
    control_vx_pub_ = this->create_publisher<std_msgs::msg::Float32>(
        "/arussim/control_vx", 10);
    control_vy_pub_ = this->create_publisher<std_msgs::msg::Float32>(
        "/arussim/control_vy", 10);
    control_r_pub_ = this->create_publisher<std_msgs::msg::Float32>(
        "/arussim/control_r", 10);
    track_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        "/arussim/track", 10);
    lidar_perception_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        "/arussim/perception", 10);
    marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(
        "/arussim/vehicle_visualization", 1);
    cone_marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
        "/arussim/cone_visualization", 1);
    lap_signal_pub_ = this->create_publisher<std_msgs::msg::Bool>(
        "/arussim/tpl_signal", 10);
    hit_cones_pub_ = this->create_publisher<arussim_msgs::msg::PointXY>(
        "/arussim/hit_cones", 10);
    fixed_trajectory_pub_ = this->create_publisher<arussim_msgs::msg::Trajectory>(
        "/arussim/fixed_trajectory", 10);
    camera_perception_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        "/arussim/camera_perception", 10);
    slip_ratio_pub_ = this->create_publisher<arussim_msgs::msg::FourWheelDrive>(
        "/arussim/slip_ratio", 10);
    slip_angle_pub_ = this->create_publisher<arussim_msgs::msg::FourWheelDrive>(
        "/arussim/slip_angle", 10);
    tire_load_pub_ = this->create_publisher<arussim_msgs::msg::FourWheelDrive>(
        "/arussim/tire_load", 10);

    tv_out_pub_ = this->create_publisher<arussim_msgs::msg::FourWheelDrive>(
        "/arussim/tv_out", 10);
    tc_out_pub_ = this->create_publisher<arussim_msgs::msg::FourWheelDrive>(
        "/arussim/tc_out", 10);
    pl_out_pub_ = this->create_publisher<arussim_msgs::msg::FourWheelDrive>(
        "/arussim/pl_out", 10);
    torque_cmd_pub_ = this->create_publisher<arussim_msgs::msg::FourWheelDrive>(
        "/arussim/torque_cmd", 10);
    sr_t_pub_ = this->create_publisher<arussim_msgs::msg::FourWheelDrive>(
        "/arussim/sr_t", 10);
    sr_tc_pub_ = this->create_publisher<arussim_msgs::msg::FourWheelDrive>(
        "/arussim/sr_tc", 10);
    ff_torque_pub_ = this->create_publisher<arussim_msgs::msg::FourWheelDrive>(
        "/arussim/ff_torque", 10);

    slow_timer_ = this->create_wall_timer(
        std::chrono::milliseconds((int)(1000 / kSensorRate)),
        std::bind(&Simulator::on_slow_timer, this));
    fast_timer_ = this->create_wall_timer(
        std::chrono::milliseconds((int)(1000 / kStateUpdateRate)),
        std::bind(&Simulator::on_fast_timer, this));

    perception_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(1),
        std::bind(&Simulator::process_perception_queue, this));

    circuit_sub_ = this->create_subscription<std_msgs::msg::String>("/arussim/circuit", 1, 
        std::bind(&Simulator::load_track, this, std::placeholders::_1));
    rviz_telep_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
        "/initialpose", 1, std::bind(&Simulator::rviz_telep_callback, this, std::placeholders::_1));
    ebs_sub_ = this->create_subscription<std_msgs::msg::Bool>(
        "/arussim_interface/EBS", 1, std::bind(&Simulator::ebs_callback, this, std::placeholders::_1));

    reset_sub_ = this->create_subscription<std_msgs::msg::Bool>("/arussim/reset", 1, 
        std::bind(&Simulator::reset_callback, this, std::placeholders::_1));

    noisy_ax_sub_ = this->create_subscription<std_msgs::msg::Float32>(
        "/arussim/IMU/ax", 10, std::bind(&Simulator::noisy_ax_callback, this, std::placeholders::_1));
    noisy_ay_sub_ = this->create_subscription<std_msgs::msg::Float32>(
        "/arussim/IMU/ay", 10, std::bind(&Simulator::noisy_ay_callback, this, std::placeholders::_1));
    noisy_r_sub_ = this->create_subscription<std_msgs::msg::Float32>(
        "/arussim/IMU/yaw_rate", 10, std::bind(&Simulator::noisy_r_callback, this, std::placeholders::_1));

    noisy_ms_sub_ = this->create_subscription<arussim_msgs::msg::FourWheelDrive>(
        "/arussim/motor_speed", 10, std::bind(&Simulator::noisy_ms_callback, this, std::placeholders::_1));
    noisy_torque_sub_ = this->create_subscription<arussim_msgs::msg::FourWheelDrive>(
        "/arussim/torque4WD", 10, std::bind(&Simulator::noisy_torque_callback, this, std::placeholders::_1));

    noisy_vx_sub_ = this->create_subscription<std_msgs::msg::Float32>(
        "/arussim/gss/vx", 10, std::bind(&Simulator::noisy_vx_callback, this, std::placeholders::_1));
    noisy_vy_sub_ = this->create_subscription<std_msgs::msg::Float32>(
        "/arussim/gss/vy", 10, std::bind(&Simulator::noisy_vy_callback, this, std::placeholders::_1));
    noisy_delta_sub_ = this->create_subscription<std_msgs::msg::Float32>(
        "/arussim/extensometer", 10, std::bind(&Simulator::noisy_delta_callback, this, std::placeholders::_1));

    // Load the car mesh
    marker_.header.frame_id = "arussim/vehicle_cog";
    marker_.type = visualization_msgs::msg::Marker::MESH_RESOURCE;
    marker_.mesh_resource = "package://arussim/resources/meshes/ART24D.stl";
    marker_.action = visualization_msgs::msg::Marker::ADD;
    marker_.pose.position.x = 1.0;
    marker_.pose.position.y = 0;
    marker_.pose.position.z = 0;
    marker_.scale.x = 0.001;
    marker_.scale.y = 0.001;
    marker_.scale.z = 0.001;
    marker_.color.r = 60.0 / 255.0;
    marker_.color.g = 55.0 / 255.0;
    marker_.color.b = 70.0 / 255.0;
    marker_.color.a = 1.0;
    marker_.lifetime = rclcpp::Duration::from_seconds(0.0);

    // Thread for CAN reception
    init_can_sockets();

    std::thread thread_0_(&Simulator::receive_can_0, this);
    thread_0_.detach();

    std::thread thread_1_(&Simulator::receive_can_1, this);
    thread_1_.detach();

    std::thread thread_2_(&Simulator::receive_can_2, this);
    thread_2_.detach();

    launch_sub_ = this->create_subscription<std_msgs::msg::Bool>("/arussim/launch", 1,
    std::bind(&Simulator::launch_callback, this, std::placeholders::_1));

    // Lognormal perception delay
    perception_delay_gen = std::mt19937(std::random_device{}());
    perception_delay_dist = std::lognormal_distribution<double>(kLidarMuTime, kLidarSigmaTime);

    // Initialize torque variable 
    torque_cmd_ = {0.0, 0.0, 0.0, 0.0};

    // Initialize sensors struct
    sensors_.gss_ok = 1; // Assume GSS is always OK in simulation
    sensors_.battery_voltage = 470;

    // Initialize DV struct
    dv_.autonomous = 1; // 1 -> DV mode
    dv_.driving = 1;    // 1 -> Car is driving

    try
    {
        this->simulation_car_csv_ = kSimulationCar + ".csv";
        std::string csv_path = this->get_csv_path(this->simulation_car_csv_);
        this->parameters_map_ = this->load_car_parameters(csv_path);
        this->vehicle_dynamics_.set_parameters(this->parameters_map_);
    }
    catch (const std::exception &e)
    {
        RCLCPP_ERROR(this->get_logger(), "Failed loading car parameters: %s", e.what());
    }

    if (kSimulationMode == "default")
    {
        this->load_control_parameters();
        control_init(&sensors_, &car_parameters_, &dv_); // Initialize CON-VehicleControl
        RCLCPP_WARN(this->get_logger(), "SIL control simulation enabled.");
        controller_sim_timer_ = this->create_wall_timer(
            std::chrono::milliseconds((int)(1000 / kControllerRate)),
            std::bind(&Simulator::on_controller_sim_timer, this));

    }
    else if(kSimulationMode == "raspi_sim")
    {
        RCLCPP_WARN(this->get_logger(), "Raspi control simulation enabled.");
    }
    else if(kSimulationMode == "hil")
    {
        RCLCPP_WARN(this->get_logger(), "HIL control simulation enabled.");
    }
    else
    {
        RCLCPP_ERROR(this->get_logger(), "Unknown simulation mode '%s'. No control simulation will be enabled.", kSimulationMode.c_str());
    }

    // Set CSV file
    if (kCSVState)
    {
        csv_generator_state_ = std::make_shared<CSVGenerator>("state");
    }
    if (kCSVVehicleDynamics)
    {
        auto csv_gen = std::make_shared<CSVGenerator>("vehicle_dynamics");
        vehicle_dynamics_.set_csv_generator(csv_gen);
    }

    prev_circuit_ = kTrackName;

    // Call load_track with kTrackName
    auto track_msg = std::make_shared<std_msgs::msg::String>();
    track_msg->data = kTrackName + ".pcd";
    load_track(track_msg);

}

/**
 * @brief Destructor for the Simulator class.
 *
 * Stops the rosbag recording if it is still active.
 */
Simulator::~Simulator()
{
    stop_rosbag_recording();
}

/**
 * @brief Determines if the vehicle is above, below, or on the line formed by the two cones.
 *
 * @param tpl_cones_ Vector of two points (cones), where each point is represented as a pair (x, y).
 */
void Simulator::check_lap()
{
    double x = vehicle_dynamics_.x_;
    double y = vehicle_dynamics_.y_;

    // Calculate the signed distance from the vehicle to the line equation through the TPLs
    double dist_to_tpl = y - tpl_coef_a_ * x - tpl_coef_b_;

    // Calculate the distance from the vehicle to the midpoint between the TPLs
    double d = std::sqrt(std::pow(x - mid_tpl_x_, 2) + std::pow(y - mid_tpl_y_, 2));

    if (dist_to_tpl * prev_dist_to_tpl_ < 0 and d < 5)
    {
        // Publish the result
        std_msgs::msg::Bool msg;
        msg.data = true;
        lap_signal_pub_->publish(msg);
    }
    prev_dist_to_tpl_ = dist_to_tpl;
}

/**
 * @brief Checks if car has started acc event
 *
 */
void Simulator::check_acc_start()
{
    double vx = vehicle_dynamics_.vx_;

    if (vx > 0 && !started_acc_)
    {
        std_msgs::msg::Bool msg;
        msg.data = true;
        lap_signal_pub_->publish(msg);
        started_acc_ = true;
    }
}

void Simulator::process_perception_queue()
{
    while (!perception_queue_.empty() && clock_->now() >= perception_queue_.front().publish_time)
    {
        lidar_perception_pub_->publish(perception_queue_.front().msg);
        perception_queue_.pop();
    }
}

/**
 * @brief Creates, configures and links POSIX sockets for the CAN bus
 */
void Simulator::init_can_sockets()
{

    if (kSimulationMode == "default" || kSimulationMode == "raspi_sim")
    {

        // Virtual can
        RCLCPP_INFO(this->get_logger(), "Rising virtual CAN interfaces (vcan0, vcan1 and vcan2)...");

        std::system("sudo modprobe vcan");

        std::system("ip link show can0 >/dev/null 2>&1 || sudo ip link add dev can0 type vcan");
        std::system("sudo ip link set up can0");

        std::system("ip link show can1 >/dev/null 2>&1 || sudo ip link add dev can1 type vcan");
        std::system("sudo ip link set up can1");

        std::system("ip link show can2 >/dev/null 2>&1 || sudo ip link add dev can2 type vcan");
        std::system("sudo ip link set up can2");
    }
    else if (kSimulationMode == "hil")
    {

        // Hardware CAN
        RCLCPP_INFO(this->get_logger(), "Rising hardware CAN interfaces (can0, can1 and can2)...");

        std::system("ip link show can0 >/dev/null 2>&1 && sudo ip link set can0 down"); 
        std::system("sudo ip link set can0 type can bitrate 1000000");                  
        std::system("sudo ip link set up can0");                                        

        std::system("ip link show can1 >/dev/null 2>&1 && sudo ip link set can1 down");
        std::system("sudo ip link set can1 type can bitrate 1000000");
        std::system("sudo ip link set up can1");

        std::system("ip link show can2 >/dev/null 2>&1 && sudo ip link set can2 down");
        std::system("sudo ip link set can2 type can bitrate 500000");
        std::system("sudo ip link set up can2");
    }
    else
    {
        RCLCPP_ERROR(this->get_logger(), "Error: Unknown simulation mode ('%s'). Aborting init_can.", kSimulationMode.c_str());
        return;
    }

    // Init CAN0
    can_socket_0_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    std::strcpy(ifr_0_.ifr_name, "can0");
    ioctl(can_socket_0_, SIOCGIFINDEX, &ifr_0_);
    addr_0_.can_family = AF_CAN;
    addr_0_.can_ifindex = ifr_0_.ifr_ifindex;
    bind(can_socket_0_, (struct sockaddr *)&addr_0_, sizeof(addr_0_));

    // Init CAN1
    can_socket_1_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    std::strcpy(ifr_1_.ifr_name, "can1");
    ioctl(can_socket_1_, SIOCGIFINDEX, &ifr_1_);
    addr_1_.can_family = AF_CAN;
    addr_1_.can_ifindex = ifr_1_.ifr_ifindex;
    bind(can_socket_1_, (struct sockaddr *)&addr_1_, sizeof(addr_1_));

    // Init CAN2
    can_socket_2_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    std::strcpy(ifr_2_.ifr_name, "can2");
    ioctl(can_socket_2_, SIOCGIFINDEX, &ifr_2_);
    addr_2_.can_family = AF_CAN;
    addr_2_.can_ifindex = ifr_2_.ifr_ifindex;
    bind(can_socket_2_, (struct sockaddr *)&addr_2_, sizeof(addr_2_));

    can_torque_cmd_ = {0.0, 0.0, 0.0, 0.0};
}

/**
 * @brief Slow timer callback for sensor data updates.
 *
 * This method updates the sensor data by publishing the track and generating
 * random noise to simulate sensor inaccuracy. It also publishes the perception data.
 */
void Simulator::on_slow_timer()
{
    double x = vehicle_dynamics_.x_;
    double y = vehicle_dynamics_.y_;
    double yaw = vehicle_dynamics_.yaw_;

    // Update track
    sensor_msgs::msg::PointCloud2 track_msg;
    pcl::toROSMsg(track_, track_msg);
    track_msg.header.stamp = clock_->now();
    track_msg.header.frame_id = "arussim/world";
    track_pub_->publish(track_msg);

    fixed_trajectory_pub_->publish(fixed_trajectory_msg_);

    // Random noise generation for position (lidar)
    std::random_device rd_p;
    std::mt19937 gen_p(rd_p());
    std::normal_distribution<> dist_p(0.0, kNoiseLidarPosPerception);

    // Random noise generation for visualization (lidar)
    std::random_device rd_v;
    std::mt19937 gen_v(rd_v());
    std::uniform_real_distribution<double> dist_v(0.0, kNoiseLidarProbPerception);

    // Random noise generation for color (lidar)
    std::random_device rd_c;
    std::mt19937 gen_c(rd_c());
    std::uniform_real_distribution<double> dist_c(0.0, kNoiseLidarColor);

    // Lidar perception simulation
    struct ConeData {
        PointXYZProbColorScore p;
        double d;
        double angle;
        double p_min;
        double p_max;
        double depth;
    };

    std::vector<ConeData> cones;
    auto perception_cloud = pcl::PointCloud<PointXYZProbColorScore>();

    for (auto &point : track_.points){

        double dx = point.x - (x + kPosLidarX * std::cos(yaw));
        double dy = point.y - (y + kPosLidarX * std::sin(yaw));
        double d = std::sqrt(dx * dx + dy * dy);

        if (d >= kMaxLidarDistance) continue;

        double angle_to_cone = std::atan2(dy, dx) - yaw;
        while (angle_to_cone > M_PI) angle_to_cone -= 2.0 * M_PI;
        while (angle_to_cone < -M_PI) angle_to_cone += 2.0 * M_PI;
        double delta = std::atan2(kConeWidth/2.0, d);

        PointXYZProbColorScore p;
        p.x = (point.x - x)*std::cos(yaw) + (point.y - y)*std::sin(yaw) + dist_p(gen_p);
        p.y = -(point.x - x)*std::sin(yaw) + (point.y - y)*std::cos(yaw) + dist_p(gen_p);
        p.z = 0.0;
        double p_r = std::exp(-0.005 * d);  // Exponential decay for color
        double a = std::log(2.0) / kMaxLidarDistance; // Calculate 'a' paramater for exponential decay based on max distance
        double p_v = std::exp(-a * d) - dist_v(gen_v); // Exponential decay for visualization
        if (point.color == 0) {
            p.prob_yellow = (point.color + dist_c(gen_c)) * p_r;
            p.prob_blue = (1 - point.color - dist_c(gen_c)) * p_r;
        } else if (point.color == 1) {
            p.prob_yellow = (point.color - dist_c(gen_c)) * p_r;
            p.prob_blue = (1 - point.color + dist_c(gen_c)) * p_r;
        } else {
            p.prob_yellow = 0.0 + dist_c(gen_c);
            p.prob_blue = 0.0 + dist_c(gen_c);
        }
        p.prob_yellow = std::clamp(p.prob_yellow, 0.0, 1.0);
        p.prob_blue   = std::clamp(p.prob_blue, 0.0, 1.0);
        p.score = 1.0;
        if (std::abs(angle_to_cone) < (kLidarFOV * M_PI / 180.0) / 2.0 && p.x > kPosLidarX + kMinPerceptionX && p_v > 0.5 && d > kMinLidarDistance) {
            ConeData c;
            c.p = p;
            c.d = d;
            c.angle = angle_to_cone;
            c.p_min = angle_to_cone - delta;
            c.p_max = angle_to_cone + delta;
            c.depth = d * (kLidarHeight / (kLidarHeight - kConeHeight));

            cones.push_back(c);
        } if (p.x >= kCOGBackDist && p.x <= kCOGFrontDist && p.y >= -kCarWidth && p.y <= kCarWidth) {
            arussim_msgs::msg::PointXY msg;
            msg.x = point.x;
            msg.y = point.y;
            hit_cones_pub_->publish(msg);
        }
    }
    
    std::sort(cones.begin(), cones.end(),
          [](const auto &a, const auto &b){ return a.d < b.d; });
    
    std::vector<bool> occluded(cones.size(), false);
    bool infinite_shadow = kLidarHeight <= kConeHeight;
    
    for (size_t i = 0; i < cones.size(); ++i) {
        const auto &coneA = cones[i];

        for (size_t j = i+1; j < cones.size(); ++j) { 
            const auto &coneB = cones[j];
            if (i == j || coneA.d >= coneB.d) continue;

            if (infinite_shadow) {
                if (!(coneB.p_max < coneA.p_min || coneB.p_min > coneA.p_max)) occluded[j] = true;
                continue;
            }

            double t = (coneB.d - coneA.d) / (coneA.depth - coneA.d);
            t = std::clamp(t, 0.0, 1.0);

            double half_width_A = (coneA.p_max - coneA.p_min) * 0.5;
            double half_width_eff = (1.0 - t) * half_width_A;
            double min_eff = coneA.angle - half_width_eff;
            double max_eff = coneA.angle + half_width_eff;

            bool z_ray = kLidarHeight * (1.0 - coneA.d / coneB.d) <= kConeHeight;
            //bool in_depth = (coneB.d >= coneA.d && coneB.d <= coneA.depth);

            if (!(coneB.p_max < min_eff || coneB.p_min > max_eff) && z_ray) occluded[j] = true;
        }
        if (!occluded[i]) perception_cloud.push_back(cones[i].p);
    }

    sensor_msgs::msg::PointCloud2 perception_msg;
    pcl::toROSMsg(perception_cloud, perception_msg);

    perception_msg.header.stamp = clock_->now(); 
    perception_msg.header.frame_id="arussim/vehicle_cog";

    double delay_ms = perception_delay_dist(perception_delay_gen);
    rclcpp::Time publish_time = clock_->now() + rclcpp::Duration::from_seconds(delay_ms / 1000.0);
    perception_queue_.push({perception_msg, publish_time});

    // Random noise generation for position (camera)
    std::random_device rd_pc;
    std::mt19937 gen_pc(rd_pc());
    std::normal_distribution<> dist_pc(0.0, kNoiseCameraPerception);

    // Random noise generation for color (camera)
    std::random_device rd_cc;
    std::mt19937 gen_cc(rd_cc());
    std::uniform_real_distribution<double> dist_cc(0.0, kNoiseCameraColor);

    // Camera perception simulation
    auto camera_perception_cloud = pcl::PointCloud<ConeXYZColorScore>();
    for (auto &point : track_.points)
    {
        double d = std::sqrt(std::pow(point.x - (x + kPosCameraX * std::cos(yaw)), 2) + std::pow(point.y - (y + kPosCameraX * std::sin(yaw)), 2));
        if (d < kCameraFOV)
        {
            ConeXYZColorScore p;
            p.x = (point.x - x) * std::cos(yaw) + (point.y - y) * std::sin(yaw) + dist_pc(gen_pc);
            p.y = -(point.x - x) * std::sin(yaw) + (point.y - y) * std::cos(yaw) + dist_pc(gen_pc);
            p.z = 0.0;
            if (point.color == 0)
            {
                p.color = point.color;
                p.score = (kCameraColorProbability - dist_cc(gen_cc));
            }
            else if (point.color == 1)
            {
                p.color = point.color;
                p.score = (kCameraColorProbability - dist_cc(gen_cc));
            }
            else
            {
                p.color = -1;
                p.score = kCameraColorProbability - dist_cc(gen_cc);
            }
            p.score = std::clamp(p.score, 0.0f, 1.0f);
            if (p.x > kPosCameraX)
            {
                camera_perception_cloud.push_back(p);
            }
        }
    }

    sensor_msgs::msg::PointCloud2 camera_perception_msg;
    pcl::toROSMsg(camera_perception_cloud, camera_perception_msg);
    camera_perception_msg.header.stamp = clock_->now();
    camera_perception_msg.header.frame_id = "arussim/vehicle_cog";
    camera_perception_pub_->publish(camera_perception_msg);

    cone_visualization();
}

void Simulator::on_controller_sim_timer()
{

    // DV command
    dv_.acc = can_acc_;
    dv_.target_r = can_target_r_;

    // Save controller output
    double tv_out[4], tc_out[4], pl_out[4], torque_cmd_out[4], state_out[4], Fz[4],
        fx_obj_tc[4], t_ff_tc[4], sr_tc[4], sr_t[4], sa_tc[4],
        PL_debug_data[4], TC_int_error[4], TC_calc[4], saturation;

    estimation_update(&sensors_, state_out, Fz);

    control_update(&sensors_, &dv_, tv_out, tc_out, pl_out, torque_cmd_out, state_out,
                fx_obj_tc, t_ff_tc, sr_tc, sr_t, sa_tc, PL_debug_data,
                TC_int_error, TC_calc, &saturation);

    torque_cmd_ = {
        static_cast<double>(torque_cmd_out[0] * car_parameters_.gear_ratio),
        static_cast<double>(torque_cmd_out[1] * car_parameters_.gear_ratio),
        static_cast<double>(torque_cmd_out[2] * car_parameters_.gear_ratio),
        static_cast<double>(torque_cmd_out[3] * car_parameters_.gear_ratio)};

    std_msgs::msg::Float32 control_vx_msg;
    control_vx_msg.data = state_out[0];
    control_vx_pub_->publish(control_vx_msg);

    std_msgs::msg::Float32 control_vy_msg;
    control_vy_msg.data = state_out[1];
    control_vy_pub_->publish(control_vy_msg);

    std_msgs::msg::Float32 control_r_msg;
    control_r_msg.data = state_out[2];
    control_r_pub_->publish(control_r_msg);

    int16_t vx_scaled = static_cast<int16_t>(control_vx_msg.data * 100);
    int16_t vy_scaled = static_cast<int16_t>(control_vy_msg.data * 100);
    int16_t yaw_rate_scaled = static_cast<int16_t>(control_r_msg.data * 1000);
    struct can_frame frame;
    frame.can_id = 0x122;
    frame.can_dlc = 6;
    frame.data[0] = vx_scaled & 0xFF;
    frame.data[1] = (vx_scaled >> 8) & 0xFF;
    frame.data[2] = vy_scaled & 0xFF;
    frame.data[3] = (vy_scaled >> 8) & 0xFF;
    frame.data[4] = yaw_rate_scaled & 0xFF;
    frame.data[5] = (yaw_rate_scaled >> 8) & 0xFF;
    write(can_socket_0_, &frame, sizeof(struct can_frame));

    // Publish control debug info
    auto tv_out_msg = arussim_msgs::msg::FourWheelDrive();
    tv_out_msg.front_left = tv_out[0];
    tv_out_msg.front_right = tv_out[1];
    tv_out_msg.rear_left = tv_out[2];
    tv_out_msg.rear_right = tv_out[3];
    tv_out_pub_->publish(tv_out_msg);

    auto tc_out_msg = arussim_msgs::msg::FourWheelDrive();
    tc_out_msg.front_left = tc_out[0];
    tc_out_msg.front_right = tc_out[1];
    tc_out_msg.rear_left = tc_out[2];   
    tc_out_msg.rear_right = tc_out[3];
    tc_out_pub_->publish(tc_out_msg);

    auto pl_out_msg = arussim_msgs::msg::FourWheelDrive();
    pl_out_msg.front_left = pl_out[0];
    pl_out_msg.front_right = pl_out[1];
    pl_out_msg.rear_left = pl_out[2];
    pl_out_msg.rear_right = pl_out[3];
    pl_out_pub_->publish(pl_out_msg);

    auto torque_cmd_msg = arussim_msgs::msg::FourWheelDrive();
    torque_cmd_msg.front_left = torque_cmd_out[0];
    torque_cmd_msg.front_right = torque_cmd_out[1];
    torque_cmd_msg.rear_left = torque_cmd_out[2];
    torque_cmd_msg.rear_right = torque_cmd_out[3];
    torque_cmd_pub_->publish(torque_cmd_msg);

    auto sr_t_msg = arussim_msgs::msg::FourWheelDrive();
    sr_t_msg.front_left = sr_t[0];
    sr_t_msg.front_right = sr_t[1];
    sr_t_msg.rear_left = sr_t[2];
    sr_t_msg.rear_right = sr_t[3];
    sr_t_pub_->publish(sr_t_msg);

    auto sr_tc_msg = arussim_msgs::msg::FourWheelDrive();
    sr_tc_msg.front_left = sr_tc[0];
    sr_tc_msg.front_right = sr_tc[1];
    sr_tc_msg.rear_left = sr_tc[2];
    sr_tc_msg.rear_right = sr_tc[3];
    sr_tc_pub_->publish(sr_tc_msg);

    auto ff_torque_msg = arussim_msgs::msg::FourWheelDrive();
    ff_torque_msg.front_left = t_ff_tc[0];
    ff_torque_msg.front_right = t_ff_tc[1];
    ff_torque_msg.rear_left = t_ff_tc[2];
    ff_torque_msg.rear_right = t_ff_tc[3];
    ff_torque_pub_->publish(ff_torque_msg);
    frame.can_id = 0x120;
    frame.can_dlc = 3;
    frame.data[2] = (uint8_t)saturation;
    write(can_socket_0_, &frame, sizeof(struct can_frame));
}

/**
 * @brief Fast timer callback for state updates.
 *
 * This method handles vehicle state updates, broadcasting the transform
 * and updating the vehicle marker in RViz.
 */
void Simulator::on_fast_timer()
{
    // Update state and broadcast transform
    rclcpp::Time current_time = clock_->now();

    // Record a rosbag while the AS status is Driving (0x03)
    if (as_status_ == 0x03 && prev_as_status_ != 0x03)
    {
        start_rosbag_recording();
    }
    else if (as_status_ != 0x03 && prev_as_status_ == 0x03)
    {
        stop_rosbag_recording();
    }
    prev_as_status_ = as_status_;

    if ((current_time - time_last_cmd_).seconds() > 0.2 && vehicle_dynamics_.vx_ != 0)
    {
        can_acc_ = 0;
    }
    if (as_status_ != 0x03)
    {
        // Trigger EBS
        vehicle_dynamics_.torque_cmd_.fl_ = 0.;
        vehicle_dynamics_.torque_cmd_.fr_ = 0.;
        vehicle_dynamics_.torque_cmd_.rl_ = 0.;
        vehicle_dynamics_.torque_cmd_.rr_ = 0.;
        vehicle_dynamics_.vx_ = 0.0;
        vehicle_dynamics_.vy_ = 0.0;
        vehicle_dynamics_.r_ = 0.0;
    }

    double dt = 1.0 / kStateUpdateRate;

    if (kSimulationMode == "default")
    {
        vehicle_dynamics_.update_simulation(can_delta_, torque_cmd_, dt);
    }
    else 
    {
        vehicle_dynamics_.update_simulation(can_delta_, can_torque_cmd_, dt);
    }

    if (use_tpl_)
    {
        check_lap();
    }

    if ((kTrackName == "acceleration" && !started_acc_) || (track_name_ == "acceleration" && !started_acc_))
    {
        check_acc_start();
    }

    auto message = arussim_msgs::msg::State();
    message.header.stamp = clock_->now();
    message.x = vehicle_dynamics_.x_;
    message.y = vehicle_dynamics_.y_;
    message.yaw = vehicle_dynamics_.yaw_;
    message.vx = vehicle_dynamics_.vx_;
    message.vy = vehicle_dynamics_.vy_;
    message.r = vehicle_dynamics_.r_;
    message.ax = vehicle_dynamics_.ax_;
    message.ay = vehicle_dynamics_.ay_;
    message.delta = vehicle_dynamics_.delta_;

    auto wheel_speeds = arussim_msgs::msg::FourWheelDrive();
    wheel_speeds.front_left = vehicle_dynamics_.wheel_speed_.fl_;
    wheel_speeds.front_right = vehicle_dynamics_.wheel_speed_.fr_;
    wheel_speeds.rear_left = vehicle_dynamics_.wheel_speed_.rl_;
    wheel_speeds.rear_right = vehicle_dynamics_.wheel_speed_.rr_;
    message.wheel_speeds = wheel_speeds;

    auto torque = arussim_msgs::msg::FourWheelDrive();
    torque.front_left = vehicle_dynamics_.torque_cmd_.fl_;
    torque.front_right = vehicle_dynamics_.torque_cmd_.fr_;
    torque.rear_left = vehicle_dynamics_.torque_cmd_.rl_;
    torque.rear_right = vehicle_dynamics_.torque_cmd_.rr_;
    message.torque = torque;

    if (kCSVState)
    {
        std::vector<std::string> row_values;
        row_values.push_back(std::to_string(vehicle_dynamics_.x_));
        row_values.push_back(std::to_string(vehicle_dynamics_.y_));
        row_values.push_back(std::to_string(vehicle_dynamics_.yaw_));
        row_values.push_back(std::to_string(vehicle_dynamics_.vx_));
        row_values.push_back(std::to_string(vehicle_dynamics_.vy_));
        row_values.push_back(std::to_string(vehicle_dynamics_.r_));
        row_values.push_back(std::to_string(vehicle_dynamics_.ax_));
        row_values.push_back(std::to_string(vehicle_dynamics_.ay_));
        row_values.push_back(std::to_string(vehicle_dynamics_.delta_));
        csv_generator_state_->write_row("x,y,yaw,vx,vy,r,ax,ay,delta", row_values);
    }
    if (kCSVVehicleDynamics)
    {
        vehicle_dynamics_.write_csv_row();
    }

    state_pub_->publish(message);

    auto ground_truth_msg = common_msgs::msg::State();
    ground_truth_msg.x = vehicle_dynamics_.x_;
    ground_truth_msg.y = vehicle_dynamics_.y_;
    ground_truth_msg.yaw = vehicle_dynamics_.yaw_;
    ground_truth_msg.vx = vehicle_dynamics_.vx_;
    ground_truth_msg.vy = vehicle_dynamics_.vy_;
    ground_truth_msg.r = vehicle_dynamics_.r_;
    ground_truth_msg.ax = vehicle_dynamics_.ax_;
    ground_truth_msg.ay = vehicle_dynamics_.ay_;
    ground_truth_msg.delta = vehicle_dynamics_.delta_;

    ground_truth_pub_->publish(ground_truth_msg);

    auto slip_ratio_msg = arussim_msgs::msg::FourWheelDrive();
    slip_ratio_msg.front_left = vehicle_dynamics_.tire_slip_.lambda_fl_;
    slip_ratio_msg.front_right = vehicle_dynamics_.tire_slip_.lambda_fr_;
    slip_ratio_msg.rear_left = vehicle_dynamics_.tire_slip_.lambda_rl_;
    slip_ratio_msg.rear_right = vehicle_dynamics_.tire_slip_.lambda_rr_;
    slip_ratio_pub_->publish(slip_ratio_msg);

    auto slip_angle_msg = arussim_msgs::msg::FourWheelDrive();
    slip_angle_msg.front_left = vehicle_dynamics_.tire_slip_.alpha_fl_;
    slip_angle_msg.front_right = vehicle_dynamics_.tire_slip_.alpha_fr_;
    slip_angle_msg.rear_left = vehicle_dynamics_.tire_slip_.alpha_rl_;
    slip_angle_msg.rear_right = vehicle_dynamics_.tire_slip_.alpha_rr_;
    slip_angle_pub_->publish(slip_angle_msg);

    auto tire_load_msg = arussim_msgs::msg::FourWheelDrive();
    tire_load_msg.front_left = vehicle_dynamics_.tire_loads_.fl_;
    tire_load_msg.front_right = vehicle_dynamics_.tire_loads_.fr_;
    tire_load_msg.rear_left = vehicle_dynamics_.tire_loads_.rl_;
    tire_load_msg.rear_right = vehicle_dynamics_.tire_loads_.rr_;
    tire_load_pub_->publish(tire_load_msg);

    broadcast_transform();

    // Update vehicle marker
    marker_.header.stamp = clock_->now();
    marker_pub_->publish(marker_);
}

void Simulator::receive_can_0()
{
    int flags = fcntl(can_socket_0_, F_GETFL, 0);
    fcntl(can_socket_0_, F_SETFL, flags | O_NONBLOCK);
    while (rclcpp::ok())
    {
        int nbytes = read(can_socket_0_, &frame_0_, sizeof(struct can_frame));
        if (nbytes < 0)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
            continue;
        }

        if (frame_0_.can_id == 0x122)
        {
            int16_t state_0_raw = static_cast<int16_t>((frame_0_.data[1] << 8) | frame_0_.data[0]);
            int16_t state_1_raw = static_cast<int16_t>((frame_0_.data[3] << 8) | frame_0_.data[2]);
            int16_t state_2_raw = static_cast<int16_t>((frame_0_.data[5] << 8) | frame_0_.data[4]);

            double can_state_0 = static_cast<double>(state_0_raw) / 100.0;
            double can_state_1 = static_cast<double>(state_1_raw) / 100.0;
            double can_state_2 = static_cast<double>(state_2_raw) / 1000.0;

        }

        else if (frame_0_.can_id == 0x222)
        {
            int16_t acc_scaled = static_cast<int16_t>((frame_0_.data[1] << 8) | frame_0_.data[0]);
            int16_t yaw_scaled = static_cast<int16_t>((frame_0_.data[3] << 8) | frame_0_.data[2]);
            int16_t delta_scaled = static_cast<int16_t>((frame_0_.data[5] << 8) | frame_0_.data[4]);

            can_acc_ = static_cast<float>(acc_scaled) / 100.0f;
            can_target_r_ = static_cast<float>(yaw_scaled) / 1000.0f;
            // can_delta_ = static_cast<float>(delta_scaled) / 100.0f;
            time_last_cmd_ = clock_->now();
        }

        else if (frame_0_.can_id == 0x261)
        {
            as_status_ = frame_0_.data[1];
        }

        else if ((frame_0_.can_id & CAN_EFF_MASK) == 0x401 || (frame_0_.can_id & CAN_EFF_MASK) == 0x601)
        {
            int32_t delta_scaled = static_cast<int32_t>((frame_0_.data[0] << 24) | (frame_0_.data[1] << 16) | (frame_0_.data[2] << 8) | frame_0_.data[3]);
            can_delta_ = static_cast<float>(delta_scaled) / 10000.0f * M_PI / 180.0f / 4.279f;
        }
    }
}

void Simulator::receive_can_1()
{
    int flags = fcntl(can_socket_1_, F_GETFL, 0);
    fcntl(can_socket_1_, F_SETFL, flags | O_NONBLOCK);
    while (rclcpp::ok())
    {
        int nbytes = read(can_socket_1_, &frame_1_, sizeof(struct can_frame));
        if (nbytes < 0)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
            continue;
        }

    }
}

void Simulator::receive_can_2()
{
    int flags = fcntl(can_socket_2_, F_GETFL, 0);
    fcntl(can_socket_2_, F_SETFL, flags | O_NONBLOCK);
    while (rclcpp::ok())
    {
        int nbytes = read(can_socket_2_, &frame_2_, sizeof(struct can_frame));
        if (nbytes < 0)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
            continue;
        }

        if (frame_0_.can_id == 0x200 || frame_0_.can_id == 0x203 ||
                 frame_0_.can_id == 0x206 || frame_0_.can_id == 0x209)
        {
            int idx = (frame_0_.can_id - 0x200) / 3; // 0,1,2,3
            int16_t torque_scaled = static_cast<int16_t>((frame_0_.data[3] << 8) | frame_0_.data[2]);
            can_torque_cmd_.at(idx) = torque_scaled * 9.8 / 1000.0 * car_parameters_.gear_ratio;
        }
    }
}

void Simulator::launch_callback([[maybe_unused]] const std_msgs::msg::Bool::SharedPtr msg)
{
    if (as_status_ == 0x02)
        as_status_ = 0x03;

    if (msg && msg->data && (kSimulationMode == "raspi_sim"))
    {
        control_raspi_manager_.start_control_rasp(this->get_logger());
    }
}

/**
 * @brief Starts recording a rosbag of the current run in ~/ARUS_logs/arussim/.
 *
 * Only one recording is kept: the previous one is deleted before starting.
 */
void Simulator::start_rosbag_recording()
{
    if (rosbag_pid_ > 0)
        return;

    const char* home = std::getenv("HOME");
    if (!home)
    {
        RCLCPP_ERROR(this->get_logger(), "HOME is not set, the rosbag will not be recorded");
        return;
    }

    std::filesystem::path bag_dir = std::filesystem::path(home) / "ARUS_logs" / "arussim";
    std::filesystem::path bag_path = bag_dir / "sim_rosbag";

    std::error_code ec;
    std::filesystem::create_directories(bag_dir, ec);
    std::filesystem::remove_all(bag_path, ec);

    pid_t pid = fork();
    if (pid == 0)
    {
        // Own process group so the recorder only receives the signals we send it
        setpgid(0, 0);
        execlp("ros2", "ros2", "bag", "record", "-a", "-s", "mcap",
               "-o", bag_path.c_str(), (char*)NULL);
        _exit(1);
    }
    else if (pid > 0)
    {
        rosbag_pid_ = pid;
        RCLCPP_INFO(this->get_logger(), "Recording rosbag in %s", bag_path.c_str());
    }
    else
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to start the rosbag recording");
    }
}

/**
 * @brief Stops the active rosbag recording.
 */
void Simulator::stop_rosbag_recording()
{
    if (rosbag_pid_ <= 0)
        return;

    kill(rosbag_pid_, SIGINT);
    waitpid(rosbag_pid_, nullptr, 0);
    rosbag_pid_ = -1;
    RCLCPP_INFO(this->get_logger(), "Rosbag recording stopped");
}

/**
 * @brief Callback for receiving noisy x_acceleration data.
 *
 * This method updates the noisy acceleration value based on incoming messages.
 * @param msg Incoming message containing the noisy x_acceleration data.
 */
void Simulator::noisy_ax_callback(const std_msgs::msg::Float32::SharedPtr msg)
{
    sensors_.acceleration_x = msg->data;
}

/**
 * @brief Callback for receiving noisy y_acceleration data.
 *
 * This method updates the noisy acceleration value based on incoming messages.
 * @param msg Incoming message containing the noisy y_acceleration data.
 */
void Simulator::noisy_ay_callback(const std_msgs::msg::Float32::SharedPtr msg)
{
    sensors_.acceleration_y = msg->data;
}

/**
 * @brief Callback for receiving noisy angular velocity data.
 *
 * This method updates the noisy angular velocity value based on incoming messages.
 * @param msg Incoming message containing the noisy angular velocity data.
 */
void Simulator::noisy_r_callback(const std_msgs::msg::Float32::SharedPtr msg)
{
    sensors_.angular_z = msg->data;
}

/**
 * @brief Callback for receiving noisy motor speed data.
 *
 * This method updates the noisy motor speed value based on incoming messages.
 * @param msg Incoming message containing the noisy motor speed data.
 */
void Simulator::noisy_ms_callback(const arussim_msgs::msg::FourWheelDrive::SharedPtr msg)
{
    sensors_.motor_speed[0] = msg->front_left;
    sensors_.motor_speed[1] = msg->front_right;
    sensors_.motor_speed[2] = msg->rear_left;
    sensors_.motor_speed[3] = msg->rear_right;
}

/**
 * @brief Callback for receiving noisy motor torque data.
 *
 * This method updates the noisy motor torque value based on incoming messages.
 * @param msg Incoming message containing the noisy motor torque data.
 */
void Simulator::noisy_torque_callback(const arussim_msgs::msg::FourWheelDrive::SharedPtr msg)
{
    sensors_.motor_torque[0] = msg->front_left;
    sensors_.motor_torque[1] = msg->front_right;
    sensors_.motor_torque[2] = msg->rear_left;
    sensors_.motor_torque[3] = msg->rear_right;
}

/**
 * @brief Callbacks for receiving noisy groundspeed data.
 *
 */
void Simulator::noisy_vx_callback(const std_msgs::msg::Float32::SharedPtr msg)
{
    sensors_.speed_x = msg->data;
}

void Simulator::noisy_vy_callback(const std_msgs::msg::Float32::SharedPtr msg)
{
    sensors_.speed_y = msg->data;
}

void Simulator::noisy_delta_callback(const std_msgs::msg::Float32::SharedPtr msg)
{
    sensors_.steering_angle = msg->data;
}


void Simulator::ebs_callback(const std_msgs::msg::Bool::SharedPtr msg)
{
    if (msg->data)
    {
        can_acc_ = 0.0;
        can_delta_ = 0.0;
        torque_cmd_ = {0.0, 0.0, 0.0, 0.0};
        vehicle_dynamics_.vx_ = 0.0;
        vehicle_dynamics_.vy_ = 0.0;
        vehicle_dynamics_.r_ = 0.0;
    }
}

void Simulator::reset_callback([[maybe_unused]] const std_msgs::msg::Bool::SharedPtr msg)
{
    if (msg && msg->data && (kSimulationMode == "default")) 
    {
        sensors_.speed_x = 0.;
        sensors_.speed_y = 0.;
        sensors_.angular_z = 0.;
        sensors_.acceleration_x = 0.;
        sensors_.acceleration_y = 0.;
        sensors_.steering_angle = 0.;

        control_init(&sensors_, &car_parameters_, &dv_);
    }
    else if (msg && msg->data && (kSimulationMode == "raspi_sim")) 
    {
        control_raspi_manager_.stop_control_rasp(this->get_logger());
    }

    torque_cmd_ = {0.0, 0.0, 0.0, 0.0};
    can_acc_ = 0.0;
    can_delta_ = 0.0;
    can_target_r_ = 0.0;
    can_torque_cmd_ = {0.0, 0.0, 0.0, 0.0};

    as_status_ = 0x02;
    vehicle_dynamics_ = VehicleDynamics();

    try {
        this->simulation_car_csv_ = kSimulationCar + ".csv";
        std::string csv_path = this->get_csv_path(this->simulation_car_csv_);
        this->parameters_map_ = this->load_car_parameters(csv_path);
        this->vehicle_dynamics_.set_parameters(this->parameters_map_);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Failed loading car parameters: %s", e.what());
    }

    started_acc_ = false;
    if (prev_circuit_ != track_name_)
    {
        fixed_trajectory_msg_.points.clear();
        fixed_trajectory_msg_.s.clear();
        fixed_trajectory_msg_.k.clear();
        fixed_trajectory_msg_.speed_profile.clear();
        fixed_trajectory_msg_.acc_profile.clear();
        auto track_msg = std::make_shared<std_msgs::msg::String>();
        track_msg->data = track_name_ + ".pcd";
        load_track(track_msg);
    }

    // Clear cone markers
    visualization_msgs::msg::MarkerArray empty_markers;
    cone_marker_pub_->publish(empty_markers);

    for (auto &marker : current_cone_markers_.markers)
    {
        marker.action = visualization_msgs::msg::Marker::DELETE;
    }
    cone_marker_pub_->publish(current_cone_markers_);
    current_cone_markers_.markers.clear();
}

/**
 * @brief Callback for receiving RViz teleportation commands.
 *
 * This method teleports the vehicle to a new pose based on incoming RViz commands.
 *
 * @param msg Incoming pose message from RViz.
 */
void Simulator::rviz_telep_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
{
    tf2::Quaternion q(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
                      msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
    tf2::Matrix3x3 mat(q);
    double roll, pitch, yaw;
    mat.getRPY(roll, pitch, yaw);

    vehicle_dynamics_.x_ = msg->pose.pose.position.x;
    vehicle_dynamics_.y_ = msg->pose.pose.position.y;
    vehicle_dynamics_.yaw_ = yaw;
    vehicle_dynamics_.vx_ = 0;
    vehicle_dynamics_.vy_ = 0;
    vehicle_dynamics_.r_ = 0;
}

/**
 * @brief Broadcasts the transform of the vehicle to the TF tree.
 *
 * This method sends the current pose of the vehicle to the ROS TF system
 * to be visualized in RViz or used in other nodes.
 */
void Simulator::broadcast_transform()
{
    // Create a TransformStamped message
    geometry_msgs::msg::TransformStamped transform_stamped;

    // Set the frame ID and child frame ID
    transform_stamped.header.stamp = clock_->now();
    transform_stamped.header.frame_id = "arussim/world";
    transform_stamped.child_frame_id = "arussim/vehicle_cog";

    // Set the translation (x, y, z)
    transform_stamped.transform.translation.x = vehicle_dynamics_.x_;
    transform_stamped.transform.translation.y = vehicle_dynamics_.y_;
    transform_stamped.transform.translation.z = 0.0;

    // Set the rotation (using a quaternion)
    tf2::Quaternion q;
    q.setRPY(0, 0, vehicle_dynamics_.yaw_); // Roll, Pitch, Yaw in radians
    transform_stamped.transform.rotation.x = q.x();
    transform_stamped.transform.rotation.y = q.y();
    transform_stamped.transform.rotation.z = q.z();
    transform_stamped.transform.rotation.w = q.w();

    // Broadcast the transform
    tf_broadcaster_->sendTransform(transform_stamped);
}

/**
 * @brief Filters the track point cloud to extract the TPLs.
 *
 * @param track
 */
void Simulator::load_track(const std_msgs::msg::String::SharedPtr track_msg)
{
    // Extract and remove the ".pcd" suffix if present
    track_name_ = track_msg->data;
    if (track_name_.size() >= 4 && track_name_.substr(track_name_.size() - 4) == ".pcd")
    {
        track_name_.resize(track_name_.size() - 4);
    }

    // Load the track point cloud using the .pcd extension
    std::string package_path = ament_index_cpp::get_package_share_directory("arussim");
    std::string filename = package_path + "/resources/tracks/" + track_name_ + ".pcd";
    if (pcl::io::loadPCDFile<ConeXYZColorScore>(filename, track_) == -1)
    {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Couldn't read file %s", filename.c_str());
        return;
    }
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Loaded %ld data points from %s",
                track_.points.size(), filename.c_str());

    // Clear previous TPL cones
    tpl_cones_.clear();

    // Extract the TPLs
    for (const auto &point : track_.points)
    {
        if (point.color == 4)
        {
            tpl_cones_.push_back(std::make_pair(point.x, point.y));
        }
    }
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "TPLs: %ld", tpl_cones_.size());

    if (tpl_cones_.size() != 2)
    {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "tpl_cones_ does not contain exactly 2 points.");
        use_tpl_ = false;
    }
    else
    {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "TPL1: (%f, %f), TPL2: (%f, %f)",
                    tpl_cones_[0].first, tpl_cones_[0].second, tpl_cones_[1].first, tpl_cones_[1].second);
        use_tpl_ = true;

        // Coordinates of the two cones
        double x1 = tpl_cones_[0].first;
        double y1 = tpl_cones_[0].second;
        double x2 = tpl_cones_[1].first;
        double y2 = tpl_cones_[1].second;

        // Calculate slope (a) and y-intercept (b)
        tpl_coef_a_ = (y2 - y1) / (x2 - x1 + 0.000001); // Avoid division by zero
        tpl_coef_b_ = y1 - tpl_coef_a_ * x1;

        // Calculate midpoint between the two cones
        mid_tpl_x_ = (x1 + x2) / 2.0;
        mid_tpl_y_ = (y1 + y2) / 2.0;
    }

    // Extract the fixed trajectory using the .json extension
    std::string json_filename = package_path + "/resources/tracks/" + track_name_ + ".json";
    std::ifstream tray_json(json_filename);
    if (!tray_json.is_open())
    {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Couldn't open JSON trajectory file %s", json_filename.c_str());
        return;
    }

    nlohmann::json tray_data;
    tray_json >> tray_data;
    tray_json.close();

    std::vector<float> traj_x = tray_data["x"].get<std::vector<float>>();
    std::vector<float> traj_y = tray_data["y"].get<std::vector<float>>();
    std::vector<float> traj_s = tray_data["s"].get<std::vector<float>>();
    std::vector<float> traj_k = tray_data["k"].get<std::vector<float>>();
    std::vector<float> traj_speed_profile = tray_data["speed_profile"].get<std::vector<float>>();
    std::vector<float> traj_acc_profile = tray_data["acc_profile"].get<std::vector<float>>();

    if (traj_x.size() == traj_y.size() && traj_x.size() > 0)
    {
        for (size_t i = 0; i < traj_x.size(); i++)
        {
            arussim_msgs::msg::PointXY point;
            point.x = traj_x[i];
            point.y = traj_y[i];
            fixed_trajectory_msg_.points.push_back(point);
        }
    }

    if (traj_s.size() == traj_k.size() && traj_s.size() > 0)
    {
        for (size_t i = 0; i < traj_s.size(); i++)
        {
            fixed_trajectory_msg_.s.push_back(traj_s[i]);
            fixed_trajectory_msg_.k.push_back(traj_k[i]);
        }
    }

    if (traj_speed_profile.size() > 0)
    {
        for (size_t i = 0; i < traj_speed_profile.size(); i++)
        {
            fixed_trajectory_msg_.speed_profile.push_back(traj_speed_profile[i]);
        }
    }

    if (traj_acc_profile.size() > 0)
    {
        for (size_t i = 0; i < traj_acc_profile.size(); i++)
        {
            fixed_trajectory_msg_.acc_profile.push_back(traj_acc_profile[i]);
        }
    }
}

/**
 * @brief Get the path to the CSV file based on simulation_car
 */
std::string Simulator::get_csv_path(const std::string &csv_filename)
{
    // Implementation for loading car parameters based on simulation_car
    std::string parameters_directory = ament_index_cpp::get_package_share_directory("arussim") + "/resources/parameters/";
    std::string parameters_path = parameters_directory + csv_filename;
    return parameters_path;
}

/**
 * @brief Load car parameters from a CSV file and return them as a map.
 * @param filepath The path to the CSV file containing the car parameters.
 */
std::map<std::string, double> Simulator::load_car_parameters(const std::string &filepath)
{
    std::map<std::string, double> parameters_map;
    std::ifstream file(filepath);
    if (!file.is_open())
    {
        RCLCPP_ERROR(
            this->get_logger(),
            "Could not open CSV parameter file: %s",
            filepath.c_str());
        return parameters_map;
    }

    std::string line;

    // Skip the first line (header)
    std::getline(file, line);

    // Read the file line by line
    while (std::getline(file, line))
    {
        std::stringstream ss(line);
        std::string key;
        std::string value_str;

        // Separate the line by commas
        if (std::getline(ss, key, ',') && std::getline(ss, value_str, ','))
        {
            try
            {
                double value = std::stod(value_str);
                parameters_map[key] = value;
            }
            catch (const std::invalid_argument &e)
            {
                RCLCPP_WARN(
                    this->get_logger(),
                    "Warning: Parameter '%s' is not a number: '%s'",
                    key.c_str(), value_str.c_str());
            }
        }
    }

    file.close();
    return parameters_map;
}

void Simulator::load_control_parameters()
{
    // Car data - Distances
    car_parameters_.g = parameters_map_["g"];
    car_parameters_.trackwidthF = parameters_map_["trackwidthF"];
    car_parameters_.trackwidthR = parameters_map_["trackwidthR"];
    car_parameters_.wheelbase = parameters_map_["wheelbase"];
    car_parameters_.r_cdg = parameters_map_["r_cdg"];
    car_parameters_.x_cdg = car_parameters_.wheelbase * car_parameters_.r_cdg;
    car_parameters_.lf = car_parameters_.wheelbase * car_parameters_.r_cdg;
    car_parameters_.lr = car_parameters_.wheelbase * (1.0 - car_parameters_.r_cdg);
    
    // Mass and inertia
    car_parameters_.Iz = parameters_map_["Iz"];
    car_parameters_.Ixx_F = parameters_map_["Ixx_F"];
    car_parameters_.Ixx_R = parameters_map_["Ixx_R"];
    car_parameters_.Iyy = parameters_map_["Iyy"];
    car_parameters_.nsm_f = parameters_map_["nsm_f"];
    car_parameters_.nsm_r = parameters_map_["nsm_r"];
    car_parameters_.sm = parameters_map_["sm"];
    car_parameters_.sm_f = car_parameters_.sm * (1.0 - car_parameters_.r_cdg);
    car_parameters_.sm_r = car_parameters_.sm * car_parameters_.r_cdg;
    car_parameters_.mass = car_parameters_.sm + car_parameters_.nsm_f + car_parameters_.nsm_r;
    
    car_parameters_.h_cdg = parameters_map_["h_cdg"];
    car_parameters_.h_cdg_nsm_f = parameters_map_["h_cdg_nsm_f"];
    car_parameters_.h_cdg_nsm_r = parameters_map_["h_cdg_nsm_r"];
    car_parameters_.h_cdg_sm = ((car_parameters_.mass * car_parameters_.h_cdg) - (car_parameters_.nsm_f * car_parameters_.h_cdg_nsm_f) 
        - (car_parameters_.nsm_r * car_parameters_.h_cdg_nsm_r)) / car_parameters_.sm;
    
    car_parameters_.h_RC_f = parameters_map_["h_RC_f"];
    car_parameters_.h_RC_r = parameters_map_["h_RC_r"];
    car_parameters_.h_RA = car_parameters_.h_RC_f + (car_parameters_.h_RC_r - car_parameters_.h_RC_f) * car_parameters_.lf / car_parameters_.wheelbase;
    
    // Stiffness
    // car_parameters_.K_tire = parameters_map_["K_tire_FL"];
    car_parameters_.k_F = parameters_map_["k_F"];
    car_parameters_.k_R = parameters_map_["k_R"];
    car_parameters_.k_ARB_F = parameters_map_["k_ARB_F"];
    car_parameters_.k_ARB_R = parameters_map_["k_ARB_R"];
    car_parameters_.K_tors = parameters_map_["K_tors"];
    car_parameters_.d_f = parameters_map_["d_f"];
    car_parameters_.d_r = parameters_map_["d_r"];
    car_parameters_.K_tire_FL = parameters_map_["K_tire_FL"];
    car_parameters_.K_tire_FR = parameters_map_["K_tire_FR"];
    car_parameters_.K_tire_RL = parameters_map_["K_tire_RL"];
    car_parameters_.K_tire_RR = parameters_map_["K_tire_RR"];
    car_parameters_.d_tire_FL = parameters_map_["d_tire_FL"];
    car_parameters_.d_tire_FR = parameters_map_["d_tire_FR"];
    car_parameters_.d_tire_RL = parameters_map_["d_tire_RL"];
    car_parameters_.d_tire_RR = parameters_map_["d_tire_RR"];
    car_parameters_.pressure_F = parameters_map_["pressure_F"];
    car_parameters_.pressure_R = parameters_map_["pressure_R"];
    
    // Motion Ratios
    car_parameters_.MR_F = parameters_map_["MR_F"];
    car_parameters_.MR_R = parameters_map_["MR_R"];
    car_parameters_.MR_ARB_F = parameters_map_["MR_ARB_F"];
    car_parameters_.MR_ARB_R = parameters_map_["MR_ARB_R"];
    double WR_f = car_parameters_.k_F / (car_parameters_.MR_F * car_parameters_.MR_F);
    double WR_r = car_parameters_.k_R / (car_parameters_.MR_R * car_parameters_.MR_R);
    
    if (car_parameters_.MR_ARB_F == 0) {
        car_parameters_.RS_f = 0.5 * car_parameters_.trackwidthF * car_parameters_.trackwidthF * WR_f;
    } else {
        car_parameters_.RS_f = 0.5 * car_parameters_.trackwidthF * car_parameters_.trackwidthF 
            * (WR_f + car_parameters_.k_ARB_F / (car_parameters_.MR_ARB_F * car_parameters_.MR_ARB_F));
    }
    
    if (car_parameters_.MR_ARB_R == 0) {
        car_parameters_.RS_r = 0.5 * car_parameters_.trackwidthR * car_parameters_.trackwidthR * WR_r;
    } else {
        car_parameters_.RS_r = 0.5 * car_parameters_.trackwidthR * car_parameters_.trackwidthR 
            * (WR_r + car_parameters_.k_ARB_R / (car_parameters_.MR_ARB_R * car_parameters_.MR_ARB_R));
    }
    
    car_parameters_.RS = car_parameters_.RS_f + car_parameters_.RS_r;
    
    car_parameters_.gear_ratio = parameters_map_["gear_ratio"];
    car_parameters_.rdyn = parameters_map_["rdyn"];
    car_parameters_.I_wheel_F = parameters_map_["I_wheel_F"];
    car_parameters_.I_wheel_R = parameters_map_["I_wheel_R"];
    
    // Torque limits
    double torque_max = parameters_map_["torque_limit_positive"];
    double torque_min = parameters_map_["torque_limit_negative"];
    for (int i = 0; i < 4; i++) {
        car_parameters_.torque_limit_positive[i] = torque_max;
        car_parameters_.torque_limit_negative[i] = torque_min;
    }
    
    // 2WD mode adjustment
    car_parameters_.mode_2wd = (int)parameters_map_["mode_2wd"];
    if (car_parameters_.mode_2wd) {
        car_parameters_.torque_limit_positive[0] = 0;
        car_parameters_.torque_limit_positive[1] = 0;
        car_parameters_.torque_limit_negative[0] = 0;
        car_parameters_.torque_limit_negative[1] = 0;
    }
    
    // Ackermann
    car_parameters_.ackermann[0] = parameters_map_["ackermann0"];
    car_parameters_.ackermann[1] = parameters_map_["ackermann1"];
    
    // Steering
    car_parameters_.toe_FL = parameters_map_["toe_FL"];
    car_parameters_.toe_FR = parameters_map_["toe_FR"];
    car_parameters_.toe_RL = parameters_map_["toe_RL"];
    car_parameters_.toe_RR = parameters_map_["toe_RR"];
    car_parameters_.camber_FL = parameters_map_["camber_FL"];
    car_parameters_.camber_FR = parameters_map_["camber_FR"];
    car_parameters_.camber_RL = parameters_map_["camber_RL"];
    car_parameters_.camber_RR = parameters_map_["camber_RR"];
    car_parameters_.delta_max_FL = parameters_map_["delta_max_FL"];
    car_parameters_.delta_max_FR = parameters_map_["delta_max_FR"];
    car_parameters_.r_steer = parameters_map_["r_steer"];
    
    // Aero
    car_parameters_.rho = parameters_map_["rho"];
    car_parameters_.CDA = parameters_map_["CDA"];
    car_parameters_.CLA = parameters_map_["CLA"];
    car_parameters_.r_cdp = parameters_map_["r_cdp"];
    car_parameters_.h_cdp = parameters_map_["h_cdp"];
    car_parameters_.x_cdp = parameters_map_["x_cdp"];
    
    // Electrical / power
    car_parameters_.P_max_powerunit = parameters_map_["P_max_powerunit"];
    car_parameters_.P_loss_offset = parameters_map_["P_loss_offset"];
    car_parameters_.P_inverter_loss = parameters_map_["P_inverter_loss"];
    car_parameters_.R_wire_F = parameters_map_["R_wire_F"];
    car_parameters_.R_wire_R = parameters_map_["R_wire_R"];
    car_parameters_.R_battery = parameters_map_["R_battery"];
    car_parameters_.cell_capacity = parameters_map_["cell_capacity"];
    car_parameters_.cell_voltage_max = parameters_map_["cell_voltage_max"];
    car_parameters_.cell_voltage_min = parameters_map_["cell_voltage_min"];
    car_parameters_.n_cells_series = (int)parameters_map_["n_cells_series"];
    car_parameters_.n_cells_parallel = (int)parameters_map_["n_cells_parallel"];
    car_parameters_.V_max = car_parameters_.cell_voltage_max * car_parameters_.n_cells_series;
    car_parameters_.V_min = car_parameters_.cell_voltage_min * car_parameters_.n_cells_series;

    // Pacejka parameters
    car_parameters_.Fz0 = parameters_map_["Fz0"];

    car_parameters_.D1_x = parameters_map_["D1_x"];
    car_parameters_.D2_x = parameters_map_["D2_x"];
    car_parameters_.Cx   = parameters_map_["Cx"];
    car_parameters_.Bx   = parameters_map_["Bx"];
    car_parameters_.Ex   = parameters_map_["Ex"];

    car_parameters_.D1_y = parameters_map_["D1_y"];
    car_parameters_.D2_y = parameters_map_["D2_y"];
    car_parameters_.Cy   = parameters_map_["Cy"];
    car_parameters_.By   = parameters_map_["By"];
    car_parameters_.Ey   = parameters_map_["Ey"];

    car_parameters_.SH = parameters_map_["SH"];
    car_parameters_.SV = parameters_map_["SV"];

    car_parameters_.rB1_x = parameters_map_["rB1_x"];
    car_parameters_.rB2_x = parameters_map_["rB2_x"];
    car_parameters_.rC1_x = parameters_map_["rC1_x"];
    car_parameters_.rE1_x = parameters_map_["rE1_x"];

    car_parameters_.rB1_y = parameters_map_["rB1_y"];
    car_parameters_.rB2_y = parameters_map_["rB2_y"];
    car_parameters_.rC1_y = parameters_map_["rC1_y"];
    car_parameters_.rSh   = parameters_map_["rSh"];

    car_parameters_.rGx1 = parameters_map_["rGx1"];
    car_parameters_.rBx  = parameters_map_["rBx"];
    car_parameters_.rAx  = parameters_map_["rAx"];
    car_parameters_.rCx  = parameters_map_["rCx"];

    car_parameters_.rGy1 = parameters_map_["rGy1"];
    car_parameters_.rBy  = parameters_map_["rBy"];

    car_parameters_.comb_model = (int)parameters_map_["comb_model"];
}

/**
 * @brief Make a MarkerArray of all cones of the track
 *
 */
void Simulator::cone_visualization()
{
    visualization_msgs::msg::MarkerArray cone_markers;
    int id_counter = 0;

    for (const auto &point : track_.points)
    {
        visualization_msgs::msg::Marker cone_marker;
        cone_marker.header.frame_id = "arussim/world";
        cone_marker.ns = "arussim/cones";
        cone_marker.id = id_counter++; // unique ID for each cone
        cone_marker.type = visualization_msgs::msg::Marker::MESH_RESOURCE;
        cone_marker.mesh_resource = "package://arussim/resources/meshes/cone.stl";
        cone_marker.action = visualization_msgs::msg::Marker::ADD;
        cone_marker.pose.position.x = point.x;
        cone_marker.pose.position.y = point.y;
        cone_marker.pose.position.z = 0.0;
        cone_marker.scale.x = 0.001;
        cone_marker.scale.y = 0.001;
        cone_marker.scale.z = 0.001;
        if (point.color == 0)
        {
            cone_marker.color.r = 0.0;
            cone_marker.color.g = 0.0;
            cone_marker.color.b = 1.0;
        }
        else if (point.color == 1)
        {
            cone_marker.color.r = 1.0;
            cone_marker.color.g = 1.0;
            cone_marker.color.b = 0.0;
        }
        else if (point.color == 2)
        {
            cone_marker.color.r = 1.0;
            cone_marker.color.g = 0.65;
            cone_marker.color.b = 0.0;
        }
        else if (point.color == 3)
        {
            cone_marker.color.r = 1.0;
            cone_marker.color.g = 0.65;
            cone_marker.color.b = 0.0;
        }
        else if (point.color == 4)
        {
            cone_marker.color.r = 0.0;
            cone_marker.color.g = 0.5;
            cone_marker.color.b = 0.0;
        }
        cone_marker.color.a = 1.0;
        cone_marker.lifetime = rclcpp::Duration::from_seconds(0.0); // Infinite lifetime

        cone_markers.markers.push_back(cone_marker);
    }

    cone_marker_pub_->publish(cone_markers);

    // Store them to delete later
    current_cone_markers_ = cone_markers;
}

/**
 * @brief Main entry point for the simulator node.
 *
 * This initializes the ROS 2 system and starts spinning the Simulator node.
 *
 * @param argc Number of command line arguments.
 * @param argv Array of command line arguments.
 * @return int Exit status of the application.
 */
int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Simulator>());
    rclcpp::shutdown();
    return 0;
}
