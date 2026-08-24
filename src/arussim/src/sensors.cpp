/**
 * @file sensors.cpp
 * @author Rafael Guil (rafaguilvalero@gmail.com)
 * @brief Sensors node for the ARUSSIM package. This node simulates the sensors of the vehicle.
 * @version 0.1
 * @date 2024-10-16
 * 
 */
#include "arussim/sensors.hpp"
#include <cerrno>
#include <cstring>
#include <unistd.h>
std::vector<Sensors::CanFrame> Sensors::frames;

/**
 * @class Sensors
 * @brief Sensors class for ARUSsim
 * This class simulates some sensors of the vehicle like the IMU, inverters and extensometer.
 */
Sensors::Sensors() : Node("sensors")
{
    // Declare ground speed parameters
    this->declare_parameter<double>("gss.noise_gss_vx", 0.01);
    this->declare_parameter<double>("gss.noise_gss_vy", 0.01);
    this->declare_parameter<double>("gss.noise_gss_ax", 0.01);
    this->declare_parameter<double>("gss.noise_gss_ay", 0.01);
    this->declare_parameter<double>("gss.noise_gss_r", 0.01);
    this->declare_parameter<double>("gss.gss_frequency", 200.0);
    
    // Declare and get noise parameters for each IMU variable
    this->declare_parameter<double>("imu.noise_imu_ax", 0.01);
    this->declare_parameter<double>("imu.noise_imu_ay", 0.01);
    this->declare_parameter<double>("imu.noise_imu_r", 0.01);
    this->declare_parameter<double>("imu.imu_frequency", 200.0);
    this->declare_parameter<double>("imu.noise_imu_pose", 0.1);

    // Declare wheel speed parameters
    this->declare_parameter<double>("inverter.noise_motor_speed_front_right", 0.01);
    this->declare_parameter<double>("inverter.noise_motor_speed_front_left", 0.01);
    this->declare_parameter<double>("inverter.noise_motor_speed_rear_right", 0.01);
    this->declare_parameter<double>("inverter.noise_motor_speed_rear_left", 0.01);
    this->declare_parameter<double>("inverter.noise_torque_front_right", 0.01);
    this->declare_parameter<double>("inverter.noise_torque_front_left", 0.01);
    this->declare_parameter<double>("inverter.noise_torque_rear_right", 0.01);
    this->declare_parameter<double>("inverter.noise_torque_rear_left", 0.01);
    this->declare_parameter<double>("inverter.inverter_frequency", 200.0);

    // Declare extensometer parameters
    this->declare_parameter<double>("extensometer.extensometer_frequency", 100.0);
    this->declare_parameter<double>("extensometer.noise_extensometer", 0.01);

    // Declare BMS parameters
    this->declare_parameter<double>("bms.bms_frequency", 100.0);
    this->declare_parameter<double>("bms.noise_battery_voltage", 0.1);

    // Declare AS PCB parameters
    this->declare_parameter<double>("as.as_frequency", 10.0);

    // Get parameters
    this->get_parameter("gss.noise_gss_ax", kNoiseGssAx);
    this->get_parameter("gss.noise_gss_ay", kNoiseGssAy);
    this->get_parameter("gss.noise_gss_r", kNoiseGssR);
    this->get_parameter("gss.noise_gss_vx", kNoiseGssVx);
    this->get_parameter("gss.noise_gss_vy", kNoiseGssVy);
    this->get_parameter("gss.gss_frequency", kGssFrequency);

    this->get_parameter("imu.noise_imu_ax", kNoiseImuAx);
    this->get_parameter("imu.noise_imu_ay", kNoiseImuAy);
    this->get_parameter("imu.noise_imu_r", kNoiseImuR);
    this->get_parameter("imu.imu_frequency", kImuFrequency);
    this->get_parameter("imu.noise_imu_pose", kNoiseImuPose);

    this->get_parameter("inverter.noise_motor_speed_front_right", kNoiseMotorSpeedFrontRight);
    this->get_parameter("inverter.noise_motor_speed_front_left", kNoiseMotorSpeedFrontLeft);
    this->get_parameter("inverter.noise_motor_speed_rear_right", kNoiseMotorSpeedRearRight);
    this->get_parameter("inverter.noise_motor_speed_rear_left", kNoiseMotorSpeedRearLeft);
    this->get_parameter("inverter.noise_torque_front_right", kNoiseTorqueFrontRight);
    this->get_parameter("inverter.noise_torque_front_left", kNoiseTorqueFrontLeft);
    this->get_parameter("inverter.noise_torque_rear_right", kNoiseTorqueRearRight);
    this->get_parameter("inverter.noise_torque_rear_left", kNoiseTorqueRearLeft);
    this->get_parameter("inverter.inverter_frequency", kInverterFrequency);

    this->get_parameter("extensometer.extensometer_frequency", kExtensometerFrequency);
    this->get_parameter("extensometer.noise_extensometer", kNoiseExtensometer);

    this->get_parameter("bms.bms_frequency", kBMSFrequency);
    this->get_parameter("bms.noise_battery_voltage", kNoiseBatteryVoltage);

    this->get_parameter("as.as_frequency", kASFrequency);
    
    launch_sub_ = this->create_subscription<std_msgs::msg::Bool>("/arussim/launch", 1,
    std::bind(&Sensors::launch_callback, this, std::placeholders::_1));

    reset_sub_ = this->create_subscription<std_msgs::msg::Bool>("/arussim/reset", 1, 
    std::bind(&Sensors::reset_callback, this, std::placeholders::_1));

    //CAN Socket setup
    can0_socket_ = setup_can_socket("can0");
    can1_socket_ = setup_can_socket("can1");
    can2_socket_ = setup_can_socket("can2");

    // State subscriber
    state_sub_ = this->create_subscription<arussim_msgs::msg::State>(
        "/arussim/state", 1, 
        std::bind(&Sensors::state_callback, this, std::placeholders::_1)
    );

    // Groundspeed
    gss_vx_pub_ = this->create_publisher<std_msgs::msg::Float32>(
        "/arussim/gss/vx", 10);
    gss_vy_pub_ = this->create_publisher<std_msgs::msg::Float32>(
        "/arussim/gss/vy", 10);

    gss_timer_ = this->create_wall_timer(
        std::chrono::milliseconds((int)(1000/kGssFrequency)),
        std::bind(&Sensors::groundspeed_timer, this)
    );

    // IMU
    ax_pub_ = this->create_publisher<std_msgs::msg::Float32>("/arussim/IMU/ax", 10);
    ay_pub_ = this->create_publisher<std_msgs::msg::Float32>("/arussim/IMU/ay", 10);
    r_pub_ = this->create_publisher<std_msgs::msg::Float32>("/arussim/IMU/yaw_rate", 10);
    x_pub_ = this->create_publisher<std_msgs::msg::Float32>("/arussim/IMU/x", 10);
    y_pub_ = this->create_publisher<std_msgs::msg::Float32>("/arussim/IMU/y", 10);

    imu_timer_ = this->create_wall_timer(
        std::chrono::milliseconds((int)(1000/kImuFrequency)),
        std::bind(&Sensors::imu_timer, this)
    );

    // Inverter
    motor_speed_pub_ = this->create_publisher<arussim_msgs::msg::FourWheelDrive>(
        "/arussim/motor_speed", 10);
    torque_pub_ = this->create_publisher<arussim_msgs::msg::FourWheelDrive>(
        "/arussim/torque4WD", 10);
        
    inv_timer_ = this->create_wall_timer(
        std::chrono::milliseconds((int)(1000/kInverterFrequency)),
        std::bind(&Sensors::inverter_timer, this)
    );

    // Extensometer
    ext_pub_ = this->create_publisher<std_msgs::msg::Float32>(
        "/arussim/extensometer", 10);

    ext_timer_ = this->create_wall_timer(
        std::chrono::milliseconds((int)(1000/kExtensometerFrequency)),
        std::bind(&Sensors::extensometer_timer, this)
    );

    // BMS
    bms_timer_ = this->create_wall_timer(
        std::chrono::milliseconds((int)(1000/kBMSFrequency)),
        std::bind(&Sensors::bms_timer, this)
    );

    // AS PCB
    as_timer_ = this->create_wall_timer(
        std::chrono::milliseconds((int) (1000/kASFrequency)),
        std::bind(&Sensors::as_timer, this)
    );

    frames = {
        {0x1A0, 4, {  // GSS speeds
            {"GSS/vx", {0, 15, true, 0.02/3.6, 0.0}},
            {"GSS/vy", {16, 31, true, 0.02/3.6, 0.0}}
            }, Sensors::CanBus::kCan1
        },
        {0x1A3, 4, { // GSS accelerations
            {"GSS/ax", {0, 15, true, 0.02, 0.0}},
            {"GSS/ay", {16, 31, true, 0.02, 0.0}},
            }, Sensors::CanBus::kCan1
        },
        {0x1A4, 6, { // GSS rates
            {"GSS/yaw_rate", {32, 47, true, 0.02*M_PI/180, 0.0}}
            }, Sensors::CanBus::kCan1
        },
        {0x1B0, 6, { // IMU accelerations (Control-RaspPi reads it on can2)
            {"IMU/ax", {0, 15, true, 0.01, 0.0}},
            {"IMU/ay", {16, 31, true, 0.01, 0.0}},
            }, Sensors::CanBus::kCan2
        },
        {0x1B1, 6, { // IMU rates (Control-RaspPi decodes rad/s with factor 0.001)
            {"IMU/yaw_rate", {32, 47, true, 0.001, 0.0}}
            }, Sensors::CanBus::kCan2
        },
        {0x540, 8, { // Doppla GSS speeds (Control-RaspPi reads it on can2, m/s x0.01)
            {"Doppla/vx", {32, 47, true, 0.01, 0.0}},
            {"Doppla/vy", {48, 63, true, 0.01, 0.0}}
            }, Sensors::CanBus::kCan2
        },
        {0x54A, 1, { // Doppla GSS status (bit 0 of data0 = 1 -> GSS OK)
            {"Doppla/status", {0, 7, false, 1.0, 0.0}}
            }, Sensors::CanBus::kCan2
        },
        {0x134, 2, { // Extensometer
            {"extensometer", {0, 15, true, 0.000035, -0.482442}}
            }, Sensors::CanBus::kCan0
        },
        {0x123, 6, {
            {"fl_inv_speed", {0, 31, true, 0.01, 0.0}}, // Front Left inverter
            {"fr_inv_speed", {32, 63, true, 0.01, 0.0}}, // Front Right inverter
            {"rl_inv_speed", {64, 95, true, 0.01, 0.0}}, // Rear Left inverter
            {"rr_inv_speed", {96, 127, true, 0.01, 0.0}}, // Rear Right inverter
            }, Sensors::CanBus::kCan2
        },
        // AMK inverters actual values: motor speed (raw x0.0001 = rpm) and
        // motor torque (raw x0.0098 = Nm). Control-RaspPi reads them on can1.
        {0x102, 8, { // Motor 0: Front Left
            {"motor_speed", {0, 31, true, 0.0001 * 2 * M_PI / 60, 0.0}},
            {"motor_torque", {32, 47, true, 0.1 * 9.8 / 100, 0.0}}
            }, Sensors::CanBus::kCan1
        },
        {0x106, 8, { // Motor 1: Front Right
            {"motor_speed", {0, 31, true, 0.0001 * 2 * M_PI / 60, 0.0}},
            {"motor_torque", {32, 47, true, 0.1 * 9.8 / 100, 0.0}}
            }, Sensors::CanBus::kCan1
        },
        {0x110, 8, { // Motor 2: Rear Left
            {"motor_speed", {0, 31, true, 0.0001 * 2 * M_PI / 60, 0.0}},
            {"motor_torque", {32, 47, true, 0.1 * 9.8 / 100, 0.0}}
            }, Sensors::CanBus::kCan1
        },
        {0x114, 8, { // Motor 3: Rear Right
            {"motor_speed", {0, 31, true, 0.0001 * 2 * M_PI / 60, 0.0}},
            {"motor_torque", {32, 47, true, 0.1 * 9.8 / 100, 0.0}}
            }, Sensors::CanBus::kCan1
        },
        {0x100, 2, {
            {"enable_amk_status_byte1", {8, 15, false, 1.0, 0.0}}
            }, Sensors::CanBus::kCan1
        },
        {0x104, 2, {
            {"enable_amk_status_byte1", {8, 15, false, 1.0, 0.0}}
            }, Sensors::CanBus::kCan1
        },
        {0x108, 2, {
            {"enable_amk_status_byte1", {8, 15, false, 1.0, 0.0}}
            }, Sensors::CanBus::kCan1
        },
        {0x112, 2, {
            {"enable_amk_status_byte1", {8, 15, false, 1.0, 0.0}}
            }, Sensors::CanBus::kCan1
        },
        {0x221, 1, {
            {"enable_flag", {0, 7, false, 1.0, 0.0}}
            }, Sensors::CanBus::kCan0
        },
        {0x192, 6, {
            {"battery_voltage", {16, 47, true, 0.001, 0.0}}
            }, Sensors::CanBus::kCan0
        }
    };
}


int Sensors::setup_can_socket(const char * interface_name)
{
    const int can_socket = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (can_socket < 0) {
        RCLCPP_ERROR(this->get_logger(), "socket(PF_CAN) failed for %s: %s", interface_name, std::strerror(errno));
        return -1;
    }

    struct ifreq ifr = {};
    std::strncpy(ifr.ifr_name, interface_name, IFNAMSIZ - 1);
    ifr.ifr_name[IFNAMSIZ - 1] = '\0';

    if (ioctl(can_socket, SIOCGIFINDEX, &ifr) < 0) {
        RCLCPP_ERROR(this->get_logger(), "ioctl(SIOCGIFINDEX) failed for %s: %s", interface_name, std::strerror(errno));
        close(can_socket);
        return -1;
    }

    struct sockaddr_can addr = {};
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    if (bind(can_socket, reinterpret_cast<struct sockaddr *>(&addr), sizeof(addr)) < 0) {
        RCLCPP_ERROR(this->get_logger(), "bind() failed for %s: %s", interface_name, std::strerror(errno));
        close(can_socket);
        return -1;
    }

    return can_socket;
}


uint64_t Sensors::encode_signal(double value, double scale, double offset, int bit_len, bool is_signed) {
    double raw_f = (value - offset) / scale;
    int64_t raw_i = static_cast<int64_t>(std::llround(raw_f));

    if (is_signed) {
        int64_t min_val = -(1LL << (bit_len - 1));
        int64_t max_val =  (1LL << (bit_len - 1)) - 1;
        raw_i = std::clamp(raw_i, min_val, max_val);
        return static_cast<uint64_t>(raw_i) &
               ((1ULL << bit_len) - 1);
    } else {
        int64_t min_val = static_cast<int64_t>(0);
        int64_t max_val = static_cast<int64_t>((1ULL << bit_len) - 1);
        uint64_t raw_u = static_cast<uint64_t>(std::clamp(raw_i, min_val, max_val));
        return raw_u;
    }
}


void Sensors::state_callback(const arussim_msgs::msg::State::SharedPtr msg)
{
    // Update state variables with incoming data
    x_ = msg->x;
    y_ = msg->y;
    yaw_ = msg->yaw;
    vx_ = msg->vx;
    vy_ = msg->vy;
    r_ = msg->r;
    ax_ = msg->ax;
    ay_ = msg->ay;
    delta_ = msg->delta;
    wheel_speed_msg_ = msg->wheel_speeds;
    torque_cmd_msg_ = msg->torque;
}


void Sensors::imu_timer()
{
    // Random noise generation with different noise for each variable
    std::random_device rd;
    std::mt19937 gen(rd());

    std::normal_distribution<> dist_ax(0.0, kNoiseImuAx);
    std::normal_distribution<> dist_ay(0.0, kNoiseImuAy);
    std::normal_distribution<> dist_r(0.0, kNoiseImuR);
    std::normal_distribution<> dist_pose(0.0, kNoiseImuPose);

    // Create IMU data messages
    auto msg_ax = std_msgs::msg::Float32();
    auto msg_ay = std_msgs::msg::Float32();
    auto msg_r = std_msgs::msg::Float32();
    auto msg_x = std_msgs::msg::Float32();
    auto msg_y = std_msgs::msg::Float32();

    // Yaw rate
    msg_r.data = r_ + dist_r(gen);  

    // Linear acceleration
    msg_ax.data = ax_ + dist_ax(gen);  
    msg_ay.data = ay_ + dist_ay(gen);  

    // Vehicle pose
    msg_x.data = x_ + dist_pose(gen);
    msg_y.data = y_ + dist_pose(gen);

    // Publish the IMU message
    ax_pub_->publish(msg_ax);
    ay_pub_->publish(msg_ay);
    r_pub_->publish(msg_r);

    std::map<std::string,double> values = { {"IMU/ax", msg_ax.data}, 
    {"IMU/ay", msg_ay.data}, {"IMU/yaw_rate", msg_r.data} }; 

    for (auto &frame : frames) { 
        if (frame.id == 0x1B0 || frame.id == 0x1B1)  { 
            send_can_frame(frame, values); 
        } 
        
    }
    x_pub_->publish(msg_x);
    y_pub_->publish(msg_y);
}


void Sensors::inverter_timer()
{
    // Random noise generation with different noise for each wheel
    std::random_device rd; 
    std::mt19937 gen(rd());

    // ---------- Wheelspeed ------------
    std::normal_distribution<> dist_front_right(0.0, kNoiseMotorSpeedFrontRight);
    std::normal_distribution<> dist_front_left(0.0, kNoiseMotorSpeedFrontLeft);
    std::normal_distribution<> dist_rear_right(0.0, kNoiseMotorSpeedRearRight);
    std::normal_distribution<> dist_rear_left(0.0, kNoiseMotorSpeedRearLeft);

    // Apply noise to the state variables
    motor_speed_.fr_ = wheel_speed_msg_.front_right*kGearRatio + dist_front_right(gen);
    motor_speed_.fl_ = wheel_speed_msg_.front_left*kGearRatio + dist_front_left(gen);
    motor_speed_.rr_ = wheel_speed_msg_.rear_right*kGearRatio + dist_rear_right(gen);
    motor_speed_.rl_ = wheel_speed_msg_.rear_left*kGearRatio + dist_rear_left(gen);

    // Create the wheel speed message
    auto motor_speed_msg = arussim_msgs::msg::FourWheelDrive();

    motor_speed_msg.front_right = motor_speed_.fr_;    
    motor_speed_msg.front_left = motor_speed_.fl_;
    motor_speed_msg.rear_right = motor_speed_.rr_;
    motor_speed_msg.rear_left = motor_speed_.rl_;

    // Publish the wheel speed message
    motor_speed_pub_->publish(motor_speed_msg);

    // ---------- Torque ------------
    std::normal_distribution<> dist_fr(0.0, kNoiseTorqueFrontRight);
    std::normal_distribution<> dist_fl(0.0, kNoiseTorqueFrontLeft);
    std::normal_distribution<> dist_rr(0.0, kNoiseTorqueRearRight);
    std::normal_distribution<> dist_rl(0.0, kNoiseTorqueRearLeft);

    // Apply noise to the state variables
    torque_cmd_.fr_ = torque_cmd_msg_.front_right/kGearRatio + dist_fr(gen);
    torque_cmd_.fl_ = torque_cmd_msg_.front_left/kGearRatio + dist_fl(gen);
    torque_cmd_.rr_ = torque_cmd_msg_.rear_right/kGearRatio + dist_rr(gen);
    torque_cmd_.rl_ = torque_cmd_msg_.rear_left/kGearRatio + dist_rl(gen);

    // Create the torque message
    auto torque_msg = arussim_msgs::msg::FourWheelDrive();

    torque_msg.front_right = torque_cmd_.fr_;    
    torque_msg.front_left = torque_cmd_.fl_;      
    torque_msg.rear_right = torque_cmd_.rr_;     
    torque_msg.rear_left = torque_cmd_.rl_;     

    // Publish the torque message
    torque_pub_->publish(torque_msg);
    
    //Send Inverter CAN frames
    std::map<std::string,double> values = {
    {"fl_inv_speed", motor_speed_msg.front_left},
    {"fr_inv_speed", motor_speed_msg.front_right},
    {"rl_inv_speed", motor_speed_msg.rear_left},
    {"rr_inv_speed", motor_speed_msg.rear_right},
    {"enable_amk_status_byte1", 96.0}
    };

    // AMK actual values per motor (0 = FL, 1 = FR, 2 = RL, 3 = RR), read by Control-RaspPi on can1
    const std::array<std::pair<uint32_t, std::map<std::string,double>>, 4> amk_values = {{
        {0x102, {{"motor_speed", motor_speed_msg.front_left},  {"motor_torque", torque_msg.front_left}}},
        {0x106, {{"motor_speed", motor_speed_msg.front_right}, {"motor_torque", torque_msg.front_right}}},
        {0x110, {{"motor_speed", motor_speed_msg.rear_left},   {"motor_torque", torque_msg.rear_left}}},
        {0x114, {{"motor_speed", motor_speed_msg.rear_right},  {"motor_torque", torque_msg.rear_right}}}
    }};

    for (auto &frame : frames) {
        if (frame.id == 0x123) {
            send_can_frame(frame, values);
        }
        for (const auto &amk : amk_values) {
            if (frame.id == amk.first) {
                send_can_frame(frame, amk.second);
            }
        }
    }
}


void Sensors::groundspeed_timer()
{
    // Random noise generation
    std::random_device rd; 
    std::mt19937 gen(rd());
    std::normal_distribution<> dist_vx(0.0, kNoiseGssVx);
    std::normal_distribution<> dist_vy(0.0, kNoiseGssVy);
    std::normal_distribution<> dist_ax(0.0, kNoiseGssAx);
    std::normal_distribution<> dist_ay(0.0, kNoiseGssAy);
    std::normal_distribution<> dist_r(0.0, kNoiseGssR);

    auto msg_vx = std_msgs::msg::Float32();
    auto msg_vy = std_msgs::msg::Float32();

    msg_vx.data = vx_ + dist_vx(gen);
    msg_vy.data = vy_ + dist_vy(gen);

    gss_vx_pub_->publish(msg_vx);
    gss_vy_pub_->publish(msg_vy);
    
    std::map<std::string,double> values = { {"GSS/ax", ax_ + dist_ax(gen)},
    {"GSS/ay", ay_ + dist_ay(gen)}, {"GSS/yaw_rate", r_ + dist_r(gen)},
    {"GSS/vx", vx_ + dist_vx(gen)}, {"GSS/vy", vy_ + dist_vy(gen)},
    {"Doppla/vx", vx_ + dist_vx(gen)}, {"Doppla/vy", vy_ + dist_vy(gen)},
    {"Doppla/status", 1.0} };

    for (auto &frame : frames) {
        if (frame.id == 0x1A3 || frame.id == 0x1A4 || frame.id == 0x1A0 ||
            frame.id == 0x540 || frame.id == 0x54A)  {
            send_can_frame(frame, values);
        }

    }
}


void Sensors::extensometer_timer()
{
    // Random noise generation
    std::random_device rd; 
    std::mt19937 gen(rd());
    std::normal_distribution<> dist(0.0, kNoiseExtensometer);

    auto message = std_msgs::msg::Float32();
    message.data = delta_ + dist(gen);
    ext_pub_->publish(message);

    //Send CAN frame for extensometer
    std::map<std::string,double> values = { {"extensometer", delta_ + dist(gen)} }; 

    for (auto &frame : frames) { 
        if (frame.id == 0x134) { 
            send_can_frame(frame, values);  
        }
    }
}


void Sensors::bms_timer() {
    // Random noise generation with different noise for each variable
    std::random_device rd;
    std::mt19937 gen(rd());

    std::normal_distribution<> dist_voltage(0.0, kNoiseBatteryVoltage);
    // TODO: add battery voltage simulation
    std::map<std::string,double> values = { {"battery_voltage", 500 + dist_voltage(gen)} };

    for (auto &frame : frames) { 
        if (frame.id == 0x192)  { 
            send_can_frame(frame, values); 
        } 
    }
}


void Sensors::as_timer() {

    // 0x161 (dv_driving) is sent by the arussim node in raspi_sim mode
    std::map<std::string,double> values = { {"enable_flag", 1.0} };

    for (auto &frame : frames) {
        if (frame.id == 0x221)  {
            send_can_frame(frame, values);
        }
    }
}


void Sensors::send_can_frame(const CanFrame &frame, const std::map<std::string,double> &values) { 
    canMsg_.can_id = frame.id; 
    canMsg_.can_dlc = frame.size; 
    std::memset(canMsg_.data, 0, sizeof(canMsg_.data)); 
    
    for (const auto &sig_pair : frame.signals) {
        const auto &topic = sig_pair.first;
        const auto &sig = sig_pair.second;

        int bit_len = sig.bit_fin - sig.bit_in + 1;
        uint64_t raw = encode_signal(values.at(topic),
                                     sig.scale,
                                     sig.offset,
                                     bit_len,
                                     sig.is_signed);

        for (int i = 0; i < bit_len; ++i) {
            int bit_pos = sig.bit_in + i;
            int byte_idx = bit_pos / 8;
            int bit_offset = bit_pos % 8;

            if (raw & (1ULL << i)) {
                canMsg_.data[byte_idx] |= (1U << bit_offset);
            }
        }
    }

    if (frame.can_bus == CanBus::kCan0) {
        if (can0_socket_ < 0) {
            RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(),1000, "can0_socket_ not initialized for frame 0x%X", frame.id);
            can0_socket_ = setup_can_socket("can0");
            return;
        }
        write(can0_socket_, &canMsg_, sizeof(canMsg_)); 
        return;
    }
    
    if (frame.can_bus == CanBus::kCan1) {
        if (can1_socket_ < 0) {
            RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(),1000, "can1_socket_ not initialized for frame 0x%X", frame.id);
            can1_socket_ = setup_can_socket("can1");
            return;
        }
        write(can1_socket_, &canMsg_, sizeof(canMsg_));
        return;
    }
  
    if (frame.can_bus == CanBus::kCan2) {
        if (can2_socket_ < 0) {
            RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(),1000, "can2_socket_ not initialized for frame 0x%X", frame.id);
            can2_socket_ = setup_can_socket("can2");
            return;
        }
        write(can2_socket_, &canMsg_, sizeof(canMsg_));
        return;
    }
}

void Sensors::launch_callback(const std_msgs::msg::Bool::SharedPtr msg)
{
    
    if (msg && msg->data) 
    {
        RCLCPP_INFO(this->get_logger(), "[SENSORS] Launch recibido. Estado anterior: %f", as_status_);
        if (as_status_ == 2.0 || as_status_ == 0x02) 
        {
            as_status_ = 3.0;
            RCLCPP_INFO(this->get_logger(), "[SENSORS] Estado actualizado a 3.0");
        }
    }
}

void Sensors::reset_callback([[maybe_unused]] const std_msgs::msg::Bool::SharedPtr msg)
{
    as_status_ = 0x02;
}

/**
 * @brief Main function
 * 
 * This initializes the ROS 2 system and starts spinning the Sensor node.
 * 
 * @param argc Number of command line arguments.
 * @param argv Array of command line arguments.
 * @return int Exit status of the application. 
 */
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Sensors>());
  rclcpp::shutdown();
  return 0;
}