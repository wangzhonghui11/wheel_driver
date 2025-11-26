#ifndef M0602C_DRIVER_HPP
#define M0602C_DRIVER_HPP

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <serial/serial.h>
#include <thread>
#include <mutex>
#include <atomic>
#include <memory>
#include <array>

class M0602CDriver {
public:
    enum class MotorMode {
        CURRENT_LOOP = 1,
        SPEED_LOOP = 2,
        POSITION_LOOP = 3
    };

    struct MotorStatus {
        uint8_t id = 0;
        uint8_t mode = 0;
        int16_t current = 0;
        int16_t speed = 0;
        uint8_t temp = 0;
        uint8_t position = 0;
        uint8_t error = 0;
        bool valid = false;
    };

    M0602CDriver(const std::string& port = "/dev/ttyUSB0", uint32_t baudrate = 115200);
    ~M0602CDriver();

    void emergencyStop();
    void clearEmergency();
    void controlMotor(uint8_t motor_id, int16_t value, MotorMode mode = MotorMode::SPEED_LOOP, 
                     uint8_t accel_time = 1, bool brake = false);
    void controlDualMotors(double left_speed, double right_speed, uint8_t accel_time = 4, bool brake = false);
    void forceControlMotor(uint8_t motor_id, int16_t value, MotorMode mode = MotorMode::SPEED_LOOP, 
                          uint8_t accel_time = 1, bool brake = false);
    void setMotorMode(uint8_t motor_id, MotorMode mode);
    MotorStatus getMotorState(uint8_t motor_id);

    // 状态访问方法
    bool isEmergency() const { return emergency_state_; }
    bool isBrakeActive() const { return brake_state_; }
    void setBrakeState(bool state) { brake_state_ = state; }

private:
    serial::Serial ser_;
    std::mutex serial_mutex_;
    std::recursive_mutex emergency_mutex_;
    
    std::atomic<bool> emergency_state_{false};
    std::atomic<bool> brake_state_{false};
    
    std::array<int, 2> comm_failure_counts_{0, 0};
    static constexpr int MAX_FAILURES = 50;
    std::array<bool, 2> comm_lost_{false, false};

    // CRC8表
    static constexpr std::array<uint8_t, 256> CRC8_TABLE = {
        0,94,188,226,97,63,221,131,194,156,126,32,163,253,31,65,
        157,195,33,127,252,162,64,30,95,1,227,189,62,96,130,220,
        35,125,159,193,66,28,254,160,225,191,93,3,128,222,60,98,
        190,224,2,92,223,129,99,61,124,34,192,158,29,67,161,255,
        70,24,250,164,39,121,155,197,132,218,56,102,229,187,89,7,
        219,133,103,57,186,228,6,88,25,71,165,251,120,38,196,154,
        101,59,217,135,4,90,184,230,167,249,27,69,198,152,122,36,
        248,166,68,26,153,199,37,123,58,100,134,216,91,5,231,185,
        140,210,48,110,237,179,81,15,78,16,242,172,47,113,147,205,
        17,79,173,243,112,46,204,146,211,141,111,49,178,236,14,80,
        175,241,19,77,206,144,114,44,109,51,209,143,12,82,176,238,
        50,108,142,208,83,13,239,177,240,174,76,18,145,207,45,115,
        202,148,118,40,171,245,23,73,8,86,180,234,105,55,213,139,
        87,9,235,181,54,104,138,212,149,203,41,119,244,170,72,22,
        233,183,85,11,136,214,52,106,43,117,151,201,74,20,246,168,
        116,42,200,150,21,75,169,247,182,232,10,84,215,137,107,53
    };

    uint8_t crc8(const std::vector<uint8_t>& data);
    void sendCommand(const std::vector<uint8_t>& tx);
    void sendMotorCommand(uint8_t motor_id, int16_t value, uint8_t accel_time, bool brake);
};

class M0602CNode : public rclcpp::Node {
public:
    M0602CNode();
    ~M0602CNode();

private:
    std::unique_ptr<M0602CDriver> driver_;
    
    // ROS2 组件
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr status_pub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr emergency_stop_srv_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr emergency_reset_srv_;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr brake_srv_;
    
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    
    rclcpp::TimerBase::SharedPtr publish_timer_;
    
    // 里程计状态
    double x_ = 0.0, y_ = 0.0, th_ = 0.0;
    double vx_ = 0.0, vy_ = 0.0, vth_ = 0.0;
    rclcpp::Time last_time_;
    
    // 参数
    const double WHEEL_SPACE_ = 0.26;
    const double WHEEL_DIAMETER_ = 0.1007;
    
    void initPublishers();
    void initSubscribers();
    void initServices();
    void initBrakeService();
    
    void publishAll();
    void publishMotorStatus();
    void publishOdometry();
    
    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
    
    void handleEmergencyStop(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                            std::shared_ptr<std_srvs::srv::Trigger::Response> response);
    void handleEmergencyReset(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                            std::shared_ptr<std_srvs::srv::Trigger::Response> response);
    void handleBrakeRequest(const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
                          std::shared_ptr<std_srvs::srv::SetBool::Response> response);
    
    std::pair<double, double> calculateWheelSpeeds(double linear, double angular);
};

#endif // M0602C_DRIVER_HPP