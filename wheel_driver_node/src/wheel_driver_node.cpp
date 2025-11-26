#include "wheel_driver_node.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <chrono>
#include <functional>
#include <memory>
#include <string>

using namespace std::chrono_literals;

// ==================== M0602CDriver 实现 ====================
M0602CDriver::M0602CDriver(const std::string& port, uint32_t baudrate) {
    try {
        ser_.setPort(port);
        ser_.setBaudrate(baudrate);
        serial::Timeout timeout = serial::Timeout(
                serial::Timeout::max(),  // 读取最大等待时间
                100,                     // 读取每字节间隔
                0,                       // 写入最大等待时间
                0,                       // 写入每字节间隔
                0                        // 多字节间隔
            );
        ser_.setTimeout(timeout);
        ser_.setBytesize(serial::eightbits);
        ser_.setParity(serial::parity_none);
        ser_.setStopbits(serial::stopbits_one);
        ser_.setFlowcontrol(serial::flowcontrol_none);
        ser_.open();
    } catch (const std::exception& e) {
        throw std::runtime_error("无法打开串口: " + std::string(e.what()));
    }
}

M0602CDriver::~M0602CDriver() {
    if (ser_.isOpen()) {
        ser_.close();
    }
}

uint8_t M0602CDriver::crc8(const std::vector<uint8_t>& data) {
    uint8_t crc = 0;
    for (const auto& byte : data) {  // 使用auto避免类型问题
        crc = CRC8_TABLE[crc ^ byte];
    }
    return crc;
}

void M0602CDriver::sendCommand(const std::vector<uint8_t>& tx) {
    //     std::cout << "发送数据包: ";
    // for (size_t i = 0; i < tx.size(); ++i) {
    //     std::cout << std::hex << std::setw(2) << std::setfill('0') 
    //               << static_cast<int>(tx[i]) << " ";
    // }
    // std::cout << std::dec << std::endl;
    std::lock_guard<std::recursive_mutex> emergency_lock(emergency_mutex_);
    if (emergency_state_) {
        throw std::runtime_error("急停激活中，命令被拒绝");
    }
    
    std::lock_guard<std::mutex> serial_lock(serial_mutex_);
    ser_.write(tx);
    std::this_thread::sleep_for(20ms);
}

void M0602CDriver::emergencyStop() {
    std::lock_guard<std::recursive_mutex> lock(emergency_mutex_);
    emergency_state_ = true;
    
    for (uint8_t motor_id : {1, 2}) {
        std::vector<uint8_t> tx = {
            motor_id,
            0x64,
            0x00, 0x00,
            0x00, 0x00,
            0x01,
            0xFF,
            0x00,
            0
        };
        std::vector<uint8_t> data_for_crc(tx.begin(), tx.begin() + 9);  // 只取前9字节
        tx[9] = crc8(data_for_crc);  // ✅ 正确：只计算前9字节
        ser_.write(tx);
    }
    std::this_thread::sleep_for(50ms);
}

void M0602CDriver::clearEmergency() {
    std::lock_guard<std::recursive_mutex> lock(emergency_mutex_);
    emergency_state_ = false;
}

void M0602CDriver::controlMotor(uint8_t motor_id, int16_t value, MotorMode mode, 
                               uint8_t accel_time, bool brake) {
    if (emergency_state_ || brake_state_) {
        throw std::runtime_error("刹车激活中，电机控制被阻止");
    }
    sendMotorCommand(motor_id, value, accel_time, brake);
}

void M0602CDriver::controlDualMotors(double left_speed, double right_speed, 
                                    uint8_t accel_time, bool brake) {
    if (brake_state_ || emergency_state_) {
        forceControlMotor(1, 0, MotorMode::SPEED_LOOP, 1, true);
        forceControlMotor(2, 0, MotorMode::SPEED_LOOP, 1, true);
    } else {
        int16_t left_rpm = static_cast<int16_t>(left_speed / (0.1007 * M_PI) * 60.0);
        int16_t right_rpm = static_cast<int16_t>(right_speed / (0.1007 * M_PI) * 60.0);
        controlMotor(1, left_rpm, MotorMode::SPEED_LOOP, accel_time, brake);
        controlMotor(2, right_rpm, MotorMode::SPEED_LOOP, accel_time, brake);
    }
}

void M0602CDriver::forceControlMotor(uint8_t motor_id, int16_t value, MotorMode mode, 
                                   uint8_t accel_time, bool brake) {
    sendMotorCommand(motor_id, value, accel_time, brake);
}

void M0602CDriver::sendMotorCommand(uint8_t motor_id, int16_t value, uint8_t accel_time, bool brake) {
    std::vector<uint8_t> tx = {
        motor_id,
        0x64,
        static_cast<uint8_t>((value >> 8) & 0xFF),
        static_cast<uint8_t>(value & 0xFF),
        0, 0,
        accel_time,
        brake ? 0xFF : 0,
        0,
        0
    };
    std::vector<uint8_t> data_for_crc(tx.begin(), tx.begin() + 9);  // 只取前9字节
    tx[9] = crc8(data_for_crc);  // ✅ 正确：只计算前9字节
    sendCommand(tx);
    // RCLCPP_INFO(rclcpp::get_logger("M0602CDriver"), "brake=%s", brake ? "true" : "false");
}

void M0602CDriver::setMotorMode(uint8_t motor_id, MotorMode mode) {
    std::vector<uint8_t> tx = {
        motor_id, 0xA0, 0, 0, 0, 0, 0, 0, 0, static_cast<uint8_t>(mode)
    };
    sendCommand(tx);
}

M0602CDriver::MotorStatus M0602CDriver::getMotorState(uint8_t motor_id) {
    std::vector<uint8_t> tx = {motor_id, 0x74, 0, 0, 0, 0, 0, 0, 0, 0};
    std::vector<uint8_t> data_for_crc(tx.begin(), tx.begin() + 9);  // 只取前9字节
    tx[9] = crc8(data_for_crc);  // ✅ 正确：只计算前9字节
    // std::cout << "\n🔍 查询电机状态命令分析:" << std::endl;
    // std::cout << "数据包长度: " << tx.size() << " 字节" << std::endl;
    // std::cout << "十六进制: ";
    // for (size_t i = 0; i < tx.size(); ++i) {
    //     printf("%02X ", tx[i]);
    // }
    std::cout << std::endl;
    std::lock_guard<std::mutex> lock(serial_mutex_);
    ser_.flushInput();
    ser_.write(tx);


    std::vector<uint8_t> rx(10);
    size_t bytes_read = ser_.read(rx.data(), rx.size());  // ✅ 正确
    // 打印读取的字节数和数据
    // std::cout << "读取到 " << bytes_read << " 字节: ";
    // for (size_t i = 0; i < bytes_read; ++i) {
    //     std::cout << std::hex << std::setw(2) << std::setfill('0') 
    //             << static_cast<int>(rx[i]) << " ";
    // }
    // std::cout << std::dec << std::endl;
    MotorStatus status;
    if (bytes_read != 10) {
        return status;
    }
    
    if (rx[9] != crc8(std::vector<uint8_t>(rx.begin(), rx.begin() + 9))) {
        return status;  // CRC校验失败
    }
    
    status.id = rx[0];
    status.mode = rx[1];
    status.current = static_cast<int16_t>((rx[2] << 8) | rx[3]);
    status.speed = static_cast<int16_t>((rx[4] << 8) | rx[5]);
    status.temp = rx[6];
    status.position = rx[7];
    status.error = rx[8];
    status.valid = true;
    
    return status;
}

// ==================== M0602CNode 实现 ====================
M0602CNode::M0602CNode() : Node("m0602c_driver") {
    driver_ = std::make_unique<M0602CDriver>();
    driver_->setMotorMode(1, M0602CDriver::MotorMode::SPEED_LOOP);
    driver_->setMotorMode(2, M0602CDriver::MotorMode::SPEED_LOOP);
    
    x_ = y_ = th_ = vx_ = vy_ = vth_ = 0.0;
    last_time_ = this->now();
    
    initPublishers();
    initSubscribers();
    initServices();
    initBrakeService();
    
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    
    RCLCPP_INFO(this->get_logger(), "M0602C电机驱动节点已启动");
}

M0602CNode::~M0602CNode() {
    try {
        RCLCPP_WARN(this->get_logger(), "节点关闭中...发送停止指令");
        driver_->forceControlMotor(1, 0, M0602CDriver::MotorMode::SPEED_LOOP, 1, true);
        driver_->forceControlMotor(2, 0, M0602CDriver::MotorMode::SPEED_LOOP, 1, true);
        std::this_thread::sleep_for(50ms);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "关闭时停止电机失败: %s", e.what());
    }
}

void M0602CNode::initPublishers() {
    status_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("motor_status", 10);
    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", 10);
    publish_timer_ = this->create_wall_timer(10ms, std::bind(&M0602CNode::publishAll, this));
}

void M0602CNode::initSubscribers() {
    cmd_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "/cmd_vel", 10, std::bind(&M0602CNode::cmdVelCallback, this, std::placeholders::_1));
}

void M0602CNode::initServices() {
    emergency_stop_srv_ = this->create_service<std_srvs::srv::Trigger>(
        "emergency_stop", std::bind(&M0602CNode::handleEmergencyStop, this, 
                                   std::placeholders::_1, std::placeholders::_2));
    
    emergency_reset_srv_ = this->create_service<std_srvs::srv::Trigger>(
        "emergency_reset", std::bind(&M0602CNode::handleEmergencyReset, this, 
                                    std::placeholders::_1, std::placeholders::_2));
}

void M0602CNode::initBrakeService() {
    brake_srv_ = this->create_service<std_srvs::srv::SetBool>(
        "wheel_brake", std::bind(&M0602CNode::handleBrakeRequest, this, 
                                std::placeholders::_1, std::placeholders::_2));
    RCLCPP_INFO(this->get_logger(), "刹车服务已初始化: /wheel_brake");
}

void M0602CNode::handleBrakeRequest(const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
                                   std::shared_ptr<std_srvs::srv::SetBool::Response> response) {
    try {
        if (request->data) {
            driver_->setBrakeState(true);
            driver_->forceControlMotor(1, 0, M0602CDriver::MotorMode::SPEED_LOOP, 1, true);
            driver_->forceControlMotor(2, 0, M0602CDriver::MotorMode::SPEED_LOOP, 1, true);
            response->success = true;
            response->message = "刹车已激活";
            RCLCPP_WARN(this->get_logger(), "刹车已激活，禁止/cmd_vel 控制");
        } else {
            driver_->setBrakeState(false);
            driver_->forceControlMotor(1, 0, M0602CDriver::MotorMode::SPEED_LOOP, 1, false);
            driver_->forceControlMotor(2, 0, M0602CDriver::MotorMode::SPEED_LOOP, 1, false);
            response->success = true;
            response->message = "刹车已释放";
            RCLCPP_INFO(this->get_logger(), "刹车已释放");
        }
    } catch (const std::exception& e) {
        response->success = false;
        response->message = "刹车操作失败: " + std::string(e.what());
        RCLCPP_ERROR(this->get_logger(), "%s", response->message.c_str());
    }
}

void M0602CNode::publishAll() {
    publishMotorStatus();
    publishOdometry();
}

void M0602CNode::publishMotorStatus() {
    auto msg = std::make_unique<sensor_msgs::msg::JointState>();
    msg->header.stamp = this->now();
    msg->name = {"left_wheel", "right_wheel"};
    
    auto left = driver_->getMotorState(1);
    auto right = driver_->getMotorState(2);
    
    double left_speed_mps = static_cast<double>(left.speed) * M_PI * WHEEL_DIAMETER_ / 60.0;
    double right_speed_mps = static_cast<double>(-right.speed) * M_PI * WHEEL_DIAMETER_ / 60.0;
    
    msg->position = {
        static_cast<double>(left.position) / 32767.0 * 360.0,
        static_cast<double>(right.position) / 32767.0 * 360.0
    };
    msg->velocity = {left_speed_mps, right_speed_mps};
    msg->effort = {
        static_cast<double>(left.current) / 32767.0 * 8.0,
        static_cast<double>(right.current) / 32767.0 * 8.0
    };
    
    vx_ = (left_speed_mps + right_speed_mps) / 2.0;
    vth_ = (right_speed_mps - left_speed_mps) / WHEEL_SPACE_;
    
    std::string status_header;
    if (driver_->isEmergency()) {
        status_header += "EMERGENCY_STOP";
    }
    if (left.error != 0) {
        if (!status_header.empty()) status_header += "|";
        status_header += "left_fault:0x" + std::to_string(left.error);
    }
    if (right.error != 0) {
        if (!status_header.empty()) status_header += "|";
        status_header += "right_fault:0x" + std::to_string(right.error);
    }
    
    msg->header.frame_id = status_header.empty() ? "normal" : status_header;
    status_pub_->publish(std::move(msg));
}

std::pair<double, double> M0602CNode::calculateWheelSpeeds(double linear, double angular) {
    if (std::abs(linear) < 1e-3) {
        return {-angular * WHEEL_SPACE_ / 2.0, angular * WHEEL_SPACE_ / 2.0};
    } else if (std::abs(angular) < 1e-3) {
        return {linear, linear};
    } else {
        return {linear - angular * WHEEL_SPACE_ / 2.0, linear + angular * WHEEL_SPACE_ / 2.0};
    }
}

void M0602CNode::cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
    if (driver_->isBrakeActive()) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "命令被拒绝：刹车状态中");
        return;
    }
    
    auto [left, right] = calculateWheelSpeeds(msg->linear.x, msg->angular.z);
    driver_->controlDualMotors(left, -right);
    
    // RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 100, 
    //                     "速度指令: 左=%.3f m/s, 右=%.3f m/s", left, right);
}

void M0602CNode::publishOdometry() {
    try {
        auto current_time = this->now();
        double dt = (current_time - last_time_).nanoseconds()/1e9;
        // std::cout << "当前时间: " << current_time.seconds() << "秒" << std::endl;
        // std::cout << "dt_ns = " << dt << std::endl; 
        // 检查时间有效性
        // if (dt <= 0 || dt > 1.0) {  // 时间差太大可能是异常
        //     last_time_ = current_time;
        //     return;
        // }
        
        // 积分计算位置
        double delta_x = vx_ * std::cos(th_) * dt;
        double delta_y = vx_ * std::sin(th_) * dt;
        double delta_th = vth_ * dt;
        std::cout << "vx_ = " << vx_ << std::endl; 
        x_ += delta_x;
        y_ += delta_y;
        th_ += delta_th;
        
        // 规范化角度
        while (th_ > M_PI) th_ -= 2.0 * M_PI;
        while (th_ < -M_PI) th_ += 2.0 * M_PI;
        
        last_time_ = current_time;
        
        // 发布TF变换
        geometry_msgs::msg::TransformStamped odom_trans;
        odom_trans.header.stamp = current_time;
        odom_trans.header.frame_id = "odom";
        odom_trans.child_frame_id = "base_footprint";
        
        odom_trans.transform.translation.x = x_;
        odom_trans.transform.translation.y = y_;
        odom_trans.transform.translation.z = 0.0;
        
        tf2::Quaternion q;
        q.setRPY(0, 0, th_);
        odom_trans.transform.rotation.x = q.x();
        odom_trans.transform.rotation.y = q.y();
        odom_trans.transform.rotation.z = q.z();
        odom_trans.transform.rotation.w = q.w();
        
        // 发送TF
        tf_broadcaster_->sendTransform(odom_trans);
        
        // 发布里程计消息
        auto odom_msg = std::make_unique<nav_msgs::msg::Odometry>();
        odom_msg->header.stamp = current_time;
        odom_msg->header.frame_id = "odom";
        odom_msg->child_frame_id = "base_footprint";
        
        // 位置
        odom_msg->pose.pose.position.x = x_;
        odom_msg->pose.pose.position.y = y_;
        odom_msg->pose.pose.position.z = 0.0;
        odom_msg->pose.pose.orientation.x = q.x();
        odom_msg->pose.pose.orientation.y = q.y();
        odom_msg->pose.pose.orientation.z = q.z();
        odom_msg->pose.pose.orientation.w = q.w();
        
        // 速度
        odom_msg->twist.twist.linear.x = vx_;
        odom_msg->twist.twist.linear.y = 0.0;
        odom_msg->twist.twist.linear.z = 0.0;
        odom_msg->twist.twist.angular.x = 0.0;
        odom_msg->twist.twist.angular.y = 0.0;
        odom_msg->twist.twist.angular.z = vth_;
        
        // 发布里程计
        odom_pub_->publish(std::move(odom_msg));
        
        // 调试输出
        static int odom_count = 0;
        if (odom_count++ % 50 == 0) {  // 每50次发布打印一次
            RCLCPP_INFO(this->get_logger(), 
                       "里程计发布: 位置(%.2f, %.2f, %.2f) 速度(%.2f, %.2f)", 
                       x_, y_, th_, vx_, vth_);
        }
        
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "里程计发布异常: %s", e.what());
    }
}
void M0602CNode::handleEmergencyStop(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                                    std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
    try {
        RCLCPP_FATAL(this->get_logger(), "!!! 紧急停止触发 !!!");
        driver_->emergencyStop();
        
        auto msg = std::make_unique<sensor_msgs::msg::JointState>();
        msg->header.stamp = this->now();
        msg->header.frame_id = "EMERGENCY_STOP";
        msg->name = {"left_wheel", "right_wheel"};
        msg->velocity = {0.0, 0.0};
        msg->effort = {0.0, 0.0};
        status_pub_->publish(std::move(msg));
        
        response->success = true;
        response->message = "急停已激活";
    } catch (const std::exception& e) {
        response->success = false;
        response->message = std::string("急停失败: ") + e.what();
        RCLCPP_ERROR(this->get_logger(), "%s", response->message.c_str());
    }
}

void M0602CNode::handleEmergencyReset(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                                     std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
    try {
        driver_->clearEmergency();
        RCLCPP_WARN(this->get_logger(), "急停状态已复位");
        response->success = true;
        response->message = "急停已解除";
    } catch (const std::exception& e) {
        response->success = false;
        response->message = std::string("急停复位失败: ") + e.what();
    }
}

// ==================== 主函数 ====================
int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    
    auto node = std::make_shared<M0602CNode>();
    
    // 设置信号处理
    std::signal(SIGINT, [](int) {
        RCLCPP_FATAL(rclcpp::get_logger("main"), "收到终止信号！执行紧急停止");
        rclcpp::shutdown();
    });
    
    try {
        rclcpp::spin(node);
    } catch (const std::exception& e) {
        RCLCPP_FATAL(rclcpp::get_logger("main"), "异常退出: %s", e.what());
    }
    
    rclcpp::shutdown();
    return 0;
}