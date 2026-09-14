// DVL -> MAVROS external-vision node with lever-arm correction and depth fusion.
//
// What this node does:
//   * Reads a Teledyne Pathfinder DVL over serial (PD0 binary output).
//   * Converts the 4 Janus beam velocities into an instrument-frame velocity.
//   * Corrects that velocity from the DVL location to the Pixhawk location
//     using  v_pixhawk = v_dvl - omega x r_lever.
//   * Optionally folds in an absolute depth reading (from the Teensy's pressure
//     sensor, relayed as a NAMED_VALUE_FLOAT and surfaced by MAVROS on
//     /mavros/debug_value/named_value_float) as the Z position.
//   * Publishes ONE nav_msgs/Odometry on /mavros/odometry/out so the MAVROS
//     "odometry" plugin forwards it to PX4 as a single ODOMETRY (external
//     vision) message.
//
// Why merge here instead of sending depth separately: PX4 fills its vision
// estimate from one odometry stream, so two independent senders (DVL and depth)
// would overwrite each other. Combining them into one message gives the EKF a
// coherent estimate — DVL velocity for motion, depth for absolute height (which
// is what EKF2_HGT_REF = vision expects).
//
// Field validity (PX4 uses each field only when finite):
//   * velocity  : always sent (the DVL measurement).
//   * position  : (0, 0, -depth) with huge X/Y variance so only Z (depth) is
//                 trusted; NaN (ignored) until a depth reading has arrived.
//   * orientation: NaN (not fused; yaw comes from the vehicle's own IMU/mag).
//
// Frames: velocity in base_link (FLU), position in odom (ENU, so z-up = -depth).
// The node broadcasts the ENU->NED and FLU->FRD static transforms the MAVROS
// odometry plugin requires.

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <cmath>
#include <cstring>
#include <thread>
#include <mutex>
#include <atomic>
#include <limits>
#include "nav_msgs/msg/odometry.hpp"
#include "mavros_msgs/msg/debug_value.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/static_transform_broadcaster.h"
#include <Eigen/Dense>

#define BUFFER_SIZE 2048

static constexpr int16_t PD0_BAD_VELOCITY = -32768;

class DVLNode : public rclcpp::Node
{
public:
    DVLNode() : Node("dvl_node")
    {
        serial_port_ = declare_parameter<std::string>("serial_port", "/dev/ttyUSB0");
        lever_arm_[0] = declare_parameter<double>("lever_arm_x", -0.111506);
        lever_arm_[1] = declare_parameter<double>("lever_arm_y", 0.0);
        lever_arm_[2] = declare_parameter<double>("lever_arm_z", 0.0650748);
        double vel_sigma = declare_parameter<double>("velocity_sigma", 0.05);
        vel_variance_ = vel_sigma * vel_sigma;
        odom_frame_ = declare_parameter<std::string>("odom_frame", "odom");
        base_frame_ = declare_parameter<std::string>("base_frame", "base_link");

        // Depth fusion (from the Teensy pressure sensor via NAMED_VALUE_FLOAT).
        use_depth_ = declare_parameter<bool>("use_depth", true);
        depth_topic_ = declare_parameter<std::string>(
            "depth_topic", "/mavros/debug_value/named_value_float");
        depth_key_ = declare_parameter<std::string>("depth_key", "DEPTH");
        double depth_sigma = declare_parameter<double>("depth_sigma", 0.05); // m
        depth_variance_ = depth_sigma * depth_sigma;
        depth_timeout_ = declare_parameter<double>("depth_timeout", 1.0);    // s

        auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();
        imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
            "/mavros/imu/data", qos,
            std::bind(&DVLNode::imu_callback, this, std::placeholders::_1));
        mavros_odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("/mavros/odometry/out", 10);

        if (use_depth_)
        {
            depth_sub_ = create_subscription<mavros_msgs::msg::DebugValue>(
                depth_topic_, rclcpp::QoS(rclcpp::KeepLast(10)).best_effort(),
                std::bind(&DVLNode::depth_callback, this, std::placeholders::_1));
            RCLCPP_INFO(get_logger(), "Depth fusion on: '%s' key '%s'",
                        depth_topic_.c_str(), depth_key_.c_str());
        }

        broadcastStaticTransforms();
        initSerial();
        initDVL();
        reader_thread_ = std::thread(&DVLNode::readLoop, this);
    }

    ~DVLNode()
    {
        running_ = false;
        if (reader_thread_.joinable())
            reader_thread_.join();
        if (serial_fd_ >= 0)
            close(serial_fd_);
    }

private:
    int serial_fd_ = -1;
    std::string serial_port_, odom_frame_, base_frame_;
    double lever_arm_[3] = {0.0, 0.0, 0.0};
    double vel_variance_ = 0.0025;

    bool use_depth_ = true;
    std::string depth_topic_, depth_key_;
    double depth_variance_ = 0.0025;
    double depth_timeout_ = 1.0;
    std::atomic<bool> have_depth_{false};
    double latest_depth_ = 0.0;              // metres below surface (positive down)
    rclcpp::Time last_depth_time_;

    uint8_t buffer[BUFFER_SIZE]{};
    size_t buffer_index = 0;
    float beam_angle = 30.0 * M_PI / 180.0;

    struct BottomTrack
    {
        int16_t vel_beam[4]{};
        bool valid = false;
    } bt;

    std::thread reader_thread_;
    std::mutex data_mutex_;
    std::atomic<bool> running_{true};

    std::atomic<bool> imu_received_{false};
    double omega_x_ = 0.0, omega_y_ = 0.0, omega_z_ = 0.0;

    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Subscription<mavros_msgs::msg::DebugValue>::SharedPtr depth_sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr mavros_odom_pub_;

    void broadcastStaticTransforms()
    {
        static_tf_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
        std::vector<geometry_msgs::msg::TransformStamped> tfs;

        geometry_msgs::msg::TransformStamped enu_ned;
        enu_ned.header.stamp = now();
        enu_ned.header.frame_id = odom_frame_;
        enu_ned.child_frame_id = odom_frame_ + "_ned";
        enu_ned.transform.rotation.x = 0.7071067811865476;
        enu_ned.transform.rotation.y = 0.7071067811865476;
        enu_ned.transform.rotation.z = 0.0;
        enu_ned.transform.rotation.w = 0.0;
        tfs.push_back(enu_ned);

        geometry_msgs::msg::TransformStamped flu_frd;
        flu_frd.header.stamp = now();
        flu_frd.header.frame_id = base_frame_;
        flu_frd.child_frame_id = base_frame_ + "_frd";
        flu_frd.transform.rotation.x = 1.0;
        flu_frd.transform.rotation.y = 0.0;
        flu_frd.transform.rotation.z = 0.0;
        flu_frd.transform.rotation.w = 0.0;
        tfs.push_back(flu_frd);

        static_tf_->sendTransform(tfs);
        RCLCPP_INFO(get_logger(), "Broadcast static TF: %s->%s_ned, %s->%s_frd",
                    odom_frame_.c_str(), odom_frame_.c_str(),
                    base_frame_.c_str(), base_frame_.c_str());
    }

    void imu_callback(const sensor_msgs::msg::Imu::ConstSharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(data_mutex_);
        omega_x_ = msg->angular_velocity.x;
        omega_y_ = msg->angular_velocity.y;
        omega_z_ = msg->angular_velocity.z;
        imu_received_ = true;
    }

    void depth_callback(const mavros_msgs::msg::DebugValue::ConstSharedPtr msg)
    {
        // Only NAMED_VALUE_FLOAT with the configured key carries depth.
        if (msg->name != depth_key_)
            return;
        std::lock_guard<std::mutex> lock(data_mutex_);
        latest_depth_ = msg->value_float;   // metres below surface, positive down
        last_depth_time_ = now();
        have_depth_ = true;
    }

    void initSerial()
    {
        serial_fd_ = open(serial_port_.c_str(), O_RDWR | O_NOCTTY);
        if (serial_fd_ < 0)
        {
            RCLCPP_ERROR(get_logger(), "Failed to open serial port %s", serial_port_.c_str());
            throw std::runtime_error("Serial port not found");
        }
        termios tty{};
        tcgetattr(serial_fd_, &tty);
        cfmakeraw(&tty);
        cfsetispeed(&tty, B115200);
        cfsetospeed(&tty, B115200);
        tty.c_cflag |= (CLOCAL | CREAD);
        tty.c_cflag &= ~CSIZE;
        tty.c_cflag |= CS8;
        tcsetattr(serial_fd_, TCSANOW, &tty);
        RCLCPP_INFO(get_logger(), "Serial started on %s", serial_port_.c_str());
    }

    void sendCommand(const char *cmd)
    {
        if (write(serial_fd_, cmd, strlen(cmd)) < 0 || write(serial_fd_, "\r", 1) < 0)
            RCLCPP_WARN(get_logger(), "Serial write failed for command '%s'", cmd);
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    }

    void initDVL()
    {
        if (write(serial_fd_, "+++", 3) < 0)
            RCLCPP_WARN(get_logger(), "Serial write failed sending break");
        std::this_thread::sleep_for(std::chrono::seconds(3));
        sendCommand("CR1");
        sendCommand("PD0");
        sendCommand("CK");
        sendCommand("CS");
        RCLCPP_INFO(get_logger(), "DVL started");
    }

    void readLoop()
    {
        uint8_t byte;
        RCLCPP_INFO(get_logger(), "DVL read thread started");
        while (rclcpp::ok() && running_)
        {
            int n = read(serial_fd_, &byte, 1);
            if (n > 0)
            {
                if (buffer_index < BUFFER_SIZE)
                    buffer[buffer_index++] = byte;
                else
                    buffer_index = 0;
                processBuffer();
            }
        }
    }

    void processBuffer()
    {
        while (buffer_index >= 6)
        {
            size_t i = 0;
            bool found = false;
            for (; i + 1 < buffer_index; i++)
            {
                if (buffer[i] == 0x7F && buffer[i + 1] == 0x7F)
                {
                    found = true;
                    break;
                }
            }
            if (!found)
            {
                buffer_index = 0;
                return;
            }
            if (i > 0)
            {
                memmove(buffer, buffer + i, buffer_index - i);
                buffer_index -= i;
            }
            if (buffer_index < 6)
                return;

            uint16_t length;
            memcpy(&length, buffer + 2, sizeof(uint16_t));
            if (length < 6 || length > BUFFER_SIZE)
            {
                memmove(buffer, buffer + 2, buffer_index - 2);
                buffer_index -= 2;
                continue;
            }
            if (buffer_index < length)
                return;

            parseEnsemble(length);
            memmove(buffer, buffer + length, buffer_index - length);
            buffer_index -= length;
        }
    }

    void parseEnsemble(uint16_t length)
    {
        uint8_t types = buffer[5];
        if (types > 32)
            return;
        uint16_t offsets[32]{};
        memcpy(offsets, buffer + 6, types * sizeof(uint16_t));

        bool got_bottom_track = false;
        for (int i = 0; i < types; i++)
        {
            if (offsets[i] >= length)
                continue;
            uint8_t *block = buffer + offsets[i];
            uint16_t id;
            memcpy(&id, block, sizeof(uint16_t));
            if (id == 0x0000)
                parseFixedLeader(block);
            else if (id == 0x0600)
            {
                parseBottomTrack(block);
                got_bottom_track = true;
            }
        }
        if (got_bottom_track)
            publishData();
    }

    void parseFixedLeader(uint8_t *block)
    {
        uint16_t angle;
        memcpy(&angle, block + 58, sizeof(uint16_t));
        beam_angle = angle * 0.01 * M_PI / 180.0;
    }

    void parseBottomTrack(uint8_t *block)
    {
        std::lock_guard<std::mutex> lock(data_mutex_);
        bool valid = true;
        for (int i = 0; i < 4; i++)
        {
            memcpy(&bt.vel_beam[i], block + 16 + i * 2, sizeof(int16_t));
            if (bt.vel_beam[i] == PD0_BAD_VELOCITY)
                valid = false;
        }
        bt.valid = valid;
    }

    void publishData()
    {
        float v1, v2, v3, v4;
        double omega_x, omega_y, omega_z;
        bool valid;
        bool depth_ok = false;
        double depth = 0.0;

        {
            std::lock_guard<std::mutex> lock(data_mutex_);
            valid = bt.valid;
            v1 = bt.vel_beam[0] / 1000.0f;
            v2 = bt.vel_beam[1] / 1000.0f;
            v3 = bt.vel_beam[2] / 1000.0f;
            v4 = bt.vel_beam[3] / 1000.0f;
            omega_x = omega_x_;
            omega_y = omega_y_;
            omega_z = omega_z_;
            if (use_depth_ && have_depth_ &&
                (now() - last_depth_time_).seconds() < depth_timeout_)
            {
                depth_ok = true;
                depth = latest_depth_;
            }
        }

        if (!valid)
        {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                                 "DVL bottom-track invalid (no bottom lock); skipping publish");
            return;
        }

        const float sa = std::sin(beam_angle);
        const float ca = std::cos(beam_angle);
        if (std::fabs(sa) < 1e-6f || std::fabs(ca) < 1e-6f)
            return;

        const double vx = (v1 - v2) / (2.0 * sa);
        const double vy = (v4 - v3) / (2.0 * sa);
        const double vz = (v1 + v2 + v3 + v4) / (4.0 * ca);

        Eigen::Vector3d v_dvl(vx, vy, vz);
        Eigen::Vector3d omega(omega_x, omega_y, omega_z);
        Eigen::Vector3d r_lever(lever_arm_[0], lever_arm_[1], lever_arm_[2]);
        Eigen::Vector3d v_pixhawk = v_dvl - omega.cross(r_lever);

        const double nan = std::numeric_limits<double>::quiet_NaN();

        nav_msgs::msg::Odometry odom;
        odom.header.stamp = now();
        odom.header.frame_id = odom_frame_;
        odom.child_frame_id = base_frame_;

        // Orientation not fused.
        odom.pose.pose.orientation.x = nan;
        odom.pose.pose.orientation.y = nan;
        odom.pose.pose.orientation.z = nan;
        odom.pose.pose.orientation.w = nan;

        if (depth_ok)
        {
            // Position with only Z (depth) trusted. odom is ENU (z up), so a
            // depth below the surface is negative z; MAVROS converts ENU->NED.
            odom.pose.pose.position.x = 0.0;
            odom.pose.pose.position.y = 0.0;
            odom.pose.pose.position.z = -depth;
            odom.pose.covariance[0] = 1e6;             // X: effectively ignored
            odom.pose.covariance[7] = 1e6;             // Y: effectively ignored
            odom.pose.covariance[14] = depth_variance_; // Z: trusted (depth)
        }
        else
        {
            // No depth yet -> position ignored, velocity-only aiding.
            odom.pose.pose.position.x = nan;
            odom.pose.pose.position.y = nan;
            odom.pose.pose.position.z = nan;
        }

        odom.twist.twist.linear.x = v_pixhawk.x();
        odom.twist.twist.linear.y = v_pixhawk.y();
        odom.twist.twist.linear.z = v_pixhawk.z();
        odom.twist.covariance[0] = vel_variance_;
        odom.twist.covariance[7] = vel_variance_;
        odom.twist.covariance[14] = vel_variance_;

        mavros_odom_pub_->publish(odom);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    try
    {
        rclcpp::spin(std::make_shared<DVLNode>());
    }
    catch (const std::exception &e)
    {
        std::cerr << "Fatal error: " << e.what() << std::endl;
    }
    rclcpp::shutdown();
    return 0;
}
