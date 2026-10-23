// Compact ROV status line for bench and in-water testing.
//
// Prints one line per second summarising the most useful MAVROS state, e.g.
//   ARMED | OFFBOARD | vel_ok | batt 15.2V 87% | rpy   1.2  -0.4 173.5 | depth 0.42m
//
//   ros2 run my_mav_node status_node
//
// "vel_ok" means the EKF is fusing a valid horizontal velocity (the DVL is
// being used). "no_vel" means it is not. Fields show "--" until the matching
// topic has been heard.

#include <cmath>
#include <cstdio>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <mavros_msgs/msg/state.hpp>
#include <mavros_msgs/msg/estimator_status.hpp>
#include <mavros_msgs/msg/altitude.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <sensor_msgs/msg/battery_state.hpp>
#include <sensor_msgs/msg/imu.hpp>

using std::placeholders::_1;

class StatusNode : public rclcpp::Node
{
public:
    StatusNode() : Node("status_node")
    {
        auto sensor_qos = rclcpp::SensorDataQoS();

        state_sub_ = create_subscription<mavros_msgs::msg::State>(
            "/mavros/state", 10,
            [this](const mavros_msgs::msg::State::SharedPtr m) { state_ = *m; have_state_ = true; });

        // estimator_status is published RELIABLE + TRANSIENT_LOCAL by MAVROS.
        auto est_qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable().transient_local();
        est_sub_ = create_subscription<mavros_msgs::msg::EstimatorStatus>(
            "/mavros/estimator_status", est_qos,
            [this](const mavros_msgs::msg::EstimatorStatus::SharedPtr m) { est_ = *m; have_est_ = true; });

        batt_sub_ = create_subscription<sensor_msgs::msg::BatteryState>(
            "/mavros/battery", sensor_qos,
            [this](const sensor_msgs::msg::BatteryState::SharedPtr m) { batt_ = *m; have_batt_ = true; });

        imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
            "/mavros/imu/data", sensor_qos,
            [this](const sensor_msgs::msg::Imu::SharedPtr m) { imu_ = *m; have_imu_ = true; });

        alt_sub_ = create_subscription<mavros_msgs::msg::Altitude>(
            "/mavros/altitude", sensor_qos,
            [this](const mavros_msgs::msg::Altitude::SharedPtr m) { alt_ = *m; have_alt_ = true; });

        vel_sub_ = create_subscription<geometry_msgs::msg::TwistStamped>(
            "/mavros/local_position/velocity_body", sensor_qos,
            [this](const geometry_msgs::msg::TwistStamped::SharedPtr m) { vel_ = *m; have_vel_ = true; });

        timer_ = create_wall_timer(std::chrono::seconds(1), std::bind(&StatusNode::print, this));
    }

private:
    void quatToRPY(const geometry_msgs::msg::Quaternion &q, double &roll, double &pitch, double &yaw)
    {
        // Standard quaternion -> roll/pitch/yaw (radians).
        const double sinr = 2.0 * (q.w * q.x + q.y * q.z);
        const double cosr = 1.0 - 2.0 * (q.x * q.x + q.y * q.y);
        roll = std::atan2(sinr, cosr);

        const double sinp = 2.0 * (q.w * q.y - q.z * q.x);
        pitch = std::fabs(sinp) >= 1.0 ? std::copysign(M_PI / 2.0, sinp) : std::asin(sinp);

        const double siny = 2.0 * (q.w * q.z + q.x * q.y);
        const double cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
        yaw = std::atan2(siny, cosy);
    }

    void print()
    {
        char line[256];
        int n = 0;

        // Connection / arm / mode
        if (!have_state_ || !state_.connected)
            n += snprintf(line + n, sizeof(line) - n, "NO LINK");
        else
            n += snprintf(line + n, sizeof(line) - n, "%s | %s",
                          state_.armed ? "ARMED" : "DISARMED",
                          state_.mode.empty() ? "--" : state_.mode.c_str());

        // Velocity: x/y/z (m/s, body frame) plus a health marker.
        if (have_vel_)
        {
            const char *ok = have_est_ ? (est_.velocity_horiz_status_flag ? "ok" : "no") : "?";
            n += snprintf(line + n, sizeof(line) - n, " | vel %5.2f %5.2f %5.2f [%s]",
                          vel_.twist.linear.x, vel_.twist.linear.y, vel_.twist.linear.z, ok);
        }
        else
            n += snprintf(line + n, sizeof(line) - n, " | vel --");

        // Battery
        if (have_batt_ && std::isfinite(batt_.voltage))
        {
            n += snprintf(line + n, sizeof(line) - n, " | batt %.1fV", batt_.voltage);
            if (batt_.percentage >= 0.0f && batt_.percentage <= 1.0f)
                n += snprintf(line + n, sizeof(line) - n, " %.0f%%", batt_.percentage * 100.0);
        }
        else
            n += snprintf(line + n, sizeof(line) - n, " | batt --");

        // Attitude (roll pitch yaw, degrees)
        if (have_imu_)
        {
            double r, p, y;
            quatToRPY(imu_.orientation, r, p, y);
            const double rad2deg = 180.0 / M_PI;
            n += snprintf(line + n, sizeof(line) - n, " | rpy %6.1f %6.1f %6.1f",
                          r * rad2deg, p * rad2deg, y * rad2deg);
        }
        else
            n += snprintf(line + n, sizeof(line) - n, " | rpy --");

        // Depth: use relative altitude, negated so deeper reads positive.
        if (have_alt_ && std::isfinite(alt_.relative))
            n += snprintf(line + n, sizeof(line) - n, " | depth %.2fm", -alt_.relative);
        else
            n += snprintf(line + n, sizeof(line) - n, " | depth --");

        RCLCPP_INFO(get_logger(), "%s", line);
    }

    mavros_msgs::msg::State state_;
    mavros_msgs::msg::EstimatorStatus est_;
    sensor_msgs::msg::BatteryState batt_;
    sensor_msgs::msg::Imu imu_;
    mavros_msgs::msg::Altitude alt_;
    geometry_msgs::msg::TwistStamped vel_;
    bool have_state_ = false, have_est_ = false, have_batt_ = false, have_imu_ = false,
         have_alt_ = false, have_vel_ = false;

    rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr state_sub_;
    rclcpp::Subscription<mavros_msgs::msg::EstimatorStatus>::SharedPtr est_sub_;
    rclcpp::Subscription<sensor_msgs::msg::BatteryState>::SharedPtr batt_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Subscription<mavros_msgs::msg::Altitude>::SharedPtr alt_sub_;
    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr vel_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<StatusNode>());
    rclcpp::shutdown();
    return 0;
}
