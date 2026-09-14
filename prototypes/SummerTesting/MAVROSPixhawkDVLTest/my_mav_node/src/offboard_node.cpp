// Offboard control node for a PX4 vehicle via MAVROS.
//
// PX4 will only accept OFFBOARD (and only stay in it) while a stream of
// setpoints arrives faster than COM_OF_LOSS_T (default 1 s). Sending a mode
// switch or an arm command without an already-running stream fails. This node
// therefore:
//   1. Streams a setpoint continuously at a fixed rate from the moment it
//      starts (before any mode change), and keeps streaming forever.
//   2. Once the FCU is connected and a short warm-up of setpoints has been
//      sent, requests OFFBOARD mode (retried).
//   3. Once in OFFBOARD, requests arming (retried).
//
// The default setpoint is zero velocity in the local frame (station keep),
// which does not require a valid absolute position, only a valid velocity
// estimate. That is the least aggressive setpoint for bring-up.
//
// SAFETY: with auto_arm true (default) this arms the vehicle as soon as
// OFFBOARD engages, which can spin thrusters. Set auto_arm:=false to stream
// and switch to OFFBOARD without arming, or auto_offboard:=false to only
// stream setpoints.

#include <chrono>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <mavros_msgs/msg/position_target.hpp>
#include <mavros_msgs/msg/state.hpp>
#include <mavros_msgs/srv/command_bool.hpp>
#include <mavros_msgs/srv/set_mode.hpp>

using namespace std::chrono_literals;

class OffboardNode : public rclcpp::Node
{
public:
    OffboardNode() : Node("offboard_node")
    {
        auto_offboard_ = declare_parameter<bool>("auto_offboard", true);
        auto_arm_ = declare_parameter<bool>("auto_arm", true);
        double rate_hz = declare_parameter<double>("stream_rate_hz", 50.0);
        warmup_s_ = declare_parameter<double>("warmup_s", 2.0);
        retry_s_ = declare_parameter<double>("retry_s", 2.0);

        // Setpoint: zero velocity, hold yaw. Ignore position, acceleration and
        // absolute yaw; command velocity and yaw-rate.
        sp_.coordinate_frame = mavros_msgs::msg::PositionTarget::FRAME_LOCAL_NED;
        sp_.type_mask =
            mavros_msgs::msg::PositionTarget::IGNORE_PX |
            mavros_msgs::msg::PositionTarget::IGNORE_PY |
            mavros_msgs::msg::PositionTarget::IGNORE_PZ |
            mavros_msgs::msg::PositionTarget::IGNORE_AFX |
            mavros_msgs::msg::PositionTarget::IGNORE_AFY |
            mavros_msgs::msg::PositionTarget::IGNORE_AFZ |
            mavros_msgs::msg::PositionTarget::IGNORE_YAW;
        sp_.velocity.x = 0.0;
        sp_.velocity.y = 0.0;
        sp_.velocity.z = 0.0;
        sp_.yaw_rate = 0.0;

        // MAVROS publishes /mavros/state with default (reliable) QoS.
        state_sub_ = create_subscription<mavros_msgs::msg::State>(
            "/mavros/state", 10,
            [this](const mavros_msgs::msg::State::SharedPtr msg) { state_ = *msg; });

        sp_pub_ = create_publisher<mavros_msgs::msg::PositionTarget>(
            "/mavros/setpoint_raw/local", 10);

        set_mode_client_ = create_client<mavros_msgs::srv::SetMode>("/mavros/set_mode");
        arm_client_ = create_client<mavros_msgs::srv::CommandBool>("/mavros/cmd/arming");

        start_time_ = now();
        last_request_ = now();

        auto period = std::chrono::duration<double>(1.0 / rate_hz);
        timer_ = create_wall_timer(
            std::chrono::duration_cast<std::chrono::nanoseconds>(period),
            std::bind(&OffboardNode::loop, this));

        RCLCPP_INFO(get_logger(),
                    "offboard_node: streaming zero-velocity setpoints at %.0f Hz "
                    "(auto_offboard=%s, auto_arm=%s)",
                    rate_hz, auto_offboard_ ? "true" : "false",
                    auto_arm_ ? "true" : "false");
    }

private:
    void loop()
    {
        // 1) Always keep the setpoint stream alive.
        sp_.header.stamp = now();
        sp_pub_->publish(sp_);

        if (!state_.connected)
        {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000,
                                 "waiting for MAVROS link (/mavros/state). Is mavros_node running "
                                 "and connected to the Pixhawk?");
            return;
        }

        const double warmup = (now() - start_time_).seconds();
        if (warmup < warmup_s_)
            return; // let a few setpoints flow before commanding anything

        const bool retry_ready = (now() - last_request_).seconds() > retry_s_;

        // 2) Request OFFBOARD.
        if (auto_offboard_ && state_.mode != "OFFBOARD" && retry_ready)
        {
            requestOffboard();
            last_request_ = now();
            return;
        }

        // 3) Once in OFFBOARD, request arming.
        if (auto_arm_ && state_.mode == "OFFBOARD" && !state_.armed && retry_ready)
        {
            requestArm();
            last_request_ = now();
        }
    }

    void requestOffboard()
    {
        if (!set_mode_client_->service_is_ready())
        {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                                 "set_mode service not available yet");
            return;
        }
        auto req = std::make_shared<mavros_msgs::srv::SetMode::Request>();
        req->custom_mode = "OFFBOARD";
        set_mode_client_->async_send_request(
            req, [this](rclcpp::Client<mavros_msgs::srv::SetMode>::SharedFuture fut) {
                RCLCPP_INFO(get_logger(), "OFFBOARD request sent (mode_sent=%d)",
                            fut.get()->mode_sent);
            });
    }

    void requestArm()
    {
        if (!arm_client_->service_is_ready())
        {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                                 "arming service not available yet");
            return;
        }
        auto req = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
        req->value = true;
        arm_client_->async_send_request(
            req, [this](rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedFuture fut) {
                auto res = fut.get();
                if (res->success)
                    RCLCPP_INFO(get_logger(), "ARMED");
                else
                    RCLCPP_WARN(get_logger(),
                                "Arm rejected (result=%u) - check estimate/health",
                                res->result);
            });
    }

    bool auto_offboard_ = true;
    bool auto_arm_ = true;
    double warmup_s_ = 2.0;
    double retry_s_ = 2.0;

    mavros_msgs::msg::PositionTarget sp_;
    mavros_msgs::msg::State state_;
    rclcpp::Time start_time_;
    rclcpp::Time last_request_;

    rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr state_sub_;
    rclcpp::Publisher<mavros_msgs::msg::PositionTarget>::SharedPtr sp_pub_;
    rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr set_mode_client_;
    rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedPtr arm_client_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OffboardNode>());
    rclcpp::shutdown();
    return 0;
}
