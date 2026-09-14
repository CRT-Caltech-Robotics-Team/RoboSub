// Open-loop forward thruster check for a PX4 UUV via MAVROS.
//
// Drives the vehicle gently forward for a fixed time, then stops and disarms.
// It uses PX4's MANUAL mode, where the throttle stick feeds straight through
// to forward thrust with NO position/velocity feedback. That is the key
// difference from Offboard: on a bench the closed-loop controllers would keep
// ramping thrust chasing a target they can never reach, whereas this commands
// a fixed low thrust and holds it, so it is safe to watch briefly out of water.
//
//   ros2 run my_mav_node thruster_check
//   ros2 run my_mav_node thruster_check --ros-args -p throttle:=0.10 -p hold_s:=5.0
//
// Sequence: stream neutral -> MANUAL -> arm -> ramp up -> hold -> ramp down ->
// disarm -> exit.
//
// SAFETY:
//   * Thrusters are not meant to run dry for long. Keep hold_s small and
//     throttle low. Defaults are 5 s at 12%.
//   * Stand clear of the propellers. This spins the forward thrusters.
//   * Ctrl-C stops the stream; if it is still armed, disarm with:
//       ros2 service call /mavros/cmd/arming mavros_msgs/srv/CommandBool "{value: false}"
//
// PX4 throttle mapping: MANUAL_CONTROL.z is 0..1000 and PX4 reads it as
// throttle = z/500 - 1, so 500 is stop and z = 500 + throttle*500 is forward.

#include <chrono>
#include <memory>
#include <algorithm>
#include <rclcpp/rclcpp.hpp>
#include <mavros_msgs/msg/manual_control.hpp>
#include <mavros_msgs/msg/state.hpp>
#include <mavros_msgs/srv/command_bool.hpp>
#include <mavros_msgs/srv/set_mode.hpp>

using namespace std::chrono_literals;

class ThrusterCheck : public rclcpp::Node
{
public:
    ThrusterCheck() : Node("thruster_check")
    {
        throttle_ = declare_parameter<double>("throttle", 0.12);   // forward fraction 0..1
        hold_s_ = declare_parameter<double>("hold_s", 5.0);        // seconds at full level
        ramp_s_ = declare_parameter<double>("ramp_s", 1.0);        // ramp up/down seconds
        warmup_s_ = declare_parameter<double>("warmup_s", 2.0);
        retry_s_ = declare_parameter<double>("retry_s", 1.0);
        double rate_hz = declare_parameter<double>("rate_hz", 50.0);

        throttle_ = std::clamp(throttle_, 0.0, 1.0);
        // NOTE: actual thrust is stick * UUV_MGM_THRTL, then capped at
        // UUV_THRUST_SAT. With the PX4 defaults (both 0.1) even a full stick
        // gives only ~0.1 thrust, which may be below the thruster start point.
        // Raise UUV_MGM_THRTL and UUV_THRUST_SAT on the FCU for real motion.

        mc_pub_ = create_publisher<mavros_msgs::msg::ManualControl>("/mavros/manual_control/send", 10);
        state_sub_ = create_subscription<mavros_msgs::msg::State>(
            "/mavros/state", 10,
            [this](const mavros_msgs::msg::State::SharedPtr m) { state_ = *m; });
        set_mode_client_ = create_client<mavros_msgs::srv::SetMode>("/mavros/set_mode");
        arm_client_ = create_client<mavros_msgs::srv::CommandBool>("/mavros/cmd/arming");

        start_time_ = now();
        last_request_ = now();
        auto period = std::chrono::duration<double>(1.0 / rate_hz);
        timer_ = create_wall_timer(
            std::chrono::duration_cast<std::chrono::nanoseconds>(period),
            std::bind(&ThrusterCheck::loop, this));

        RCLCPP_INFO(get_logger(),
                    "thruster_check: forward %.0f%% for %.1fs (ramp %.1fs). STAND CLEAR.",
                    throttle_ * 100.0, hold_s_, ramp_s_);
    }

private:
    enum class Phase { WarmUp, SetMode, Arm, Run, Disarm, Done };

    // Publish a MANUAL_CONTROL with the given forward fraction (0..1).
    void publishThrottle(double forward_frac)
    {
        forward_frac = std::clamp(forward_frac, 0.0, 1.0);
        mavros_msgs::msg::ManualControl m;
        m.header.stamp = now();
        m.x = 0.0f;                                   // pitch
        m.y = 0.0f;                                   // roll
        m.r = 0.0f;                                   // yaw
        m.z = static_cast<float>(500.0 + forward_frac * 500.0); // throttle: 500 stop
        mc_pub_->publish(m);
    }

    void loop()
    {
        // Always keep a manual-control stream alive (neutral by default).
        double cmd = 0.0;

        if (!state_.connected)
        {
            publishThrottle(0.0);
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000,
                                 "waiting for MAVROS link (/mavros/state). Is mavros_node running "
                                 "and connected to the Pixhawk?");
            return;
        }

        const bool retry_ready = (now() - last_request_).seconds() > retry_s_;

        switch (phase_)
        {
        case Phase::WarmUp:
            publishThrottle(0.0);
            if ((now() - start_time_).seconds() > warmup_s_)
                phase_ = Phase::SetMode;
            return;

        case Phase::SetMode:
            publishThrottle(0.0);
            if (state_.mode == "MANUAL")
            {
                phase_ = Phase::Arm;
            }
            else if (retry_ready && set_mode_client_->service_is_ready())
            {
                auto req = std::make_shared<mavros_msgs::srv::SetMode::Request>();
                req->custom_mode = "MANUAL";
                set_mode_client_->async_send_request(req);
                last_request_ = now();
                RCLCPP_INFO(get_logger(), "requesting MANUAL mode");
            }
            return;

        case Phase::Arm:
            publishThrottle(0.0);
            if (state_.armed)
            {
                phase_ = Phase::Run;
                run_start_ = now();
                RCLCPP_INFO(get_logger(), "armed; starting forward run");
            }
            else if (retry_ready && arm_client_->service_is_ready())
            {
                auto req = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
                req->value = true;
                arm_client_->async_send_request(req);
                last_request_ = now();
            }
            return;

        case Phase::Run:
        {
            const double t = (now() - run_start_).seconds();
            const double t_end = ramp_s_ + hold_s_ + ramp_s_;
            if (t >= t_end)
            {
                publishThrottle(0.0);
                phase_ = Phase::Disarm;
                last_request_ = now() - std::chrono::duration<double>(retry_s_); // allow immediate
                return;
            }
            if (t < ramp_s_)
                cmd = throttle_ * (t / ramp_s_);                 // ramp up
            else if (t < ramp_s_ + hold_s_)
                cmd = throttle_;                                 // hold
            else
                cmd = throttle_ * (1.0 - (t - ramp_s_ - hold_s_) / ramp_s_); // ramp down
            publishThrottle(cmd);
            return;
        }

        case Phase::Disarm:
            publishThrottle(0.0);
            if (!state_.armed)
            {
                RCLCPP_INFO(get_logger(), "disarmed; thruster check complete");
                phase_ = Phase::Done;
                rclcpp::shutdown();
            }
            else if (retry_ready && arm_client_->service_is_ready())
            {
                auto req = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
                req->value = false;
                arm_client_->async_send_request(req);
                last_request_ = now();
            }
            return;

        case Phase::Done:
            return;
        }
    }

    double throttle_, hold_s_, ramp_s_, warmup_s_, retry_s_;
    Phase phase_ = Phase::WarmUp;
    mavros_msgs::msg::State state_;
    rclcpp::Time start_time_, last_request_, run_start_;

    rclcpp::Publisher<mavros_msgs::msg::ManualControl>::SharedPtr mc_pub_;
    rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr state_sub_;
    rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr set_mode_client_;
    rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedPtr arm_client_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ThrusterCheck>());
    rclcpp::shutdown();
    return 0;
}
