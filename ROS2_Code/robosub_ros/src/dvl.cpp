#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include "mavros_msgs/msg/state.hpp"
#include <iostream>
#include <cstring>
#include <cmath>
#include <thread>
#include <chrono>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <mutex>


#define BUFFER_SIZE 2048

class DVLNode : public rclcpp::Node
{
public:
    DVLNode() : Node("dvl_node")
    {
        vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/dvl/velocity", 10);
        mavros_vel_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>("/mavros/setpoint_velocity/cmd_vel_unstamped", 10);
        imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("/dvl/imu", 10);

        initSerial();
        initDVL();

        reader_thread_ = std::thread(&DVLNode::readLoop, this);
    }

    ~DVLNode()
    {
        running_ = false;

        if (reader_thread_.joinable())
            reader_thread_.join();

        if (serial_fd >= 0)
            close(serial_fd);
    }

private:
    int serial_fd = -1;

    uint8_t buffer[BUFFER_SIZE]{};
    size_t buffer_index = 0;

    float beam_angle = 30.0 * M_PI / 180.0;

    struct BottomTrack
    {
        int16_t vel_beam[4]{};
    } bt;

    struct Attitude
    {
        float heading = 0.0f;
        float pitch = 0.0f;
        float roll = 0.0f;
    } att;

    std::thread reader_thread_;
    std::mutex data_mutex_;
    bool running_ = true;

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub_;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr mavros_vel_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;

    void initSerial()
    {
        serial_fd = open("/dev/ttyUSB0", O_RDWR | O_NOCTTY);

        if (serial_fd < 0)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to open serial port");
            throw std::runtime_error("Serial port not found");
        }

        termios tty{};
        tcgetattr(serial_fd, &tty);

        cfsetispeed(&tty, B115200);
        cfsetospeed(&tty, B115200);

        tty.c_cflag |= (CLOCAL | CREAD);
        tty.c_cflag &= ~CSIZE;
        tty.c_cflag |= CS8;

        tcsetattr(serial_fd, TCSANOW, &tty);

        RCLCPP_INFO(this->get_logger(), "Serial started");
    }

    void sendCommand(const char *cmd)
    {
        write(serial_fd, cmd, strlen(cmd));
        write(serial_fd, "\r", 1);
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    }

    void initDVL()
    {
        write(serial_fd, "+++", 3);
        std::this_thread::sleep_for(std::chrono::seconds(3));

        sendCommand("CR1");
        sendCommand("PD0");
        sendCommand("CK");
        sendCommand("CS");

        RCLCPP_INFO(this->get_logger(), "DVL started");
    }

    void readLoop()
    {
        uint8_t byte;

        RCLCPP_INFO(this->get_logger(), "DVL read thread started");

        while (rclcpp::ok() && running_)
        {
            int n = read(serial_fd, &byte, 1);

            if (n > 0)
            {
                if (buffer_index < BUFFER_SIZE)
                {
                    buffer[buffer_index++] = byte;
                }
                else
                {
                    buffer_index = 0; // overflow protection
                }

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

        for (int i = 0; i < types; i++)
        {
            if (offsets[i] >= length)
                continue;

            uint8_t *block = buffer + offsets[i];

            uint16_t id;
            memcpy(&id, block, sizeof(uint16_t));

            if (id == 0x0000)
                parseFixedLeader(block);
            else if (id == 0x0080)
                parseVariableLeader(block);
            else if (id == 0x0600)
                parseBottomTrack(block);
        }

        publishData();
    }

    void parseFixedLeader(uint8_t *block)
    {
        uint16_t angle;
        memcpy(&angle, block + 58, sizeof(uint16_t));
        beam_angle = angle * 0.01 * M_PI / 180.0;
    }

    void parseVariableLeader(uint8_t *block)
    {
        int16_t heading, pitch, roll;

        memcpy(&heading, block + 18, sizeof(int16_t));
        memcpy(&pitch,   block + 20, sizeof(int16_t));
        memcpy(&roll,    block + 22, sizeof(int16_t));

        std::lock_guard<std::mutex> lock(data_mutex_);
        att.heading = heading * 0.01f;
        att.pitch   = pitch * 0.01f;
        att.roll    = roll * 0.01f;
    }

    void parseBottomTrack(uint8_t *block)
    {
        std::lock_guard<std::mutex> lock(data_mutex_);

        for (int i = 0; i < 4; i++)
            memcpy(&bt.vel_beam[i], block + 16 + i * 2, sizeof(int16_t));
    }

    void publishData()
    {
        float v1, v2, v3, v4;
        float heading, pitch, roll;

        {
            std::lock_guard<std::mutex> lock(data_mutex_);

            v1 = bt.vel_beam[0] / 1000.0;
            v2 = bt.vel_beam[1] / 1000.0;
            v3 = bt.vel_beam[2] / 1000.0;
            v4 = bt.vel_beam[3] / 1000.0;

            heading = att.heading;
            pitch   = att.pitch;
            roll    = att.roll;
        }

        float sa = sin(beam_angle);
        float ca = cos(beam_angle);

        if (fabs(sa) < 1e-6 || fabs(ca) < 1e-6)
            return;

        float vx = (v1 - v2) / (2 * sa);
        float vy = (v4 - v3) / (2 * sa);
        float vz = (v1 + v2 + v3 + v4) / (4 * ca);

        geometry_msgs::msg::Twist vel;
        vel.linear.x = vx;
        vel.linear.y = vy;
        vel.linear.z = vz;

        vel_pub_->publish(vel);

        geometry_msgs::msg::TwistStamped mavros_vel;
        mavros_vel.header.stamp = this->now();
        mavros_vel.header.frame_id = "base_link";
        mavros_vel.twist = vel;
        mavros_vel_pub_->publish(mavros_vel);

        sensor_msgs::msg::Imu imu;

        tf2::Quaternion q;
        q.setRPY(roll * M_PI / 180.0,
                 pitch * M_PI / 180.0,
                 heading * M_PI / 180.0);
        q.normalize();

        imu.orientation.x = q.x();
        imu.orientation.y = q.y();
        imu.orientation.z = q.z();
        imu.orientation.w = q.w();

        imu_pub_->publish(imu);
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
