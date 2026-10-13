//Reads from Serial port and publishes depth to /depth

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>

#include <fcntl.h>
#include <unistd.h>
#include <termios.h>

#include <string>

class DepthPublisher : public rclcpp::Node
{
public:
    DepthPublisher()
        : Node("depth_publisher")
    {
        publisher_ =
            create_publisher<std_msgs::msg::Float32>(
                "/depth", 10);

        //*TO DO, find name of Teensy port through USB connection
        const char* SERIAL_PORT = "/dev/serial/by-id/REPLACE_WITH_TEENSY";

        serial_fd_ = open(
            SERIAL_PORT,
            O_RDWR | O_NOCTTY);

        if (serial_fd_ < 0)
        {
            RCLCPP_ERROR(
                get_logger(),
                "Failed to open serial port");
            return;
        }

        struct termios tty;

        tcgetattr(serial_fd_, &tty);

        cfsetispeed(&tty, B115200);
        cfsetospeed(&tty, B115200);

        tty.c_cflag |= CREAD | CLOCAL;

        tcsetattr(
            serial_fd_,
            TCSANOW,
            &tty);

        timer_ = create_wall_timer(
            std::chrono::milliseconds(10),
            std::bind(
                &DepthPublisher::readSerial,
                this));
    }

private:

    void readSerial()
    {
        char c;

        while(read(serial_fd_, &c, 1) > 0)
        {
            if(c == '\n')
            {
                try
                {
                    float depth =
                        std::stof(buffer_);

                    std_msgs::msg::Float32 msg;
                    msg.data = depth;

                    publisher_->publish(msg);

                    RCLCPP_INFO(
                        get_logger(),
                        "Depth: %.2f",
                        depth);
                }
                catch(...)
                {
                }

                buffer_.clear();
            }
            else
            {
                buffer_ += c;
            }
        }
    }

    int serial_fd_;
    std::string buffer_;

    rclcpp::Publisher<
        std_msgs::msg::Float32>::SharedPtr publisher_;

    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    rclcpp::spin(
        std::make_shared<DepthPublisher>());

    rclcpp::shutdown();

    return 0;
}