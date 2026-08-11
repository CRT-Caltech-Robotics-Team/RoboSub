#include <iostream>
#include <cstring>
#include <cmath>
#include <thread>
#include <chrono>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>

#define BUFFER_SIZE 2048

int serial_fd;

uint8_t buffer[BUFFER_SIZE];
size_t buffer_index = 0;

float beam_angle = 30.0 * M_PI / 180.0;

struct BottomTrack
{
    int16_t vel_beam[4];
    uint16_t range_beam[4];
};

struct Attitude
{
    float heading;
    float pitch;
    float roll;
};

BottomTrack bt;
Attitude att;

void delay_ms(int ms)
{
    std::this_thread::sleep_for(std::chrono::milliseconds(ms));
}

void sendCommand(const char *cmd)
{
    write(serial_fd, cmd, strlen(cmd));
    write(serial_fd, "\r", 1);

    char resp[256];
    int n = read(serial_fd, resp, sizeof(resp)-1);

    if (n > 0)
    {
        resp[n] = '\0';
        std::cout << "DVL: " << resp << std::endl;
    }

    delay_ms(500);
}

void initSerial()
{
    serial_fd = open("/dev/ttyUSB0", O_RDWR);

    if (serial_fd < 0)
    {
        std::cerr << "Failed to open serial port\n";
        exit(1);
    }

    termios tty{};
    tcgetattr(serial_fd, &tty);

    cfsetispeed(&tty, B115200);
    cfsetospeed(&tty, B115200);

    tty.c_cflag |= (CLOCAL | CREAD);
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;
    tty.c_lflag = 0;  // no canonical mode, no echo
    tty.c_iflag = 0;  // no input processing
    tty.c_oflag = 0;  // no output processing

    // 🔴 Control read behavior
    tty.c_cc[VMIN]  = 1;  // read blocks until 1 byte
    tty.c_cc[VTIME] = 10;
    


    tcsetattr(serial_fd, TCSANOW, &tty);

    std::cout << "Serial started\n";
}

void initDVL()
{
    std::cout << "Entering command mode\n";

    write(serial_fd, "+++", 3);
    delay_ms(3000);

    sendCommand("CR1");
    sendCommand("PD0");
    sendCommand("CK");
    sendCommand("CS");

    std::cout << "DVL started\n";
}

bool findHeader()
{
    while (buffer_index >= 2)
    {
        if (buffer[0] == 0x7F && buffer[1] == 0x7F)
            return true;

        memmove(buffer, buffer + 1, buffer_index - 1);
        buffer_index--;
    }

    return false;
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

    att.heading = heading * 0.01;
    att.pitch   = pitch * 0.01;
    att.roll    = roll * 0.01;
}

void parseBottomTrack(uint8_t *block)
{
    for (int i = 0; i < 4; i++)
        memcpy(&bt.vel_beam[i], block + 16 + i * 2, sizeof(int16_t));

    for (int i = 0; i < 4; i++)
        memcpy(&bt.range_beam[i], block + 24 + i * 2, sizeof(uint16_t));
}

void convertBeamToXYZ()
{
    float v1 = bt.vel_beam[0] / 1000.0;
    float v2 = bt.vel_beam[1] / 1000.0;
    float v3 = bt.vel_beam[2] / 1000.0;
    float v4 = bt.vel_beam[3] / 1000.0;

    float sa = sin(beam_angle);
    float ca = cos(beam_angle);

    float vx = (v1 - v2) / (2 * sa);
    float vy = (v4 - v3) / (2 * sa);
    float vz = (v1 + v2 + v3 + v4) / (4 * ca);

    std::cout << "Vehicle Velocity (m/s)\n";
    std::cout << "VX: " << vx << "\n";
    std::cout << "VY: " << vy << "\n";
    std::cout << "VZ: " << vz << "\n";

    std::cout << "Heading: " << att.heading << "\n";
    std::cout << "Pitch: "   << att.pitch   << "\n";
    std::cout << "Roll: "    << att.roll    << "\n";
    std::cout << std::endl;
}

void parseEnsemble()
{
    uint16_t length;
    memcpy(&length, buffer + 2, sizeof(uint16_t));

    uint8_t types = buffer[5];

    uint16_t offsets[32];
    memcpy(offsets, buffer + 6, types * sizeof(uint16_t));

    for (int i = 0; i < types; i++)
    {
        uint8_t *block = buffer + offsets[i];

        uint16_t id;
        memcpy(&id, block, sizeof(uint16_t));

        if (id == 0x0000) parseFixedLeader(block);
        if (id == 0x0080) parseVariableLeader(block);
        if (id == 0x0600) parseBottomTrack(block);
    }

    convertBeamToXYZ();

    memmove(buffer, buffer + length, buffer_index - length);
    buffer_index -= length;
}

int main()
{
    initSerial();
    initDVL();

    uint8_t byte;

    while (true)
    {
        int n = read(serial_fd, &byte, 1);

        if (n > 0)
        {
            if (buffer_index < BUFFER_SIZE)
                buffer[buffer_index++] = byte;
        }

        while (buffer_index > 6)
        {
            if (!findHeader())
                break;

            uint16_t length;
            memcpy(&length, buffer + 2, sizeof(uint16_t));

            if (buffer_index < length)
                break;

            parseEnsemble();
        }
    }

    return 0;
}