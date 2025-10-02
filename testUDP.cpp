#include <iostream>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <cstring>
#include <chrono>
#include <thread>
#include <mavlink/common/mavlink.h>

int openSerialPort(const char* portPath) {
    int fd = open(portPath, O_RDWR | O_NOCTTY | O_SYNC);
    if (fd < 0) {
        perror("Failed to open serial port");
        return -1;
    }

    struct termios tty;
    memset(&tty, 0, sizeof tty);
    if (tcgetattr(fd, &tty) != 0) {
        perror("tcgetattr failed");
        close(fd);
        return -1;
    }

    cfsetospeed(&tty, B57600);
    cfsetispeed(&tty, B57600);

    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
    tty.c_iflag &= ~IGNBRK;
    tty.c_lflag = 0;
    tty.c_oflag = 0;
    tty.c_cc[VMIN]  = 0;
    tty.c_cc[VTIME] = 5;

    tty.c_iflag &= ~(IXON | IXOFF | IXANY);
    tty.c_cflag |= (CLOCAL | CREAD);
    tty.c_cflag &= ~(PARENB | PARODD);
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CRTSCTS;

    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
        perror("tcsetattr failed");
        close(fd);
        return -1;
    }

    return fd;
}

void send_mavlink_message(int fd, mavlink_message_t* msg) {
    uint8_t txBuffer[MAVLINK_MAX_PACKET_LEN];
    uint16_t len = mavlink_msg_to_send_buffer(txBuffer, msg);
    ssize_t written = write(fd, txBuffer, len);
    if (written < 0) {
        perror("write");
    }
}

void send_heartbeat(int fd) {
    mavlink_message_t msg;
    mavlink_msg_heartbeat_pack(
        1, 1, &msg,
        MAV_TYPE_GCS,
        MAV_AUTOPILOT_INVALID,
        MAV_MODE_MANUAL_ARMED,
        0,
        MAV_STATE_ACTIVE
    );
    send_mavlink_message(fd, &msg);
}

void sendRCOverride(int fd, int channelIndex, uint16_t pwmValue) {
    mavlink_message_t msg;
    uint16_t channels[18];
    for (int i = 0; i < 18; i++) {
        channels[i] = UINT16_MAX;  // No override
    }
    channels[channelIndex] = pwmValue;  // Override only 1 channel

    mavlink_msg_rc_channels_override_pack(
        1, 1, &msg,     // sysid, compid
        1, 1,           // target sysid, compid
        channels[0], channels[1], channels[2], channels[3],
        channels[4], channels[5], channels[6], channels[7],
        channels[8], channels[9], channels[10], channels[11],
        channels[12], channels[13], channels[14], channels[15],
        channels[16], channels[17]
    );

    send_mavlink_message(fd, &msg);
    std::cout << "Sent override on channel " << (channelIndex + 1) << " with PWM: " << pwmValue << std::endl;
}

int main() {
    const char* serialPort = "/dev/cu.usbserial-DN04T9FH";  // Change to your port
    int fd = openSerialPort(serialPort);
    if (fd < 0) return -1;

    // Send initial heartbeat
    send_heartbeat(fd);
    std::this_thread::sleep_for(std::chrono::seconds(1));

    // Sweep through channels 1 to 8
    for (int i = 0; i < 8; i++) {
        sendRCOverride(fd, i, 1700); // Send 1700 PWM on this channel only
        std::this_thread::sleep_for(std::chrono::seconds(2));
    }

    close(fd);
    return 0;
}
