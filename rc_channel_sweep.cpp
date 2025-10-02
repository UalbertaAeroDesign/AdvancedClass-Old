#include <iostream>
#include <cstring>
#include <unistd.h>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <mavlink/common/mavlink.h>

int open_udp_socket(const char* ip, int port, sockaddr_in& serverAddr) {
    int sock = socket(AF_INET, SOCK_DGRAM, 0);
    if (sock < 0) {
        perror("socket failed");
        return -1;
    }
    memset(&serverAddr, 0, sizeof(serverAddr));
    serverAddr.sin_family = AF_INET;
    serverAddr.sin_port = htons(port);
    inet_pton(AF_INET, ip, &serverAddr.sin_addr);
    return sock;
}

void send_mavlink_message(int sock, sockaddr_in& addr, const mavlink_message_t& msg) {
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
    sendto(sock, buf, len, 0, (sockaddr*)&addr, sizeof(addr));
}

void send_heartbeat(int sock, sockaddr_in& addr) {
    mavlink_message_t msg;
    mavlink_msg_heartbeat_pack(
        255, 190, &msg,
        MAV_TYPE_GCS,
        MAV_AUTOPILOT_INVALID,
        MAV_MODE_MANUAL_ARMED,
        0,
        MAV_STATE_ACTIVE
    );
    send_mavlink_message(sock, addr, msg);
}

void send_servo_command(int sock, sockaddr_in& addr, uint8_t servo_number, float pwm) {
    mavlink_message_t msg;

    mavlink_msg_command_long_pack(
        255, 190, &msg,
        1, 1, // target system/component
        MAV_CMD_DO_SET_SERVO,
        0,           // confirmation
        servo_number, // param1: servo number (e.g., 1 = SERVO1)
        pwm,          // param2: PWM value
        0, 0, 0, 0, 0 // unused params
    );

    send_mavlink_message(sock, addr, msg);
    std::cout << ">>> Set SERVO" << static_cast<int>(servo_number) << " to PWM " << pwm << std::endl;
}

int main() {
    sockaddr_in serverAddr;
    int sock = open_udp_socket("127.0.0.1", 14555, serverAddr);
    if (sock < 0) return 1;

    const float pwm_value = 1900;
    const float neutral_pwm = 1500;
    const int hold_time_ms = 2000;
    const int update_interval_ms = 100;

    for (uint8_t servo = 1; servo <= 8; ++servo) {
        std::cout << "\n>>> Testing SERVO" << static_cast<int>(servo) << std::endl;

        int elapsed = 0;
        while (elapsed < hold_time_ms) {
            send_heartbeat(sock, serverAddr);
            send_servo_command(sock, serverAddr, servo, pwm_value);
            usleep(update_interval_ms * 1000);
            elapsed += update_interval_ms;
        }

        std::cout << ">>> Resetting SERVO" << static_cast<int>(servo) << " to neutral" << std::endl;
        elapsed = 0;
        while (elapsed < hold_time_ms) {
            send_heartbeat(sock, serverAddr);
            send_servo_command(sock, serverAddr, servo, neutral_pwm);
            usleep(update_interval_ms * 1000);
            elapsed += update_interval_ms;
        }
    }

    std::cout << "✅ Servo output sweep complete!" << std::endl;
    close(sock);
    return 0;
}
