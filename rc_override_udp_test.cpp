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

void send_rc_override(int sock, sockaddr_in& addr, uint16_t ch2_pwm) {
    mavlink_message_t msg;
    uint16_t ch[18];
    for (int i = 0; i < 18; ++i) ch[i] = UINT16_MAX;

    ch[1] = ch2_pwm;  // Channel 2 (Elevator → Servo 4 on your setup)

    mavlink_msg_rc_channels_override_pack(
        255, 190, &msg,
        1, 1,
        ch[0], ch[1], ch[2], ch[3],
        ch[4], ch[5], ch[6], ch[7],
        ch[8], ch[9], ch[10], ch[11],
        ch[12], ch[13], ch[14], ch[15],
        ch[16], ch[17]
    );

    send_mavlink_message(sock, addr, msg);
    std::cout << "Sent CH2 (Elevator) override: " << ch2_pwm << std::endl;
}

int main() {
    sockaddr_in serverAddr;
    int sock = open_udp_socket("127.0.0.1", 14555, serverAddr);
    if (sock < 0) return 1;

    for (int i = 0; i < 50; ++i) {
        send_heartbeat(sock, serverAddr);
        send_rc_override(sock, serverAddr, 1100);  // Way low elevator
        usleep(100000); // 100 ms
    }

    close(sock);
    return 0;
}
