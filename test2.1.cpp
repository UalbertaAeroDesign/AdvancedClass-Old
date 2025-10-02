//#include <mavlink/common/mavlink.h>
//#include <windows.h>
//#include <iostream>
//#include <thread>
//#include <chrono>
//
//HANDLE openSerialPort(const std::wstring& portName) {
//    HANDLE hSerial = CreateFile(portName.c_str(), GENERIC_READ | GENERIC_WRITE, 0, 0, OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, 0);
//    if (hSerial == INVALID_HANDLE_VALUE) {
//        std::cerr << "Failed to open serial port!" << std::endl;
//        return INVALID_HANDLE_VALUE;
//    }
//    DCB dcbSerialParams = { 0 };
//    dcbSerialParams.DCBlength = sizeof(dcbSerialParams);
//    GetCommState(hSerial, &dcbSerialParams);
//    dcbSerialParams.BaudRate = CBR_57600;
//    dcbSerialParams.ByteSize = 8;
//    dcbSerialParams.StopBits = ONESTOPBIT;
//    dcbSerialParams.Parity = NOPARITY;
//    SetCommState(hSerial, &dcbSerialParams);
//    return hSerial;
//}
//
//void send_mavlink_message(HANDLE hSerial, mavlink_message_t* msg) {
//    uint8_t txBuffer[MAVLINK_MAX_PACKET_LEN];
//    uint16_t length = mavlink_msg_to_send_buffer(txBuffer, msg);
//    DWORD bytesWritten;
//    WriteFile(hSerial, txBuffer, length, &bytesWritten, NULL);
//}
//
//void moveServo(HANDLE hSerial, int servoNumber, int pwmValue) {
//    mavlink_message_t msg;
//    mavlink_system_t mavlink_system = { 1, 1 };
//
//    mavlink_msg_command_long_pack(mavlink_system.sysid, mavlink_system.compid, &msg, 1, 1,
//        MAV_CMD_DO_SET_SERVO, 0, servoNumber, pwmValue, 0, 0, 0, 0, 0);
//    send_mavlink_message(hSerial, &msg);
//    std::cout << "Moved servo " << servoNumber << " to PWM " << pwmValue << std::endl;
//}
//
//void checkAckResponse(HANDLE hSerial) {
//    uint8_t rxBuffer[512];
//    DWORD bytesRead;
//    while (true) {
//        if (ReadFile(hSerial, rxBuffer, sizeof(rxBuffer), &bytesRead, NULL) && bytesRead > 0) {
//            mavlink_message_t msg;
//            mavlink_status_t status;
//            for (DWORD i = 0; i < bytesRead; ++i) {
//                if (mavlink_parse_char(MAVLINK_COMM_0, rxBuffer[i], &msg, &status)) {
//                    if (msg.msgid == MAVLINK_MSG_ID_COMMAND_ACK) {
//                        mavlink_command_ack_t ack;
//                        mavlink_msg_command_ack_decode(&msg, &ack);
//                        std::cout << "Received ACK for command: " << ack.command << " Result: " << (int)ack.result << std::endl;
//                    }
//                }
//            }
//        }
//    }
//}
//
//void testAllServos(HANDLE hSerial) {
//    std::thread ackThread(checkAckResponse, hSerial);
//   // for (int servo = 1; servo <= 8; ++servo) {
//        int servo = 4;
//        moveServo(hSerial, servo, 1200);
//        std::this_thread::sleep_for(std::chrono::seconds(2));
//        moveServo(hSerial, servo, 1500);
//        std::this_thread::sleep_for(std::chrono::seconds(2));
//  //  }
//    ackThread.detach();
//}
//
//int main() {
//    HANDLE hSerial = openSerialPort(L"COM5");  // Change to correct port
//    if (hSerial == INVALID_HANDLE_VALUE) return -1;
//
//    // Test all servos one by one
//    testAllServos(hSerial);
//
//    CloseHandle(hSerial);
//    return 0;
//}
