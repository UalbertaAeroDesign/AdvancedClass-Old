#include <mavlink/common/mavlink.h>
#include <windows.h>
#include <iostream>

HANDLE openSerialPort(const std::wstring& portName) {
    HANDLE hSerial = CreateFile(portName.c_str(), GENERIC_READ | GENERIC_WRITE, 0, 0, OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, 0);
    if (hSerial == INVALID_HANDLE_VALUE) {
        std::cerr << "Failed to open serial port!" << std::endl;
        return INVALID_HANDLE_VALUE;
    }
    DCB dcbSerialParams = { 0 };
    dcbSerialParams.DCBlength = sizeof(dcbSerialParams);
    GetCommState(hSerial, &dcbSerialParams);
    dcbSerialParams.BaudRate = CBR_57600;
    dcbSerialParams.ByteSize = 8;
    dcbSerialParams.StopBits = ONESTOPBIT;
    dcbSerialParams.Parity = NOPARITY;
    SetCommState(hSerial, &dcbSerialParams);
    return hSerial;
}

void send_mavlink_message(HANDLE hSerial, mavlink_message_t* msg) {
    uint8_t txBuffer[MAVLINK_MAX_PACKET_LEN];
    uint16_t length = mavlink_msg_to_send_buffer(txBuffer, msg);
    DWORD bytesWritten;
    WriteFile(hSerial, txBuffer, length, &bytesWritten, NULL);
}

void sendRCOverride(HANDLE hSerial) {
    mavlink_message_t msg;
    mavlink_system_t mavlink_system = { 1, 1 };

    mavlink_msg_rc_channels_override_pack(mavlink_system.sysid, mavlink_system.compid, &msg,
        1, 1, 1600, 1500, 1500, 1500, 0, 0, 0, 0);
    send_mavlink_message(hSerial, &msg);
    std::cout << "RC override message sent!" << std::endl;
}

int main() {
    HANDLE hSerial = openSerialPort(L"COM5");  // Change to correct port
    if (hSerial == INVALID_HANDLE_VALUE) return -1;

    sendRCOverride(hSerial);
    CloseHandle(hSerial);
    return 0;
}
