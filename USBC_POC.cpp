#include <mavlink/common/mavlink.h>
#include <windows.h>
#include <iostream>
#include <thread>
#include <chrono>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

using namespace cv;

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
// Convert the mavlink message to raw bytes before sending it off over serial. 
// Mavlink messages must be serialized before they are sent over.
void send_mavlink_message(HANDLE hSerial, mavlink_message_t* msg) {
    uint8_t txBuffer[MAVLINK_MAX_PACKET_LEN];
    uint16_t length = mavlink_msg_to_send_buffer(txBuffer, msg);
    DWORD bytesWritten;
    WriteFile(hSerial, txBuffer, length, &bytesWritten, NULL);
}

void moveServo(HANDLE hSerial, int servoNumber, int pwmValue) {
    mavlink_message_t msg;
    mavlink_system_t mavlink_system = { 1, 1 };

    mavlink_msg_command_long_pack(mavlink_system.sysid, mavlink_system.compid, &msg, 1, 1,
        MAV_CMD_DO_SET_SERVO, 0, servoNumber, pwmValue, 0, 0, 0, 0, 0); // Last 7 params are params for the command type, i.e MAV_CMD_DO_SET_SERVO
    send_mavlink_message(hSerial, &msg);
    std::cout << "Moved servo " << servoNumber << " to PWM " << pwmValue << std::endl;
}

int main() {
    HANDLE hSerial = openSerialPort(L"COM5");
    if (hSerial == INVALID_HANDLE_VALUE) return -1;

    VideoCapture cap(0);
    if (!cap.isOpened()) { CloseHandle(hSerial); return -1; }

    Mat frame, hsvImage, mask, maskCleaned;
    Scalar lowerPurple(130, 100, 100), upperPurple(170, 255, 255); // Colour thresholds 
    Mat kernel = getStructuringElement(MORPH_RECT, Size(5, 5));

    while (true) {
        cap >> frame;
        if (frame.empty()) break;

        cvtColor(frame, hsvImage, COLOR_BGR2HSV); // Convert RGB image to HSV image
        inRange(hsvImage, lowerPurple, upperPurple, mask);
        // Reduce Noise
        erode(mask, maskCleaned, kernel);
        dilate(maskCleaned, maskCleaned, kernel);

        std::vector<std::vector<Point>> contours;
        findContours(maskCleaned, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

        for (const auto& contour : contours) {
            Rect boundingBox = boundingRect(contour);
            if (boundingBox.area() > 500 && boundingBox.area() < 10000) {
                rectangle(frame, boundingBox, Scalar(0, 255, 0), 2);
                int objectCenterY = boundingBox.y + boundingBox.height / 2;
                int centerY = frame.rows / 2;
                int pwmValue = 1500 - (objectCenterY - centerY) * 1.7;
                pwmValue = std::max(1000, std::min(2000, pwmValue)); // Keep PWM within safe range
                moveServo(hSerial, 4, pwmValue);
            }
           /* else if (boundingBox.area() > 10000) {
                int pwmValue = 2000;
                moveServo(hSerial, 4, pwmValue);
            }
            std::cout << "Bounding box size:" << boundingBox.area() << std::endl;*/
        }

        imshow("Purple Object Detection", frame);
        if ((char)waitKey(1) == 27) break;
    }

    cap.release();
    destroyAllWindows();
    CloseHandle(hSerial);
    return 0;
}
