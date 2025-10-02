#include <iostream>
#include <thread>
#include <chrono>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include "mavlink/common/mavlink.h"

#include <fcntl.h>
#include <termios.h>
#include <unistd.h> // for close()

using namespace cv;

int openSerialPort(const std::string& portName) {
    int fd = open(portName.c_str(), O_RDWR | O_NOCTTY);
    if (fd == -1) {
        std::cerr << "Failed to open serial port!" << std::endl;
        return -1;
    }

    struct termios options;
    tcgetattr(fd, &options);    // Get current port settings
    cfsetispeed(&options, B57600);  // Set input baud rate
    cfsetospeed(&options, B57600);  // Set output baud rate
    options.c_cflag &= ~PARENB;     // No parity bit
    options.c_cflag &= ~CSTOPB;     // 1 stop bit
    options.c_cflag &= ~CSIZE;
    options.c_cflag |= CS8;         // 8 data bits
    options.c_cflag &= ~CRTSCTS;    // Disable hardware flow control
    options.c_cflag |= CREAD | CLOCAL; // Enable receiver, disable modem control

    tcsetattr(fd, TCSANOW, &options); // Apply settings immediately

    return fd;
}

// Convert the mavlink message to raw bytes before sending it off over serial. 
// Mavlink messages must be serialized before they are sent over.
void send_mavlink_message(int fd, mavlink_message_t* msg) {
    uint8_t txBuffer[MAVLINK_MAX_PACKET_LEN];
    uint16_t length = mavlink_msg_to_send_buffer(txBuffer, msg);
    write(fd, txBuffer, length); // Send through serial port
}

void moveServo(int fd, int servoNumber, int pwmValue) {
    mavlink_message_t msg;
    mavlink_system_t mavlink_system = { 1, 1 };

    mavlink_msg_command_long_pack(mavlink_system.sysid, mavlink_system.compid, &msg, 1, 1,
        MAV_CMD_DO_SET_SERVO, 0, servoNumber, pwmValue, 0, 0, 0, 0, 0); // Last 7 params are params for the command type, i.e MAV_CMD_DO_SET_SERVO
    send_mavlink_message(fd, &msg);
    std::cout << "Moved servo " << servoNumber << " to PWM " << pwmValue << std::endl;
}

int main() {
    int serialFd = openSerialPort("/dev/cu.usbmodem2101");
    if (serialFd == -1) return -1;

    VideoCapture cap(0); // Open default camera
    if (!cap.isOpened()) { close(serialFd); return -1; }

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
                moveServo(serialFd, 4, pwmValue);
            }
        }

        imshow("Purple Object Detection", frame);
        if ((char)waitKey(1) == 27) break;
    }

    cap.release();
    destroyAllWindows();
    close(serialFd); // Close the serial port
    return 0;
}
