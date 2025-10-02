#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <mavlink/common/mavlink.h>
#include <iostream>
#include <errno.h>          // For error codes (optional)

#ifdef _WIN32
// Windows includes for sockets
#include <winsock2.h>
#include <Ws2tcpip.h>
#pragma comment(lib, "ws2_32.lib") // Automatically link with ws2_32.lib
#else
// POSIX/Unix includes for sockets
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>   // For close()
#endif

using namespace cv;


#ifndef _WIN32
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#endif

class SerialMavlinkSender {
public:
    SerialMavlinkSender(const char* portPath = "/dev/cu.usbserial-DN04T9FH", int baudRate = B57600) {
#ifndef _WIN32
        fd = open(portPath, O_RDWR | O_NOCTTY | O_NDELAY);
        if (fd == -1) {
            perror("Failed to open serial port");
            return;
        }

        struct termios options;
        tcgetattr(fd, &options);
        cfsetispeed(&options, baudRate);
        cfsetospeed(&options, baudRate);
        options.c_cflag |= (CLOCAL | CREAD);
        options.c_cflag &= ~CSIZE;
        options.c_cflag |= CS8;
        options.c_cflag &= ~PARENB;
        options.c_cflag &= ~CSTOPB;
        options.c_cflag &= ~CRTSCTS;
        options.c_lflag = 0;
        options.c_oflag = 0;
        options.c_iflag = 0;
        tcsetattr(fd, TCSANOW, &options);
#endif
    }

    ~SerialMavlinkSender() {
#ifndef _WIN32
        if (fd != -1) close(fd);
#endif
    }

    void send(const mavlink_message_t& msg) {
#ifndef _WIN32
        if (fd == -1) return;
        uint8_t txBuffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t length = mavlink_msg_to_send_buffer(txBuffer, &msg);
        write(fd, txBuffer, length);
#endif
    }

private:
    int fd = -1;
};





// Function to send a MAVLink message via UDP
#ifndef _WIN32
#include <fcntl.h>   // For open()
#include <termios.h> // For serial port config
#endif

// Function to send a MAVLink message via SERIAL
void send_mavlink_message(mavlink_message_t* msg)
{
    const char* serialPort = "/dev/cu.usbserial-DN04T9FH";
    const int baudRate = B57600;

    // Create a buffer and serialize the message
    uint8_t txBuffer[MAVLINK_MAX_PACKET_LEN];
    uint16_t length = mavlink_msg_to_send_buffer(txBuffer, msg);

#ifndef _WIN32
    // Open the serial port
    int fd = open(serialPort, O_RDWR | O_NOCTTY | O_NDELAY);
    if (fd == -1) {
        perror("Failed to open serial port");
        return;
    }

    // Configure the serial port
    struct termios options;
    tcgetattr(fd, &options);
    cfsetispeed(&options, baudRate);
    cfsetospeed(&options, baudRate);

    options.c_cflag |= (CLOCAL | CREAD);    // Enable receiver, set local mode
    options.c_cflag &= ~CSIZE;
    options.c_cflag |= CS8;                 // 8 data bits
    options.c_cflag &= ~PARENB;             // No parity
    options.c_cflag &= ~CSTOPB;             // 1 stop bit
    options.c_cflag &= ~CRTSCTS;            // No flow control

    options.c_lflag = 0;                    // Raw input mode
    options.c_oflag = 0;                    // Raw output mode
    options.c_iflag = 0;

    tcsetattr(fd, TCSANOW, &options);

    // Write the MAVLink message to the port
    ssize_t bytesWritten = write(fd, txBuffer, length);
    if (bytesWritten < 0) {
        perror("write");
    }

    // Close the serial port
    close(fd);
#endif
}


void sendRCOverride(SerialMavlinkSender& sender, int channel1, int channel2, int channel3, int channel4)
{
    mavlink_message_t msg;
    mavlink_system_t mavlink_system;
    mavlink_system.sysid = 1;
    mavlink_system.compid = 1;

    mavlink_msg_rc_channels_override_pack(
        mavlink_system.sysid,
        mavlink_system.compid,
        &msg,
        1,  // target system ID
        1,  // target component ID
        channel1,  // chan1
        channel2,  // chan2
        channel3,  // chan3
        channel4,  // chan4
        0, 0, 0, 0,    // chan5-chan8
        0, 0, 0, 0,    // chan9-chan12
        0, 0, 0, 0,    // chan13-chan16
        0, 0          // chan17-chan18
    );

    sender.send(msg);
}


int main()
{
    SerialMavlinkSender sender("/dev/cu.usbserial-DN04T9FH");

    VideoCapture cap(0);
    if (!cap.isOpened()) {
        std::cerr << "Error: Could not open camera!" << std::endl;
        return -1;
    }

    Mat frame, hsvImage, mask, maskCleaned;
    Scalar lowerPurple(130, 100, 100);
    Scalar upperPurple(170, 255, 255);
    int minArea = 100;
    Mat kernel = getStructuringElement(MORPH_RECT, Size(5, 5));

    while (true) {
        cap >> frame;
        if (frame.empty()) break;

        cvtColor(frame, hsvImage, COLOR_BGR2HSV);
        inRange(hsvImage, lowerPurple, upperPurple, mask);
        erode(mask, maskCleaned, kernel);
        dilate(maskCleaned, maskCleaned, kernel);

        std::vector<std::vector<Point>> contours;
        findContours(maskCleaned, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

        for (const auto& contour : contours) {
            Rect boundingBox = boundingRect(contour);
            if (boundingBox.area() > minArea) {
                rectangle(frame, boundingBox, Scalar(0, 255, 0), 2);

                int centerX = frame.cols / 2;
                int centerY = frame.rows / 2;
                int objectCenterX = boundingBox.x + boundingBox.width / 2;
                int objectCenterY = boundingBox.y + boundingBox.height / 2;

                int yaw = 1500 + static_cast<int>((objectCenterX - centerX) * 0.5);
                int pitch = 1500 - static_cast<int>((objectCenterY - centerY) * 0.5);
                int throttle = 1600;
                int rudder = 1500;

                sendRCOverride(sender, yaw, pitch, throttle, rudder);
            }
        }

        imshow("Purple Object Detection", frame);
        if ((char)waitKey(1) == 27) break;
    }

    cap.release();
    destroyAllWindows();
    return 0;
}
