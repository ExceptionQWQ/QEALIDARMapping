#include "robot.h"


struct RobotInfo robotInfo;

int robotSerial;

char robotRecvBuff[1024] = {0};
int robotRecvOffset = 0;

void Handle_Robot_Message(char* message, int len)
{
    message[len] = 0;

    const char* format = nullptr;
    if (strstr(message, "[imu]") != nullptr) {
        format = "[imu] roll:%lf pitch:%lf heading:%lf";
        double roll, pitch, heading;
        sscanf(message, format, &roll, &pitch, &heading);
        robotInfo.roll = roll;
        robotInfo.pitch = pitch;
        // robotInfo.heading = heading;
        robotInfo.heading = 0;
    }
}

void Robot_Init()
{
    robotInfo.xPos = 750;
    robotInfo.yPos = 750;
}

void Robot_Start()
{
    robotSerial = serialOpen(ROBOT_DEV_PATH, ROBOT_BAUD);
    if ( robotSerial < 0) {
        std::cout << "cannot open robot dev!" << std::endl;
        exit(0);
        return ;
    }
    while (true) {
        if (robotRecvOffset >= 1024) robotRecvOffset = 0;
        robotRecvBuff[robotRecvOffset] = serialGetchar(robotSerial);
        if (robotRecvBuff[robotRecvOffset] == '\n') {
            Handle_Robot_Message(robotRecvBuff, robotRecvOffset);
            robotRecvOffset = 0;
        } else {
            ++robotRecvOffset;
        }
    }

}

void MoveForward(int speed, int delay)
{
    char message[128] = {0};
    snprintf(message, 128, "[forward] speed=%d\n", speed);
    std::cout << message << std::endl;
    serialPuts(robotSerial, message);
    double xPos = robotInfo.xPos;
    double yPos = robotInfo.yPos;
    for (int i = 0; i < delay; ++i) {
        usleep(1000);
        robotInfo.xPos = xPos + cos(robotInfo.heading) * i * ROBOT_DIS;
        robotInfo.yPos = yPos + sin(robotInfo.heading) * i * ROBOT_DIS;
    }
    memset(message, 0, 128);
    snprintf(message, 128, "[forward] speed=%d\n", 0);
    std::cout << message << std::endl;
    serialPuts(robotSerial, message);
    
    robotInfo.xPos = xPos + cos(robotInfo.heading) * delay * ROBOT_DIS;
    robotInfo.yPos = yPos + sin(robotInfo.heading) * delay * ROBOT_DIS;
}

void MoveBackward(int speed, int delay)
{
    char message[128] = {0};
    snprintf(message, 128, "[forward] speed=%d\n", -speed);
    std::cout << message << std::endl;
    serialPuts(robotSerial, message);
    double xPos = robotInfo.xPos;
    double yPos = robotInfo.yPos;
    for (int i = 0; i < delay; ++i) {
        usleep(1000);
        robotInfo.xPos = xPos - cos(robotInfo.heading) * i * ROBOT_DIS;
        robotInfo.yPos = yPos - sin(robotInfo.heading) * i * ROBOT_DIS;
    }
    memset(message, 0, 128);
    snprintf(message, 128, "[forward] speed=%d\n", 0);
    std::cout << message << std::endl;
    serialPuts(robotSerial, message);
    
    robotInfo.xPos = xPos - cos(robotInfo.heading) * delay * ROBOT_DIS;
    robotInfo.yPos = yPos - sin(robotInfo.heading) * delay * ROBOT_DIS;
}

void SpinTo(double radian)
{
    radian = fmod(radian + 2 * PI, 2 * PI);
    char message[128] = {0};
    snprintf(message, 128, "[spin] radian=%.2f\n", radian);
    std::cout << message << std::endl;
    serialPuts(robotSerial, message);
}

void RobotStop()
{
    char message[128] = {0};
    snprintf(message, 128, "[forward] speed=%d\n", 0);
    std::cout << message << std::endl;
    serialPuts(robotSerial, message);
}