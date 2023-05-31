#include <iostream>
#include <unistd.h>
#include <pthread.h>


#include "lidar.h"
#include "view.h"
#include "robot.h"
#include "mapping.h"


void* Launch_Lidar(void*)
{
    Lidar_Start();

    return nullptr;
}

void* Launch_View(void*)
{
    View_Start();

    return nullptr;
}

void* Launch_Robot(void*)
{
    Robot_Start();
    return nullptr;
}

int main()
{
    Robot_Init();

    pthread_t handles1;
	pthread_create(&handles1, nullptr, Launch_Lidar, nullptr);
    pthread_t handles2;
	pthread_create(&handles2, nullptr, Launch_View, nullptr);
    pthread_t handles3;
	pthread_create(&handles3, nullptr, Launch_Robot, nullptr);

    sleep(1);

    int status = 0;

    while (true) {
        char ch;
        std::cin >> ch;
        switch (ch) {
            case 'w':
            MoveForward(30, 500);
            break;
            case 's':
            MoveBackward(30, 500);
            break;
            case 'a':
            SpinTo(robotInfo.heading + 0.2);
            break;
            case 'd':
            SpinTo(robotInfo.heading - 0.2);
            break;
            case 'm':
            AddToMap(GetCloudImage());
            if (status == 0) {
                status = 1;
                robotInfo.xPos += 270;
                robotInfo.yPos += 0;
            } else if (status == 1) {
                status = 2;
                robotInfo.xPos += 0;
                robotInfo.yPos -= 200;
            } else if (status == 2) {
                status = 3;
                robotInfo.xPos -= 270;
                robotInfo.yPos += 0;
            } else if (status == 3) {
                status = 4;
                robotInfo.xPos += 270;
                robotInfo.yPos += 400;
            }
            
            break;
        }
    }
    
    return 0;
}