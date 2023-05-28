#include "lidar.h"

//使用双缓冲
uint8_t lidarRecvBuff1[512];
uint8_t lidarRecvBuff2[512];
volatile int lidarRecvStatus;
volatile uint8_t* lidarPackage; //等待处理的数据包
uint8_t lidarBuff[1024];
volatile uint32_t lidarBuffOffset;

struct LidarPointData lidarPointData[361]; //存储360个雷达扫描结果，下标为角度（整数）

osThreadId_t lidarTaskHandle;
osMutexId_t debugUartMutex;
uint16_t lidar_time_stamp;


uint8_t CalCRC8(uint8_t *p, uint8_t len)
{
    uint8_t crc = 0; uint16_t i;
    for (i = 0; i < len; i++){
        crc = CrcTable[(crc ^ *p++) & 0xff];
    }
    return crc;
}

void LIDAR_RxCpltCallback()
{
    if (lidarRecvStatus == 0) {
        lidarRecvStatus = 1;
        lidarPackage = lidarRecvBuff1;
        HAL_UART_Receive_IT(&huart3, lidarRecvBuff2, 512);
    } else {
        lidarRecvStatus = 0;
        lidarPackage = lidarRecvBuff2;
        HAL_UART_Receive_IT(&huart3, lidarRecvBuff1, 512);
    }
}

int DecodeLIDARPackage()
{
    if (lidarPackage) {
        if (lidarBuffOffset + 512 > 1024) lidarBuffOffset = 0;
        memcpy(lidarBuff + lidarBuffOffset, lidarPackage, 512);
        lidarBuffOffset += 512;
        lidarPackage = 0;
    }
    //定位帧头
    int packageStart = 0;
    int flag = 0;
    for (; lidarBuffOffset > 2 && packageStart < lidarBuffOffset - 1; ++packageStart) {
        if (lidarBuff[packageStart] == 0x54 && lidarBuff[packageStart + 1] == 0x2C) { //帧头固定字节0x54 0x2C
            flag = 1;
            break;
        }
    }
    //将帧头移动到起始位置
    if (flag) {
        memcpy(lidarBuff, lidarBuff + packageStart, lidarBuffOffset - packageStart);
        lidarBuffOffset -= packageStart;
    }
    if (lidarBuffOffset >= 47) { //有一个完整的数据包
        LiDARFrameTypeDef* liDarFrameTypeDef = (LiDARFrameTypeDef*)lidarBuff;
        uint8_t crc8 = CalCRC8(lidarBuff, 46);

        int flag2 = 0;
        if (crc8 == liDarFrameTypeDef->crc8) { //判断crc
            flag2 = 1;
            lidar_time_stamp = liDarFrameTypeDef->time_stamp;
            //解析角度 (end_angle – start_angle)/(len – 1)
            double step = (liDarFrameTypeDef->end_angle - liDarFrameTypeDef->start_angle) / 11.0;
            for (int i = 0; i < 12; ++i) {
                int angle = (int)((liDarFrameTypeDef->start_angle + step * i) / 100.0);
                uint16_t distance = liDarFrameTypeDef->point[i].distance;
                uint8_t intensity = liDarFrameTypeDef->point[i].intensity;
                if (angle < 0) angle = 0;
                if (angle > 360) angle = 360;
                lidarPointData[angle].distance = distance;
                lidarPointData[angle].intensity = distance;
            }
        }

        memcpy(lidarBuff, lidarBuff + 47, lidarBuffOffset - 47);
        lidarBuffOffset -= 47;

        return flag2;
    }
    return 0;
}