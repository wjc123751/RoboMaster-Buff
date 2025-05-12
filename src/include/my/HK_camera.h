#pragma once
#include "MvCameraControl.h"
#include <opencv2/opencv.hpp>
#include <stdio.h>
#include <pthread.h>
#include "string.h"
#include <mutex>
using namespace std;

enum CONVERT_TYPE
{
    OpenCV_Mat = 0,    // Most of the time, we use 'Mat' format to store image data after OpenCV V2.1
    OpenCV_IplImage = 1,   //we may also use 'IplImage' format to store image data, usually before OpenCV V2.1
};

bool PrintDeviceInfo(MV_CC_DEVICE_INFO* pstMVDevInfo);
int RGB2BGR(unsigned char* pRgbData, unsigned int nWidth, unsigned int nHeight);
cv::Mat Convert2Mat(MV_FRAME_OUT_INFO_EX* pstImageInfo, unsigned char* pData);
int OpenDevice();
void* GetImgThread(void *arg);
bool ReadImgBuffer(cv::Mat& srcImage);
int CloseDevice();
