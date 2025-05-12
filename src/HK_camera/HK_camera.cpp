/***************************************************************************************************
*
* Notes about how to configure your OpenCV environment and project.
* 1. You can prepare the required installation package from the official website. https://opencv.org/releases.html
* 2. If the *.lib files doesn't exist in the package download, you need to compile by yourself with the CMake tool.
* 3. Add the 'bin' folder path to the PATH.
* 4. Configure the 'Additional Include Directories', 'Additional Library Directories' and 'Additional Dependencies' for current project property.
*
* If there is any question or request, please feel free to contact us.

***************************************************************************************************/


#include "HK_camera.h"

unsigned int g_nPayloadSize = 0;
void* handle = NULL;
cv::Mat srcImageA, srcImageB;
mutex mutexBufferA, mutexBufferB;
bool bufferAFull = false;
bool bufferBFull = false;
bool bufferRefreshA = false;

// print the discovered devices information to user
bool PrintDeviceInfo(MV_CC_DEVICE_INFO* pstMVDevInfo)
{
    if (NULL == pstMVDevInfo)
    {
        printf("The Pointer of pstMVDevInfo is NULL!\n");
        return false;
    }
    if (pstMVDevInfo->nTLayerType == MV_GIGE_DEVICE)
    {
        int nIp1 = ((pstMVDevInfo->SpecialInfo.stGigEInfo.nCurrentIp & 0xff000000) >> 24);
        int nIp2 = ((pstMVDevInfo->SpecialInfo.stGigEInfo.nCurrentIp & 0x00ff0000) >> 16);
        int nIp3 = ((pstMVDevInfo->SpecialInfo.stGigEInfo.nCurrentIp & 0x0000ff00) >> 8);
        int nIp4 = (pstMVDevInfo->SpecialInfo.stGigEInfo.nCurrentIp & 0x000000ff);

        // print current ip and user defined name
        printf("CurrentIp: %d.%d.%d.%d\n", nIp1, nIp2, nIp3, nIp4);
        printf("UserDefinedName: %s\n\n", pstMVDevInfo->SpecialInfo.stGigEInfo.chUserDefinedName);
    }
    else if (pstMVDevInfo->nTLayerType == MV_USB_DEVICE)
    {
        printf("UserDefinedName: %s\n", pstMVDevInfo->SpecialInfo.stUsb3VInfo.chUserDefinedName);
        printf("Serial Number: %s\n", pstMVDevInfo->SpecialInfo.stUsb3VInfo.chSerialNumber);
        printf("Device Number: %d\n\n", pstMVDevInfo->SpecialInfo.stUsb3VInfo.nDeviceNumber);
    }
    else
    {
        printf("Not support.\n");
    }

    return true;
}

int RGB2BGR(unsigned char* pRgbData, unsigned int nWidth, unsigned int nHeight)
{
    if (NULL == pRgbData)
    {
        return MV_E_PARAMETER;
    }

    for (unsigned int j = 0; j < nHeight; j++)
    {
        for (unsigned int i = 0; i < nWidth; i++)
        {
            unsigned char red = pRgbData[j * (nWidth * 3) + i * 3];
            pRgbData[j * (nWidth * 3) + i * 3] = pRgbData[j * (nWidth * 3) + i * 3 + 2];
            pRgbData[j * (nWidth * 3) + i * 3 + 2] = red;
        }
    }

    return MV_OK;
}

// convert data stream in Mat format
cv::Mat Convert2Mat(MV_FRAME_OUT_INFO_EX* pstImageInfo, unsigned char* pData)
{
    cv::Mat srcImage = cv::Mat();
    if (pstImageInfo->enPixelType == PixelType_Gvsp_BayerRG8){
        //printf("BaryerRG8\n");
        srcImage = cv::Mat(pstImageInfo->nHeight, pstImageInfo->nWidth, CV_8UC1, pData);
        cv::cvtColor(srcImage,srcImage,cv::COLOR_BayerRG2RGB);
    }
    else if (pstImageInfo->enPixelType == PixelType_Gvsp_RGB8_Packed){
        RGB2BGR(pData, pstImageInfo->nWidth, pstImageInfo->nHeight);
        srcImage = cv::Mat(pstImageInfo->nHeight, pstImageInfo->nWidth, CV_8UC3, pData);
    }
    else{
        printf("unsupported pixel format\n");
    }
    return srcImage;
}



int OpenDevice()
{
    int nRet = MV_OK;
    //void* handle = NULL;
    do
    {
        // Enum device
        MV_CC_DEVICE_INFO_LIST stDeviceList;
        memset(&stDeviceList, 0, sizeof(MV_CC_DEVICE_INFO_LIST));
        nRet = MV_CC_EnumDevices(MV_GIGE_DEVICE | MV_USB_DEVICE, &stDeviceList);
        if (MV_OK != nRet)
        {
            printf("Enum Devices fail! nRet [0x%x]\n", nRet);
            break;
        }

        if (stDeviceList.nDeviceNum > 0)
        {
            for (unsigned int i = 0; i < stDeviceList.nDeviceNum; i++)
            {
                printf("[device %d]:\n", i);
                MV_CC_DEVICE_INFO* pDeviceInfo = stDeviceList.pDeviceInfo[i];
                if (NULL == pDeviceInfo)
                {
                    break;
                }
                PrintDeviceInfo(pDeviceInfo);
            }
        }
        else
        {
            printf("Find No Devices!\n");
            break;
        }

        // input the format to convert
        printf("[0] OpenCV_Mat\n");
        printf("[1] OpenCV_IplImage\n");
        printf("Please Input Format to convert:\n");
        unsigned int nFormat = 0;
        if (nFormat >= 2)
        {
            printf("Input error!\n");
            return 0;
        }

        // select device to connect
        printf("Please Input camera index(0-%d):\n", stDeviceList.nDeviceNum - 1);
        unsigned int nIndex = 0;
        if (nIndex >= stDeviceList.nDeviceNum)
        {
            printf("Input error!\n");
            break;
        }

        // Select device and create handle
        nRet = MV_CC_CreateHandle(&handle, stDeviceList.pDeviceInfo[nIndex]);
        if (MV_OK != nRet)
        {
            printf("Create Handle fail! nRet [0x%x]\n", nRet);
            break;
        }

        // open device
        nRet = MV_CC_OpenDevice(handle);
        if (MV_OK != nRet)
        {
            printf("Open Device fail! nRet [0x%x]\n", nRet);
            break;
        }

        // Detection network optimal package size(It only works for the GigE camera)
        if (stDeviceList.pDeviceInfo[nIndex]->nTLayerType == MV_GIGE_DEVICE)
        {
            int nPacketSize = MV_CC_GetOptimalPacketSize(handle);
            if (nPacketSize > 0)
            {
                nRet = MV_CC_SetIntValue(handle, "GevSCPSPacketSize", nPacketSize);
                if (nRet != MV_OK)
                {
                    printf("Warning: Set Packet Size fail nRet [0x%x]!", nRet);
                }
            }
            else
            {
                printf("Warning: Get Packet Size fail nRet [0x%x]!", nPacketSize);
            }
        }

        // Set trigger mode as off
        nRet = MV_CC_SetEnumValue(handle, "TriggerMode", 0);
        if (MV_OK != nRet)
        {
            printf("Set Trigger Mode fail! nRet [0x%x]\n", nRet);
            break;
        }

        // Get payload size
        MVCC_INTVALUE stParam;
        memset(&stParam, 0, sizeof(MVCC_INTVALUE));
        nRet = MV_CC_GetIntValue(handle, "PayloadSize", &stParam);
        if (MV_OK != nRet)
        {
            printf("Get PayloadSize fail! nRet [0x%x]\n", nRet);
            break;
        }
        g_nPayloadSize = stParam.nCurValue;
        // Start grab image
        nRet = MV_CC_StartGrabbing(handle);
        if (MV_OK != nRet)
        {
            printf("Start Grabbing fail! nRet [0x%x]\n", nRet);
        }
    }while (0);
    return 0;
}

void* GetImgThread(void *arg)
{
    cv::Mat img;
    MV_FRAME_OUT_INFO_EX stImageInfo = { 0 };
    memset(&stImageInfo, 0, sizeof(MV_FRAME_OUT_INFO_EX));
    unsigned char* pData = (unsigned char*)malloc(sizeof(unsigned char) * (g_nPayloadSize));
    if (pData == NULL)
    {
        printf("Allocate memory failed.\n");
    }
    // get one frame from camera with timeout=1000ms
    while (1)
    {
        int nRet = MV_CC_GetOneFrameTimeout(handle, pData, g_nPayloadSize, &stImageInfo, 4000);
        if (nRet == MV_OK)
        {
            //printf("Get One Frame: Width[%d], Height[%d], nFrameNum[%d]\n",
                //stImageInfo.nWidth, stImageInfo.nHeight, stImageInfo.nFrameNum);
            //printf("Get one Frame\n");
        }
        else
        {
            printf("No data[0x%x]\n", nRet);
            free(pData);
            pData = NULL;
        }
        if (bufferAFull == false)
        {
            mutexBufferA.lock();
            srcImageA = Convert2Mat(&stImageInfo, pData);
            bufferRefreshA = false;
            bufferAFull = true;
            mutexBufferA.unlock();
        }
        else
        {
            if (bufferBFull == false)
            {
                mutexBufferB.lock();
                srcImageB = Convert2Mat(&stImageInfo, pData);
                bufferRefreshA = true;
                bufferBFull = true;
                mutexBufferB.unlock();
            }
            else
            {
                if (bufferRefreshA)
                {
                    bufferAFull = false;
                    mutexBufferA.lock();
                    srcImageA = Convert2Mat(&stImageInfo, pData);
                    bufferRefreshA = false;
                    bufferAFull = true;
                    mutexBufferA.unlock();
                }
                else
                {
                    bufferBFull = false;
                    mutexBufferB.lock();
                    srcImageB = Convert2Mat(&stImageInfo, pData);
                    bufferRefreshA = true;
                    bufferBFull = true;
                    mutexBufferB.unlock();
                }
            }
        }
    }
    CloseDevice();
    return 0;
}

bool ReadImgBuffer(cv::Mat& srcImage)
{
    int waitTimeCount = 0;
    while (true)
    {
        if (bufferAFull == true)
        {
            mutexBufferA.lock();
            srcImage = srcImageA;
            bufferAFull = false;
            mutexBufferA.unlock();
            break;
        }
        else
        {
            if (bufferBFull == true)
            {
                mutexBufferB.lock();
                srcImage = srcImageB;
                bufferBFull = false;
                mutexBufferB.unlock();
                break;
            }
            else
            {
                cv::waitKey(10);
                waitTimeCount++;
                if (waitTimeCount == 100)
                {
                    printf("ReadBuffer Over Time, waiting........\n");
                    return false;
                }
            }
        }
    }
    return true;
}

int CloseDevice()
{
    // Stop grab image
    int nRet = MV_CC_StopGrabbing(handle);
    if (MV_OK != nRet)
    {
        printf("Stop Grabbing fail! nRet [0x%x]\n", nRet);
        return -1;
    }

    // Close device
    nRet = MV_CC_CloseDevice(handle);
    if (MV_OK != nRet)
    {
        printf("ClosDevice fail! nRet [0x%x]\n", nRet);
        return -1;
    }

    // Destroy handle
    nRet = MV_CC_DestroyHandle(handle);
    if (MV_OK != nRet)
    {
        printf("Destroy Handle fail! nRet [0x%x]\n", nRet);
        return -1;
    }


    if (nRet != MV_OK)
    {
        if (handle != NULL)
        {
            MV_CC_DestroyHandle(handle);
            handle = NULL;
        }
    }

    printf("Press a key to exit.\n");
    //WaitForKeyPress();

    return 0;
}