#include "main.h"

// stuff we know about the network and the input/output blobs
static const int INPUT_H = Yolo::INPUT_H;
static const int INPUT_W = Yolo::INPUT_W;
static const int CLASS_NUM = Yolo::CLASS_NUM;
static const int OUTPUT_SIZE = Yolo::MAX_OUTPUT_BBOX_COUNT * sizeof(Yolo::Detection) / sizeof(float) + 1;  // we assume the yololayer outputs no more than MAX_OUTPUT_BBOX_COUNT boxes that conf >= 0.1
const char* INPUT_BLOB_NAME = "data";
const char* OUTPUT_BLOB_NAME = "prob";
static Logger gLogger;

void doInference(IExecutionContext& context, cudaStream_t& stream, void **buffers, float* output, int batchSize) {
    // infer on the batch asynchronously, and DMA output back to host
    context.enqueue(batchSize, buffers, stream, nullptr);
    CUDA_CHECK(cudaMemcpyAsync(output, buffers[1], batchSize * OUTPUT_SIZE * sizeof(float), cudaMemcpyDeviceToHost, stream));
    cudaStreamSynchronize(stream);
}

/*帧率*/
double fps;
double start_time, end_time;
double total_time = 0;
int num_frames = 0;

/*中心点坐标: 目标框, R标, 预测坐标*/
cv::Point2f Tcenter;
cv::Point2f Rcenter;
cv::Point2f Pcenter;

/*识别框的距离(mm)*/
float r_distance; 

/*buff旋转角速度(rad/s)*/
float angleV;

/*子弹枪口速度(mm/s)*/
float bulletV = 18000;

/*发送的角度*/
double send_yaw;
double send_pitch;

/*计算buff旋转角速度*/
double buffT = 0.2;  // s
double curBufftime;
double lastBufftime;
double totalBufftime;
float lastBuffRad;
float buffAngleV()
{
    float buffRad = std::atan2(Tcenter.y - Rcenter.y, Tcenter.x - Rcenter.x);
    // std::cout << "buffRad:  " << buffRad << std::endl;
    float buffRadChange = buffRad - lastBuffRad;
    if(std::abs(buffRadChange) < 0.01)
    {
        buffRadChange = 0.0;
    }
    // std::cout << "buffRadChange:  " << buffRadChange << std::endl;

    if(buffRadChange < -4)
    {
        buffRadChange = (PI - lastBuffRad) + (PI + buffRad);
    }
     //std::cout << "buffRadChange:  " << buffRadChange << std::endl;

    // std::cout << "lastBuffRad:  " << lastBuffRad << std::endl;
    lastBuffRad = buffRad;

    return buffRadChange / buffT;
}

int main(int argc, char** argv) 
{
    /*串口*/
    fd = open_port(0);
    if(set_uart_config(fd,115200,8,'N',1)<0){
        printf("port config failed!\n");
        //exit(1);
    }
    else
    {
        printf("port config succed\n");
    }

    /*tensorrt*/
    cudaSetDevice(DEVICE);
    std::string engine_name = "buff_R.engine";
    if (engine_name.empty()) {
        std::cerr << "arguments not right!" << std::endl;
        return 1; // 返回非零值表示出现错误
    }
    // deserialize the .engine and run inference
    std::ifstream file(engine_name, std::ios::binary);
    if (!file.good()) 
    {
        std::cerr << "read " << engine_name << " error!" << std::endl;
        return -1;
    }

    char* trtModelStream = nullptr;
    size_t size = 0;
    file.seekg(0, file.end);
    size = file.tellg();
    file.seekg(0, file.beg);
    trtModelStream = new char[size];
    assert(trtModelStream);
    file.read(trtModelStream, size);
    file.close();

    static float prob[BATCH_SIZE * OUTPUT_SIZE];
    IRuntime* runtime = createInferRuntime(gLogger);
    assert(runtime != nullptr);
    ICudaEngine* engine = runtime->deserializeCudaEngine(trtModelStream, size);
    assert(engine != nullptr);
    IExecutionContext* context = engine->createExecutionContext();
    assert(context != nullptr);
    delete[] trtModelStream;
    assert(engine->getNbBindings() == 2);
    float* buffers[2];

    // In order to bind the buffers, we need to know the names of the input and output tensors.
    // Note that indices are guaranteed to be less than IEngine::getNbBindings()
    const int inputIndex = engine->getBindingIndex(INPUT_BLOB_NAME);
    const int outputIndex = engine->getBindingIndex(OUTPUT_BLOB_NAME);
    assert(inputIndex == 0);
    assert(outputIndex == 1);

    // Create GPU buffers on device
    CUDA_CHECK(cudaMalloc((void**)&buffers[inputIndex], BATCH_SIZE * 3 * INPUT_H * INPUT_W * sizeof(float)));
    CUDA_CHECK(cudaMalloc((void**)&buffers[outputIndex], BATCH_SIZE * OUTPUT_SIZE * sizeof(float)));

    // Create stream
    cudaStream_t stream;
    CUDA_CHECK(cudaStreamCreate(&stream));
    uint8_t* img_host = nullptr;
    uint8_t* img_device = nullptr;

    // prepare input data cache in pinned memory 
    CUDA_CHECK(cudaMallocHost((void**)&img_host, MAX_IMAGE_INPUT_SIZE_THRESH * 3));

    // prepare input data cache in device memory
    CUDA_CHECK(cudaMalloc((void**)&img_device, MAX_IMAGE_INPUT_SIZE_THRESH * 3));

    // me
    cv::Mat img;
    OpenDevice();
    pthread_t getImg;
    pthread_create(&getImg,NULL,GetImgThread,NULL);

    /*开始时间*/
    lastBufftime = cv::getTickCount();


    while(1)
    {
        start_time = cv::getTickCount();
        ReadImgBuffer(img);
        
        if(img.data)
        {
            /*结果测试图*/
            cv::Mat resultimg;
            img.copyTo(resultimg);

            std::vector<cv::Mat> imgs_buffer(BATCH_SIZE);
            float* buffer_idx = (float*)buffers[inputIndex];
            imgs_buffer[0] = img;
            size_t  size_image = img.cols * img.rows * 3;
            size_t  size_image_dst = INPUT_H * INPUT_W * 3;
            //copy data to pinned memory
            memcpy(img_host, img.data, size_image);
            //copy data to device memory
            CUDA_CHECK(cudaMemcpyAsync(img_device, img_host, size_image, cudaMemcpyHostToDevice, stream));
            preprocess_kernel_img(img_device, img.cols, img.rows, buffer_idx, INPUT_W, INPUT_H, stream);
            buffer_idx += size_image_dst;
            auto start = std::chrono::system_clock::now();
            doInference(*context, stream, (void**)buffers, prob, BATCH_SIZE);
            auto end = std::chrono::system_clock::now();
            //std::cout << "inference time: " << std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count() << "ms" << std::endl;
            std::vector<std::vector<Yolo::Detection>> batch_res(1);

            auto& res = batch_res[0];
            nms(res, &prob[0], CONF_THRESH, NMS_THRESH);
            //std::cout << res.size() << std::endl;

            auto& res_pre = batch_res[0];
            /*深度推理测试图*/
            cv::Mat img_pre = imgs_buffer[0];
            //std::cout << res_pre.size() << std::endl;

            if(res_pre.size() > 0)
            {
                for (size_t j = 0; j < res_pre.size(); j++)
                {
                    cv::Rect2f r = get_rect(img_pre, res_pre[j].bbox);

                    /*pnp计算车和大符之间的深度*/
                   //r_distance =  calculation_distance((cv::Rect2d) r);
                    
                    cv::rectangle(img_pre, r, cv::Scalar(0x27, 0xC1, 0x36), 2);
                    cv::putText(img_pre, std::to_string((int)res_pre[j].class_id), cv::Point(r.x, r.y - 1), cv::FONT_HERSHEY_PLAIN, 1.2, cv::Scalar(0xFF, 0xFF, 0xFF), 2);
                    if((int)res_pre[j].class_id == 0)
                    {
                        /*目标像素坐标点*/
                        Tcenter.x = r.x + r.width / 2;
                        Tcenter.y = r.y + r.height / 2;
                        //std::cout<<"Tcenter.x"<<Tcenter.x<<std::endl;
                         /*pnp计算车和大符之间的深度*/
                         r_distance =  calculation_distance((cv::Rect2d) r);
                    }
                    else if((int)res_pre[j].class_id == 1)
                    {
                        /*目标像素坐标点*/
                        Rcenter.x = r.x + r.width / 2;
                        Rcenter.y = r.y + r.height / 2;
                        //std::cout<<"Rcenter.x"<<Rcenter.x<<"Rcenter.y"<<Rcenter.y<<std::endl;
                    }
                }
            }
            else
            {
                send_yaw = 0;
                send_pitch = 0;
                sendbuff(fd,int16_t(send_yaw*100) ,int16_t(send_pitch*100),1);
                continue;
            }
            cv::namedWindow("Inference", cv::WINDOW_FREERATIO);
            cv::imshow("Inference" , img_pre);

            /*画出推理目标点和R标中心点(像素坐标)*/
            cv::circle(resultimg, Point2f(Tcenter.x, Tcenter.y), 5, Scalar(0,255,0), -1);
            cv::circle(resultimg, Point2f(Rcenter.x, Rcenter.y), 5, Scalar(0,255,0), -1);

            /*计算buff旋转角速度(rad/s)*/
            curBufftime = cv::getTickCount();
            float diffBufftime = (curBufftime - lastBufftime) / cv::getTickFrequency();
            lastBufftime = curBufftime;
            totalBufftime += diffBufftime;
            if(totalBufftime >= buffT)
            {
                angleV = buffAngleV();
                //std::cout << "angleV: " << angleV << std::endl;

                totalBufftime = 0;
            }

            /*子弹飞行时间(s = mm / (mm/s))*/
            float t_fly = r_distance / bulletV;
            //std::cout<<"t_fly"<<t_fly<<std::endl;

            // std::cout << "angleV: " << angleV << std::endl;
            if(angleV >= -0.1)
            {
                /*buff旋转弧度*/
                float theta = angleV * t_fly;
                // std::cout << "theta: " << theta << std::endl;

                /*二维旋转矩阵*/
                cv::Mat rotMatrix = (cv::Mat_<double>(2, 2) << std::cos(theta), - std::sin(theta),
                                                                    std::sin(theta), std::cos(theta));

                /*识别向量*/
                cv::Mat recogVec = (cv::Mat_<double>(2, 1) << Tcenter.x - Rcenter.x, Tcenter.y - Rcenter.y);

                /*预测向量*/
                cv::Mat preVec = rotMatrix * recogVec;

                /*预测坐标*/
                Pcenter.x = Rcenter.x + preVec.at<double>(0, 0);
                Pcenter.y = Rcenter.y + preVec.at<double>(1, 0);
                // std::cout << "Pcenter.x: " << Pcenter.x;
                // std::cout << "------Pcenter.y: " << Pcenter.y << std::endl;

                /*画出预测目标点*/
                cv::circle(resultimg, Point2f(Pcenter.x, Pcenter.y), 5, Scalar(0,0,255), -1);

                /*画出预测目标框*/
                float fanPixel = std::abs(std::sqrt(std::pow(Tcenter.x - Rcenter.x, 2) + std::pow(Tcenter.y - Rcenter.y, 2)));      /*扇叶实际尺寸(mm)像素(pixel)比*/
                float fanRadio = 700 / fanPixel;
                //std::cout<<"fanRadio"<< fanRadio <<std::endl;
                
                cv::Rect2d buff_Rect(Pcenter.x - 150 / fanRadio , Pcenter.y - 150 / fanRadio ,300 / fanRadio , 300 / fanRadio );
                cv::rectangle(resultimg, buff_Rect, Scalar(0,0,255), 2);

                /*计算预测欧拉角*/
                calculation_angle(buff_Rect, &send_yaw, &send_pitch);
                if(Tcenter.x < Rcenter.x )
                {
                    send_pitch = send_pitch - 5 ;
                    sendbuff(fd,int16_t(send_yaw*100) ,int16_t(send_pitch*100),1);
                }
                else
                {
                    send_pitch = send_pitch - 4 ;
                    sendbuff(fd,int16_t(send_yaw*100) ,int16_t(send_pitch*100),1);
                }
                // std::cout << "yaw_angle: " << send_yaw << std::endl;
                // std::cout << "ptich_angle: " << send_pitch << std::endl;
                // std::cout << "yaw_angle: " << int16_t(send_yaw*100) << std::endl;
                // std::cout << "ptich_angle: " << int16_t(send_pitch*100) << std::endl;
            }

            end_time = cv::getTickCount();

            /*更新累计时间和帧数*/
            double time_diff = (end_time - start_time) / cv::getTickFrequency();
            num_frames++;
            total_time += time_diff;

            /*每隔一段时间输出帧率*/
            if (total_time >= 1.0) 
            {
                fps = num_frames;

                //std::cout << "FPS: " << fps << std::endl;

                total_time = 0;
                num_frames = 0;
            }
            
            cv::putText(resultimg, "FPS:" + std::to_string((int)fps), cv::Point(resultimg.cols * 0.02, resultimg.rows * 0.05), cv::FONT_HERSHEY_PLAIN, 2, cv::Scalar(0, 255, 0), 2, 8);
            cv::imshow("result" , resultimg);
            cv::waitKey(1);
        }
    }
     // Release stream and buffers
    cudaStreamDestroy(stream);
    CUDA_CHECK(cudaFree(img_device));
    CUDA_CHECK(cudaFreeHost(img_host));
    CUDA_CHECK(cudaFree(buffers[inputIndex]));
    CUDA_CHECK(cudaFree(buffers[outputIndex]));
    // Destroy the engine
    context->destroy();
    engine->destroy();
    runtime->destroy();
    
    return 0;
}
