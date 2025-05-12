cmake_minimum_required(VERSION 2.6)

project(buffR)

add_definitions(-std=c++11)
add_definitions(-DAPI_EXPORTS)
option(CUDA_USE_STATIC_CUDA_RUNTIME OFF)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_BUILD_TYPE Debug)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -pthread")

find_package(CUDA REQUIRED)

if(WIN32)
enable_language(CUDA)
endif(WIN32)

include_directories(${PROJECT_SOURCE_DIR}/include/yolo)
include_directories(${PROJECT_SOURCE_DIR}/include/my)
include_directories(${PROJECT_SOURCE_DIR})
# include and link dirs of cuda and tensorrt, you need adapt them if yours are different
# cuda
include_directories(/usr/local/cuda/include)
link_directories(/usr/local/cuda/lib64)
# tensorrt
include_directories(/home/climber/TensorRT-8.6.1.6.Linux.x86_64-gnu.cuda-12.0/TensorRT-8.6.1.6/include/)
link_directories(/home/climber/TensorRT-8.6.1.6.Linux.x86_64-gnu.cuda-12.0/TensorRT-8.6.1.6/targets/x86_64-linux-gnu/lib/)

#hk
include_directories(/opt/MVS/include)
link_directories(/opt/MVS/lib/64)
link_libraries(/opt/MVS/lib/64/libMvCameraControl.so)

#opnecv
include_directories(/usr/local/include)
include_directories(/usr/local/include/opencv2)
link_directories(/usr/local/lib)



find_package(OpenCV)
include_directories(${OpenCV_INCLUDE_DIRS})


set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall -Ofast -g -Wfatal-errors -D_MWAITXINTRIN_H_INCLUDED")
cuda_add_library(myplugins SHARED yolo/yololayer.cu)
target_link_libraries(myplugins nvinfer cudart)


cuda_add_executable(buffR 
                                                yolo/calibrator.cpp 
                                                main.cpp 
                                                HK_camera/HK_camera.cpp 
                                                calculation/calculation.cpp
                                                usart/usart.cpp
                                                yolo/preprocess.cu)

aux_source_directory(. ALL_SRCS)
target_link_libraries(buffR nvinfer)
target_link_libraries(buffR cudart)
target_link_libraries(buffR myplugins)
target_link_libraries(buffR ${OpenCV_LIBS})
target_link_libraries(buffR -lpthread)

if(UNIX)
add_definitions(-O2 -pthread)
endif(UNIX)

