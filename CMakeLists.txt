cmake_minimum_required(VERSION 3.1)
project(argus_stereo_sync)

set(ARGUS_DIR ${CMAKE_CURRENT_SOURCE_DIR}/libs/jetson_multimedia_api/argus)
set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake "${CMAKE_MODULE_PATH}")

find_package(catkin REQUIRED COMPONENTS roscpp sensor_msgs cv_bridge)
find_package(CUDA REQUIRED)
find_package(OpenCV REQUIRED)
find_package(NVMMAPI REQUIRED)
find_package(Argus REQUIRED)
find_package(EGL REQUIRED)

find_package(PkgConfig REQUIRED)
pkg_check_modules(GTK3 REQUIRED gtk+-3.0)

set(CUDA_NVCC_FLAGS ${CUDA_NVCC_FLAGS} -gencode arch=compute_53,code=sm_53 -gencode arch=compute_62,code=sm_62 -gencode arch=compute_72,code=sm_72 -gencode arch=compute_72,code=compute_72)

add_library(utils_lib
    ${ARGUS_DIR}/samples/utils/ArgusHelpers.cpp
    ${ARGUS_DIR}/samples/utils/CommonOptions.cpp
    ${ARGUS_DIR}/samples/utils/EGLGlobal.cpp
    ${ARGUS_DIR}/samples/utils/JPEGConsumer.cpp
    ${ARGUS_DIR}/samples/utils/NativeBuffer.cpp
    ${ARGUS_DIR}/samples/utils/Observed.cpp
    ${ARGUS_DIR}/samples/utils/Options.cpp
    ${ARGUS_DIR}/samples/utils/RectUtils.cpp
    ${ARGUS_DIR}/samples/utils/Thread.cpp
    ${ARGUS_DIR}/samples/utils/WindowBase.cpp
    ${ARGUS_DIR}/samples/utils/gtk/GuiElement.cpp
    ${ARGUS_DIR}/samples/utils/gtk/Window.cpp
)

target_link_libraries(utils_lib
    ${GTK3_LIBRARIES}
)

target_include_directories(utils_lib
    PUBLIC 
    ${ARGUS_DIR}/samples/utils
    ${ARGUS_DIR}/../include
    PRIVATE
    ${GTK3_INCLUDE_DIRS}
)

catkin_package()
cuda_add_library(kernel_obj src/yuv2rgb.cu)
cuda_add_executable(${PROJECT_NAME} src/main.cpp src/tic_toc.cpp)

target_link_libraries(${PROJECT_NAME}
    ${catkin_LIBRARIES}
    ${CUDA_LIBRARIES}
    ${OpenCV_LIBRARIES}
    ${NVMMAPI_LIBRARIES}
    ${ARGUS_LIBRARIES}
    ${EGL_LIBRARIES}
    utils_lib
    kernel_obj
)

target_include_directories(${PROJECT_NAME}
    PRIVATE
    ${catkin_INCLUDE_DIRS}
    ${CUDA_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
    ${NVMMAPI_INCLUDE_DIRS}
    ${ARGUS_INCLUDE_DIRS}
    ${EGL_INCLUDE_DIRS}
)
