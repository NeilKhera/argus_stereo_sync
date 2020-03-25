cmake_minimum_required(VERSION 3.1)
project(argus_stereo_sync)

set(ARGUS_DIR ${CMAKE_CURRENT_SOURCE_DIR}/libs/jetson_multimedia_api/argus)
set(CMAKE_MODULE_PATH ${ARGUS_DIR}/cmake "${CMAKE_MODULE_PATH}")

add_subdirectory(${ARGUS_DIR}/samples/utils)

find_package(catkin REQUIRED COMPONENTS roscpp sensor_msgs)
find_package(Argus REQUIRED)
find_package(EGL REQUIRED)

catkin_package()

add_executable(${PROJECT_NAME} src/main.cpp)

target_link_libraries(${PROJECT_NAME}
    ${catkin_LIBRARIES}
    ${ARGUS_LIBRARIES}
    ${EGL_LIBRARIES}
    argussampleutils     
)

target_include_directories(${PROJECT_NAME}
    PRIVATE
    ${catkin_INCLUDE_DIRS}
    ${ARGUS_INCLUDE_DIRS}   
    ${EGL_INCLUDE_DIRS}
    ${ARGUS_DIR}/samples/utils
)
