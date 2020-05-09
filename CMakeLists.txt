cmake_minimum_required (VERSION 3.1)
project(argus_stereo_sync)

set(ARGUS_DIR ${CMAKE_CURRENT_SOURCE_DIR}/libs/jetson_multimedia_api/argus)
set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake "${CMAKE_MODULE_PATH}")

find_package(catkin REQUIRED COMPONENTS roscpp sensor_msgs)
find_package(CUDA REQUIRED)
find_package(Argus REQUIRED)

find_package(PkgConfig REQUIRED)
pkg_check_modules(GTK3 REQUIRED gtk+-3.0)

add_library(utils_lib
  ${ARGUS_DIR}/samples/utils/ArgusHelpers.cpp
  ${ARGUS_DIR}/samples/utils/CUDAHelper.cpp
  ${ARGUS_DIR}/samples/utils/EGLGlobal.cpp
  ${ARGUS_DIR}/samples/utils/Observed.cpp
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

set(SOURCES src/main.cpp src/convert.cu)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${ARGUS_INCLUDE_DIR}
  ${CUDA_INCLUDE_DIRS}
)

catkin_package()
cuda_add_executable(${PROJECT_NAME} ${SOURCES})

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${ARGUS_LIBRARIES}
  ${CUDA_LIBRARIES}
  utils_lib
  cuda
)

include(CPack)
