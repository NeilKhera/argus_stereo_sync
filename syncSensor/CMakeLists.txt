cmake_minimum_required (VERSION 2.6)
project(argus_syncsensor)

set(CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake" "${CMAKE_MODULE_PATH}")

find_package(CUDA)

if(CUDA_FOUND)
  find_package(Argus REQUIRED)
  find_package(OpenGLES REQUIRED)
  find_package(EGL REQUIRED)
  find_package(roscpp REQUIRED)
  find_package(sensor_msgs REQUIRED)

  set(SOURCES main.cpp convert.cu)

  include_directories(
    ${roscpp_INCLUDE_DIRS}
    ${sensor_msgs_INCLUDE_DIRS}
    ${ARGUS_INCLUDE_DIR}
    ${CMAKE_SOURCE_DIR}/samples/utils
  )

  cuda_add_executable(${PROJECT_NAME} ${SOURCES})

  target_link_libraries(${PROJECT_NAME}
    ${roscpp_LIBRARIES}
    ${sensor_msgs_LIBRARIES}
    ${ARGUS_LIBRARIES}
    argussampleutils
    cuda
  )

  install(TARGETS ${PROJECT_NAME} DESTINATION bin)
  include(CPack)

else(CUDA_FOUND)
  message(CUDA toolkit not found, skipping ${PROJECT_NAME})

endif(CUDA_FOUND)
