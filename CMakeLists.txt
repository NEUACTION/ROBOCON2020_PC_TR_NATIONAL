cmake_minimum_required(VERSION 3.0)

set(CMAKE_BUILD_TYPE "Debug")

# Project Name
set(PROJ_NAME control)
project(${PROJ_NAME})
#set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")  #设置c++的编译选项
#set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -std=c99")  #设置c的编译选项
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -O2 -pthread -fopenmp")
set(EXECUTABLE_OUTPUT_PATH ${CMAKE_SOURCE_DIR}/bin)

FILE(GLOB HEADER_LIST "${CMAKE_SOURCE_DIR}/src/*.h" "${CyMAKE_SOURCE_DIR}/Action_User/*.h" "${CMAKE_SOURCE_DIR}/MotionCard/*.h")

FILE(GLOB SOURCE_LIST "${CMAKE_SOURCE_DIR}/src/*.c" "${CMAKE_SOURCE_DIR}/Action_User/*.c" "${CMAKE_SOURCE_DIR}/MotionCard/*.c" )

FILE(GLOB SOURCE_LIST1 "${CMAKE_SOURCE_DIR}/src/*.cpp" "${CMAKE_SOURCE_DIR}/Action_User/*.cpp" "${CMAKE_SOURCE_DIR}/MotionCard/*.cpp")




add_executable(${PROJ_NAME} ${SOURCE_LIST} ${HEADER_LIST} ${SOURCE_LIST1})
target_link_libraries(${PROJ_NAME} /usr/lib/x86_64-linux-gnu/libm.so)
#target_link_libraries(${PROJ_NAME} /usr/lib/aarch64-linux-gnu/libm.so)
include_directories(${CMAKE_SOURCE_DIR}/Action_User
					${CMAKE_SOURCE_DIR}/MotionCard
					${CMAKE_SOURCE_DIR}/src)
