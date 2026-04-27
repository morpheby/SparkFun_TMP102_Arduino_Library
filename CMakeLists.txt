cmake_minimum_required(VERSION 3.16)

if(CMAKE_CURRENT_SOURCE_DIR STREQUAL CMAKE_SOURCE_DIR)
    project(SparkFun_TMP102 CXX)
endif()

add_library(sparkfun_tmp102 STATIC
    "${CMAKE_CURRENT_SOURCE_DIR}/src/SparkFunTMP102.cpp"
)
target_include_directories(sparkfun_tmp102 PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/src")
