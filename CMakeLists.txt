cmake_minimum_required(VERSION 3.30)
project(donut)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_EXE_LINKER_FLAGS "-static")

add_executable(donut main.cpp)
