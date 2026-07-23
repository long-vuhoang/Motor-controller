cmake_minimum_required(VERSION 3.15)
project(aftech611_motor_controller CXX)

# Thiết lập chuẩn C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# Tìm các thư viện hệ thống cần thiết
find_package(Threads REQUIRED)
find_package(Boost REQUIRED)
find_package(pybind11 REQUIRED)
find_package(Python3 COMPONENTS Interpreter Development REQUIRED)


# Gọi các thư mục con chứa cấu hình build
add_subdirectory(robstride_api)
add_subdirectory(example)