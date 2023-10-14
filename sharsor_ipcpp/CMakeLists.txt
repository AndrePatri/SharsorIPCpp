cmake_minimum_required(VERSION 3.10)

set(LIBRARY_NAME SharsorIPCpp)
project(${LIBRARY_NAME})

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)

find_package(Boost REQUIRED COMPONENTS system)
find_package(Eigen3 3.3 REQUIRED)

add_library(${LIBRARY_NAME} STATIC
    src/Server.cpp
    src/Client.cpp
)

target_include_directories(${LIBRARY_NAME} PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(${LIBRARY_NAME} PUBLIC
    Boost::boost
    Eigen3::Eigen
    rt
)

add_subdirectory(tests/)
