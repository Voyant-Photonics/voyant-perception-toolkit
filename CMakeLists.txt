cmake_minimum_required(VERSION 3.15)
project(voyant_perception_toolkit VERSION 0.1.0 LANGUAGES CXX)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find project-wide dependencies
find_package(voyant-api REQUIRED PATHS /usr/lib/cmake/voyant-api)
find_package(yaml-cpp REQUIRED)

add_subdirectory(src)
