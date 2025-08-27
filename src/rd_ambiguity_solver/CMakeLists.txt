cmake_minimum_required(VERSION 3.15)
project(rd_ambiguity_solver)

find_package(PkgConfig REQUIRED)
find_package(voyant-api QUIET)
find_package(yaml-cpp QUIET)
find_package(PCL REQUIRED COMPONENTS common)

# Find dependencies if not already found at project level
if(NOT TARGET voyant-api::voyant-api)
    find_package(voyant-api REQUIRED PATHS /usr/lib/cmake/voyant-api)
endif()
if(NOT TARGET yaml-cpp)
    find_package(yaml-cpp REQUIRED)
endif()
find_package(PCL REQUIRED COMPONENTS common)

# Include Compiler Options
include(${CMAKE_CURRENT_SOURCE_DIR}/../../cmake/CompilerOptions.cmake)

# Create executable
add_executable(rng_dop_amb_solver)
set_project_warnings(rng_dop_amb_solver)

# Source files using target_sources
target_sources(rng_dop_amb_solver PRIVATE
    ego_velocity_estimator.cpp
    doppler_ambiguity_resolver.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../../utils/point_cloud_utils.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../../utils/point_cloud_logger.cpp
)

# Link everything
target_link_libraries(rng_dop_amb_solver PRIVATE
    voyant-api::voyant-api
    yaml-cpp
    ${PCL_LIBRARIES}
)

# Include directories
target_include_directories(rng_dop_amb_solver PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/../../utils
)

# Apply compiler settings
setup_modern_cpp(rng_dop_amb_solver)

# Output directory
set_target_properties(rng_dop_amb_solver PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin
)
