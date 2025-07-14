cmake_minimum_required(VERSION 3.15)
project(rd_ambiguity_solver)

# Find dependencies if not already found at project level
if(NOT TARGET voyant-api::voyant-api)
    find_package(voyant-api REQUIRED PATHS /usr/lib/cmake/voyant-api)
endif()

if(NOT TARGET yaml-cpp)
    find_package(yaml-cpp REQUIRED)
endif()

add_executable(rng_dop_amb_solver ${CMAKE_CURRENT_SOURCE_DIR}/../../utils/point_cloud_utils.cpp ego_velocity_estimator.cpp doppler_ambiguity_resolver.cpp)
target_link_libraries(rng_dop_amb_solver
    voyant-api::voyant-api
    yaml-cpp
)

# Include the headers from the "include" directory
target_include_directories(rng_dop_amb_solver PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/../../utils # Top-level "utils" directory
)
set_target_properties(rng_dop_amb_solver
    PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin"
)
