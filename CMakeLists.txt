cmake_minimum_required(VERSION 3.29)

set(CMAKE_TOOLCHAIN_FILE "C:/dev/tools/vcpkg/scripts/buildsystems/vcpkg.cmake" CACHE STRING "Vcpkg toolchain file")

project(2DKalmanFilter)

set(CMAKE_CXX_STANDARD 20)

# Update these paths to your installations
set(SFML_ROOT "C:/SFML-2.6.1/SFML-2.6.1")
set(SFML_DIR "${SFML_ROOT}/lib/cmake/SFML")

# Find packages
find_package(Eigen3 REQUIRED)
find_package(SFML 2.6.1 COMPONENTS graphics window system REQUIRED)

add_executable(2DKalmanFilter main.cpp
        kalman_filter.cpp
        kalman_filter.h
        target_simulator.cpp
        target_simulator.h
        tracker_display.cpp
        tracker_display.h)

# Add SFML_ROOT as a preprocessor definition
target_compile_definitions(2DKalmanFilter PRIVATE
        SFML_ROOT="${SFML_ROOT}"
)

target_include_directories(2DKalmanFilter PRIVATE
        ${SFML_ROOT}/include
)

target_link_directories(2DKalmanFilter PRIVATE
        ${SFML_ROOT}/lib
)

target_link_libraries(2DKalmanFilter PRIVATE
        Eigen3::Eigen
        sfml-graphics-d
        sfml-window-d
        sfml-system-d
)

# Copy SFML DLLs to output directory
if(WIN32)
    file(GLOB SFML_DLLS "${SFML_ROOT}/bin/*.dll")
    file(COPY ${SFML_DLLS} DESTINATION ${CMAKE_CURRENT_BINARY_DIR})
endif()