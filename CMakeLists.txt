cmake_minimum_required(VERSION 3.16)
project(super8_scanner VERSION 1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_AUTOMOC ON)

# Find required packages
find_package(PkgConfig REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Boost REQUIRED COMPONENTS system thread)

# Try Qt6 first, fall back to Qt5
find_package(Qt6 COMPONENTS Widgets Concurrent QUIET)
if(Qt6_FOUND)
    set(QT_VERSION_MAJOR 6)
    message(STATUS "Using Qt6")
else()
    find_package(Qt5 REQUIRED COMPONENTS Widgets Concurrent)
    set(QT_VERSION_MAJOR 5)
    message(STATUS "Using Qt5")
endif()

# Find libcamera
pkg_check_modules(LIBCAMERA REQUIRED libcamera)

# Include directories
include_directories(
    ${CMAKE_SOURCE_DIR}/include
    ${LIBCAMERA_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
)

# Link directories
link_directories(
    ${LIBCAMERA_LIBRARY_DIRS}
)

# Source files
set(SOURCES
    src/main.cpp
    src/Camera.cpp
    src/MarlinController.cpp
)

# Headers
set(HEADERS
    include/Camera.h
    include/MarlinController.h
    include/Config.h
)

# Create executable
add_executable(scanner ${SOURCES} ${HEADERS})

# Link libraries
target_link_libraries(scanner
    ${LIBCAMERA_LIBRARIES}
    ${OpenCV_LIBS}
    ${Boost_LIBRARIES}
    Qt${QT_VERSION_MAJOR}::Widgets
    Qt${QT_VERSION_MAJOR}::Concurrent
    pthread
)

# Compiler flags
target_compile_options(scanner PRIVATE
    -Wall
    -Wextra
    -O2
)

# Install target
install(TARGETS scanner DESTINATION bin)
