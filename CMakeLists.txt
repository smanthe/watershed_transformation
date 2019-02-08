cmake_minimum_required(VERSION 3.5)
project(watershed)

# OpenCV
find_package(OpenCV REQUIRED)

add_executable(watershed
    src/example.cpp
    src/Watershed.cpp)

set_property(TARGET watershed PROPERTY CXX_STANDARD 14)

target_include_directories(watershed PUBLIC
    include/
    ${OpenCV_INCLUDE_DIRS})

target_link_libraries(watershed
    ${OpenCV_LIBS})
