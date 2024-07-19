cmake_minimum_required(VERSION 3.1)
project(percent)

find_package(Pangolin REQUIRED)
find_package(OpenGL REQUIRED)
find_package(glfw3 REQUIRED)

find_package(Sophus REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(${Sophus_INCLUDE_DIRS})
include_directories(${EIGEN3_INCLUDE_DIR})

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS " -O3")

add_executable(${PROJECT_NAME} main.cpp Vis.cpp)
target_link_libraries(${PROJECT_NAME}
    ${OpenCV_LIBS}
    ${Sophus_LIBRARIES}
    fmt
    ${Pangolin_LIBRARIES}
)

