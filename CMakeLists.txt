cmake_minimum_required(VERSION 3.11)

project(RRT-Star)

file(GLOB SOURCES "src/*.cpp" "include/rrt-star/*.h")
add_library(${PROJECT_NAME} ${SOURCES})
target_include_directories(
  ${PROJECT_NAME}
  PUBLIC $<BUILD_INTERFACE:${${PROJECT_NAME}_SOURCE_DIR}/include>
)

add_executable(example example.cpp)
target_link_libraries(example ${PROJECT_NAME})