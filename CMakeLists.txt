cmake_minimum_required(VERSION 2.8.7)
project(RMBP)

find_package(OpenMP)
if (OPENMP_FOUND)
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()
set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O2 -std=c++11")

include_directories(include)
include_directories(${CMAKE_CURRENT_SOURCE_DIR})
add_subdirectory(lemon)
add_subdirectory(ann)

file(GLOB SOURCES "src/*.cpp")

add_executable(RMBP ${SOURCES})
target_link_libraries(RMBP lemon)
target_link_libraries(RMBP ann)
