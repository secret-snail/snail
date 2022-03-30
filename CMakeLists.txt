cmake_minimum_required(VERSION 3.12)
project(Snail LANGUAGES CXX)

set(CMAKE_CXX_FLAGS "${CMAKE_C_FLAGS} -std=c++17")
#set(CMAKE_C_FLAGS "-pthread -Wall -march=native -O3 -maes -mrdseed") # EMP
set(CMAKE_C_FLAGS "-Wall -march=native -O3") # EMP
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -ggdb -fno-omit-frame-pointer")

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/cmake")
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${PROJECT_BINARY_DIR}/bin")
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${PROJECT_BINARY_DIR}/lib")
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${PROJECT_BINARY_DIR}/lib")

find_package(OpenCV REQUIRED)
message(STATUS "cv libs: ${OpenCV_LIBS}")
message(STATUS "include_path: ${OpenCV_INCLUDE_DIRS}")

add_subdirectory(src/)
