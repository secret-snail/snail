cmake_minimum_required(VERSION 3.12)
project(Snail LANGUAGES CXX)

#set(CMAKE_CXX_FLAGS "${CMAKE_C_FLAGS} -std=c++17 -march=native -mfpu=neon")
set(CMAKE_CXX_FLAGS "${CMAKE_C_FLAGS} -std=c++17 -march=native -mfpu=neon")
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -ggdb -fno-omit-frame-pointer")

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/cmake")
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${PROJECT_BINARY_DIR}/bin")
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${PROJECT_BINARY_DIR}/lib")
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${PROJECT_BINARY_DIR}/lib")

find_package(OpenCV REQUIRED)
message(STATUS "cv libs: ${OpenCV_LIBS}")
message(STATUS "cv includes: ${OpenCV_INCLUDE_DIRS}")

find_package(VISP REQUIRED)
message(STATUS "visp libs: ${VISP_INCLUDE_DIRS}")
message(STATUS "visp includes: ${VISP_LIBRARIES}")

add_subdirectory(src/)
