cmake_minimum_required(VERSION 3.10)
project(main)

include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
set(ADDITIONAL_CXX_FLAG "-Wall -O3 -march=native")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${ADDITIONAL_CXX_FLAG}")

find_package(Eigen3 REQUIRED)
find_package(PythonLibs 2.7 COMPONENTS Development NumPy)

include_directories(${EIGEN3_INCLUDE_DIRS})

add_executable(main main.cpp key_value.cpp)
target_include_directories(main PRIVATE ${PYTHON_INCLUDE_DIRS})
target_link_libraries(main ${PYTHON_LIBRARIES})
