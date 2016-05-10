cmake_minimum_required(VERSION 2.8.3)
project(calibration_tf2)

# check c++11 / c++0x
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "-std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "-std=c++0x")
else()
    message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

find_package(catkin REQUIRED COMPONENTS
  cmake_modules
  tf
  tf_conversions
)

find_package(Eigen REQUIRED)
find_package(cmake_modules REQUIRED)

catkin_package(
  CATKIN_DEPENDS tf tf_conversions
  DEPENDS Eigen
)

#catkin_python_setup()

include_directories(${catkin_INCLUDE_DIRS})
add_executable(calibration_tf2 src/calibration_tf2.cpp)
target_link_libraries(calibration_tf2 ${catkin_LIBRARIES} ${Eigen_LIBRARIES})
