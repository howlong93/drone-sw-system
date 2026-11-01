cmake_minimum_required(VERSION 3.16)
project(corridor_planning)

set(PATHCOVER_EXAMPLE "quadrotor" CACHE STRING "PathCover example to build (quadrotor or jackal)")
set_property(CACHE PATHCOVER_EXAMPLE PROPERTY STRINGS quadrotor jackal)
if(NOT PATHCOVER_EXAMPLE STREQUAL "quadrotor" AND NOT PATHCOVER_EXAMPLE STREQUAL "jackal")
  message(FATAL_ERROR "PATHCOVER_EXAMPLE must be 'quadrotor' or 'jackal' (got '${PATHCOVER_EXAMPLE}')")
endif()
message(STATUS "corridor_planning: building ${PATHCOVER_EXAMPLE} example")

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_BUILD_TYPE Release)
add_compile_options(-march=native -O3 -w)

find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  std_msgs
  roscpp
  sensor_msgs
  jps_lib
  visualization_msgs
  qhull_lib
  polytope_msgs
)


catkin_package(CATKIN_DEPENDS 
  roscpp 
  geometry_msgs
  sensor_msgs
  std_msgs
  visualization_msgs
  jps_lib
  qhull_lib
  polytope_msgs
)

if(PATHCOVER_EXAMPLE STREQUAL "jackal")
  set(PATHCOVER_DIM 2)
else()
  set(PATHCOVER_DIM 3)
endif()

file(GLOB SRC_FILES src/*.cpp)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)
add_executable(${PROJECT_NAME}_node ${SRC_FILES})
target_compile_definitions(${PROJECT_NAME}_node PRIVATE PATHCOVER_DIM=${PATHCOVER_DIM})
add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES} 
  Eigen3::Eigen 
  ${OpenCV_LIBS}
)

if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(pathcover_line_seed_test
    test/pathcover_line_seed_test.cpp
  )
  if(TARGET pathcover_line_seed_test)
    target_link_libraries(pathcover_line_seed_test
      ${catkin_LIBRARIES}
      Eigen3::Eigen
      gtest_main
    )
  endif()
endif()
