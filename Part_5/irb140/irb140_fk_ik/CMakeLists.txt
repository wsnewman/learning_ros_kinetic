cmake_minimum_required(VERSION 2.8.3)
project(irb140_fk_ik)


find_package(catkin_simple REQUIRED)
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

catkin_simple()
#uncomment the following 4 lines to use the Eigen library
find_package(cmake_modules REQUIRED)
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})
add_definitions(${EIGEN_DEFINITIONS})

 
# example boost usage
# find_package(Boost REQUIRED COMPONENTS system thread)

# C++0x support - not quite the same as final C++11!
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")

# Libraries
# cs_add_libraries(my_lib src/my_lib.cpp)   
cs_add_library(irb140_fk_ik src/irb140_fk_ik.cpp) 

# Executables
# cs_add_executable(example src/example.cpp)
# target_link_library(example my_lib)
cs_add_executable(irb140_fk_ik_test_main src/irb140_fk_ik_test_main.cpp)
target_link_libraries(irb140_fk_ik_test_main irb140_fk_ik)

cs_add_executable(irb140_reachability_from_above src/irb140_reachability_from_above.cpp)
target_link_libraries(irb140_reachability_from_above irb140_fk_ik)

cs_install()
cs_export()
    
