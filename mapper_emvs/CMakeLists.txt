project(mapper_emvs)
cmake_minimum_required(VERSION 2.8.3)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

set(CMAKE_BUILD_TYPE RelWithDebInfo) # Release, RelWithDebInfo
set(CMAKE_CXX_FLAGS "-O3 -fopenmp -std=c++11 ${CMAKE_CXX_FLAGS}")

set(HEADERS
  include/mapper_emvs/mapper_emvs.hpp
  include/mapper_emvs/data_loading.hpp
  include/mapper_emvs/depth_vector.hpp
  include/mapper_emvs/trajectory.hpp
  include/mapper_emvs/geometry_utils.hpp
  include/mapper_emvs/median_filtering.hpp
)

set(SOURCES
  src/mapper_emvs.cpp
  src/data_loading.cpp
  src/median_filtering.cpp
)

option(DEFINE_USE_INVERSE_DEPTH "Use linear spacing in inverse depth (if OFF, will use linear spacing in depth)" ON)
if(DEFINE_USE_INVERSE_DEPTH)
    add_definitions(-DUSE_INVERSE_DEPTH)
endif(DEFINE_USE_INVERSE_DEPTH)

cs_add_library(${PROJECT_NAME} ${SOURCES} ${HEADERS})

# Executables
################################################################################

cs_add_executable(run_emvs src/main.cpp)
target_link_libraries(run_emvs ${PROJECT_NAME})

################################################################################
cs_install()
cs_export()

