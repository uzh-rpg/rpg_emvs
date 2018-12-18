project(cartesian3dgrid)
cmake_minimum_required(VERSION 2.8.3)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

set(CMAKE_BUILD_TYPE RelWithDebInfo) # Release, RelWithDebInfo
set(CMAKE_CXX_FLAGS "-O3 -std=c++11 ${CMAKE_CXX_FLAGS}")

file(GLOB SOURCE_FILES src/*.cpp)

cs_add_library(${PROJECT_NAME} ${SOURCE_FILES} ${HEADER_FILES})
target_link_libraries(${PROJECT_NAME})

#target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

cs_export()

