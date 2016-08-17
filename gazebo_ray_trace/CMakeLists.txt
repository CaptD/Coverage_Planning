cmake_minimum_required(VERSION 2.8.3)

# set(CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS}")
add_definitions(-std=c++11)

project(gazebo_ray_trace)


find_package(catkin REQUIRED COMPONENTS
  pcl_conversions
  pcl_ros
  roscpp
  sensor_msgs
  std_msgs
  geometry_msgs
  message_generation
  gazebo_ros
  tf)

find_package(gazebo REQUIRED)

link_directories(${GAZEBO_LIBRARY_DIRS})


add_message_files(
  FILES
  RayIntersections.msg
)

add_service_files(
  FILES
  RayTrace.srv
  RayTraceEachParticle.srv
  RayTraceEntropy.srv
  RayTraceCylinder.srv
  TouchOptimizeGrid.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)
catkin_package(
 DEPENDS roscpp gazebo_ros
 INCLUDE_DIRS include
 LIBRARIES calc_entropy plot_ray_utils
)



###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(${Boost_INCLUDE_DIR} 
  ${catkin_INCLUDE_DIRS} 
  ${GAZEBO_INCLUDE_DIRS}
  include/${PROJECT_NAME})


add_library(calc_entropy src/calcEntropy.cpp)
target_link_libraries(calc_entropy ${catkin_LIBRARIES})

add_library(plot_ray_utils src/plotRayUtils.cpp)
target_link_libraries(plot_ray_utils
  ${catkin_LIBRARIES}
  calc_entropy
)
add_dependencies(plot_ray_utils gazebo_ray_trace_gencpp)

add_library(ray_tracer src/rayTracePluginUtils.cpp)
target_link_libraries(ray_tracer
  ${catkin_LIBRARIES}
  ${GAZEBO_LIBRARIES}
  ${Boost_LIBRARIES}
  ${SDF_LIBRARIES}
  calc_entropy
)
add_dependencies(ray_tracer gazebo_ray_trace_gencpp)
# add_dependencies(ray_tracer gazebo_ray_tracer_generate_messages_cpp)

add_library(touch_optimizer src/touchOptimizationUtils.cpp)
target_link_libraries(touch_optimizer
  ${catkin_LIBRARIES}
  ray_tracer
)

add_executable(gazebo_rayTrace_test src/rayTraceTestingClient.cpp)

add_executable(gazebo_rayTrace_plot src/rayTracePlotter.cpp)
target_link_libraries(gazebo_rayTrace_plot
  ${catkin_LIBRARIES}
  calc_entropy
  plot_ray_utils
)

add_executable(non_normal_plot src/nonNormal.cpp)
target_link_libraries(non_normal_plot
  ${catkin_LIBRARIES}
  calc_entropy
  plot_ray_utils
)

add_executable(gazebo_rayTrace_grid src/gridOfRays.cpp)
target_link_libraries(gazebo_rayTrace_grid
  ${catkin_LIBRARIES}
  calc_entropy
  plot_ray_utils
)

add_executable(gazebo_ig_manual src/igManual.cpp)
target_link_libraries(gazebo_ig_manual
  ${catkin_LIBRARIES}
  calc_entropy
  plot_ray_utils
)



add_dependencies(gazebo_rayTrace_test gazebo_ray_trace_gencpp)
add_dependencies(gazebo_rayTrace_plot gazebo_ray_trace_gencpp)


target_link_libraries(gazebo_rayTrace_test
  ${catkin_LIBRARIES}
)

add_library(distance_ray_trace SHARED src/touchLocalizationPlugin.cpp)
target_link_libraries(distance_ray_trace
  ${catkin_LIBRARIES}
  ${GAZEBO_LIBRARIES}
  ${Boost_LIBRARIES}
  ${SDF_LIBRARIES}
  calc_entropy
  ray_tracer
  touch_optimizer
)


install(TARGETS calc_entropy plot_ray_utils
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})



install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h")
