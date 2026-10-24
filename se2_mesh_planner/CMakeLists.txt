cmake_minimum_required(VERSION 3.8)
project(se2_mesh_planner)

if (NOT EXISTS ${CMAKE_BINARY_DIR}/CMakeCache.txt)
  if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release" CACHE STRING "" FORCE)
  endif()
endif()

find_package(ament_cmake_ros REQUIRED)
find_package(mbf_mesh_core REQUIRED)
find_package(mbf_msgs REQUIRED)
find_package(mesh_map REQUIRED)
find_package(rclcpp REQUIRED)
find_package(lvr2 REQUIRED)
find_package(pluginlib REQUIRED)
find_package(free_span_core REQUIRED)

pluginlib_export_plugin_description_file(mbf_mesh_core se2_mesh_planner.xml)

add_library(${PROJECT_NAME}
  src/se2_mesh_planner.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(${PROJECT_NAME} mbf_mesh_core mbf_msgs mesh_map rclcpp pluginlib free_span_core)
target_link_libraries(${PROJECT_NAME}
    lvr2::lvr2
    ${MPI_CXX_LIBRARIES}
)

install(DIRECTORY include/
  DESTINATION include
)
install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_targets(export_${PROJECT_NAME})
ament_export_dependencies(mbf_mesh_core mbf_msgs mesh_map rclcpp pluginlib free_span_core)
ament_package()
