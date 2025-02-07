cmake_minimum_required(VERSION 3.8)
project(rl_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  # add_compile_options(-Wall -Wextra -Wpedantic -O3 -ffast-math -fopenmp)
  # add_compile_options(-Wall -Wextra)
  add_compile_options(-O3 -Wall -Wextra -march=native -Wpedantic -funroll-loops -ffast-math -fopenmp)
endif()

set(CMAKE_PREFIX_PATH /opt/libtorch/libtorch/)
set(THIS_PACKAGE_INCLUDE_DEPENDS
  xbot_common_interfaces
  rclcpp
)

find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rclcpp REQUIRED)
# find_package(backward_ros REQUIRED)
find_package(xbot_common_interfaces REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(Torch REQUIRED HINTS "/opt/libtorch/libtorch/")

include_directories(
  include
  ${TORCH_INCLUDE_DIRS}
)

add_executable(rl_controller_node src/rl_controller_node.cpp )
ament_target_dependencies(rl_controller_node rclcpp std_msgs xbot_common_interfaces)
target_link_libraries(rl_controller_node ${TORCH_LIBRARIES} )
target_link_directories(rl_controller_node PRIVATE /opt/libtorch/libtorch/lib)

install(
  TARGETS rl_controller_node
  # EXPORT export_${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

foreach(dir launch config model)
	install(DIRECTORY ${dir}/
		DESTINATION share/${PROJECT_NAME}/${dir})
endforeach(dir)

ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
