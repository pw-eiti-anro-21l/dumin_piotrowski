cmake_minimum_required(VERSION 3.5)
project(cpp_my_robot)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(orocos_kdl REQUIRED)
find_package(kdl_parser)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(rclcpp REQUIRED)
find_package(urdf REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Convert interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/JintControl.srv"
)
 
ament_export_dependencies(rosidl_default_runtime)

# Include Cpp "include" directory
include_directories(include)           

# Create Cpp executable                    
add_executable(KDL_DKIN src/KDL_DKIN.cpp)
ament_target_dependencies(KDL_DKIN orocos_kdl)
ament_target_dependencies(KDL_DKIN rclcpp)
ament_target_dependencies(KDL_DKIN kdl_parser)
ament_target_dependencies(KDL_DKIN urdf)
ament_target_dependencies(KDL_DKIN geometry_msgs)
ament_target_dependencies(KDL_DKIN sensor_msgs)

add_executable(jint_control_srv src/jint_control_srv.cpp)
ament_target_dependencies(jint_control_srv orocos_kdl)
ament_target_dependencies(jint_control_srv rclcpp)
ament_target_dependencies(jint_control_srv kdl_parser)
ament_target_dependencies(jint_control_srv urdf)
ament_target_dependencies(jint_control_srv sensor_msgs)
rosidl_target_interfaces(jint_control_srv
  ${PROJECT_NAME} "rosidl_typesupport_cpp")
  
add_executable(jint src/jint.cpp)
ament_target_dependencies(jint rclcpp)
rosidl_target_interfaces(jint
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

# Install Cpp executables
install(TARGETS
  KDL_DKIN
  jint_control_srv
  jint
  DESTINATION lib/${PROJECT_NAME}
  )

# Install Python modules  
ament_python_install_package(${PROJECT_NAME})
  
# Install Python executables
install(PROGRAMS
  scripts/NONKDL_DKIN.py
  DESTINATION lib/${PROJECT_NAME}
)
  
install(FILES
  urdf/my_robot.rviz
  urdf/my_robot.urdf.xml
  DESTINATION share/${PROJECT_NAME}/
  )
  
  # install the launch directory
install(FILES
	launch/my_robot.launch.py
	launch/my_robot_jint.launch.py
  launch/my_robot_KDL.launch.py
  launch/my_robot_NONKDL.launch.py
  DESTINATION share/${PROJECT_NAME}/
  )

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

