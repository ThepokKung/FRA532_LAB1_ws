cmake_minimum_required(VERSION 3.5)
project(robot_controller)

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

################ FIND DEPENDENCIES  #################
find_package(ament_cmake REQUIRED)		# cpp (optional)
find_package(rclcpp REQUIRED)			# cpp (optional)
find_package(ament_cmake_python REQUIRED)   	# python (optional)
find_package(rclpy REQUIRED)                	# python (optional)
######################################################

# Install Python Module
ament_python_install_package(${PROJECT_NAME})

# Install Python executables
install(PROGRAMS
  #Kinematic
  scripts/Kinematic/InverKinematic-basic.py
  scripts/Kinematic/InverKinematic-nscc.py
  scripts/Kinematic/ForwardKinematic-All.py
  #Controller
  scripts/Controller/PID_control.py
  scripts/Controller/PurePursuit_control.py
  scripts/Controller/Stanley_control.py
  #EKF
  scripts/EKF/EKF-Single.py
  scripts/EKF/EKF-Double.py
  scripts/EKF/EKF-Yaw.py
  DESTINATION lib/${PROJECT_NAME}
)

# Include Cpp "include" directory
include_directories(include)

# Create Cpp executable
add_executable(cpp_node_test src/cpp_node.cpp)
ament_target_dependencies(cpp_node_test rclcpp)

# Install Cpp executables
install(TARGETS
  cpp_node_test
  DESTINATION lib/${PROJECT_NAME}
)

################ INSTALL LAUNCH, ETC #################
install(DIRECTORY
  config
  launch
  rviz
  DESTINATION share/${PROJECT_NAME})

ament_package()
