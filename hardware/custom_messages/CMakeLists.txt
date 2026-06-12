cmake_minimum_required(VERSION 3.5)
project(custom_messages)

# Default to C99 for C and C++14 for C++
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Define message files
set(msg_files
  "msg/CanMsg.msg"
  "msg/DigitalAndAnalogFeedback.msg"
  "msg/DigitalAndSolenoidCommand.msg"
  "msg/EncoderFeedback.msg"
  "msg/MotorCommand.msg"
  "msg/PwmCommand.msg"
  "msg/ServoCommand.msg"
  "msg/YoloDetection.msg"
  "msg/YoloDetectionArray.msg"
)

# Define service files
set(srv_files
  "srv/ResetOdom.srv"
)

# Generate interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
)

# Testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# Package
ament_package()

