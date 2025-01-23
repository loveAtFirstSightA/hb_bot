cmake_minimum_required(VERSION 3.8)
project(bot_serial)

# 编译选项
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 11)
endif()

# 查找依赖
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(spdlog REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

# 包含头文件目录
include_directories(include)

set(executable_name bot_serial)

set(library_name ${executable_name}_core)

# 依赖库
set(dependencies
  rclcpp
  spdlog
  sensor_msgs
  nav_msgs
  tf2
  tf2_ros
)

add_executable(${executable_name} src/main.cpp)
target_include_directories(${executable_name} PRIVATE include)
ament_target_dependencies(${executable_name} ${dependencies})

# 添加库
add_library(${library_name}
  src/serial.cc
  src/impl/unix.cc
  src/impl/list_ports/list_ports_linux.cc
  src/bot_serial_controller.cpp
  src/serial_port_manager.cpp
  src/chassis_device.cpp
  src/action_unit_device.cpp
  src/common.cpp
)
target_include_directories(${library_name} PRIVATE include)
ament_target_dependencies(${library_name} ${dependencies})

target_link_libraries(${executable_name} ${library_name} fmt::fmt)

# 测试配置
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # 跳过版权检查
  set(ament_cmake_copyright_FOUND TRUE)
  # 跳过 cpplint 检查
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(TARGETS ${executable_name}
  DESTINATION lib/${PROJECT_NAME}
)

# 安装目标
install(TARGETS ${library_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# 安装头文件
install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

# 导出依赖和目标
ament_export_include_directories(include)
ament_export_libraries(${library_name})
ament_export_dependencies(${dependencies})

# ament 包
ament_package()
