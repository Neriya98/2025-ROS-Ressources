cmake_minimum_required(VERSION 3.8)
project(qr_code_scanner)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Install console scripts
install(PROGRAMS
  qr_code_scanner/qr_detector.py
  qr_code_scanner/image_capturer.py
  DESTINATION lib/${PROJECT_NAME}
)

# Create executable wrappers (rename scripts)
install(CODE "execute_process(COMMAND mv \${CMAKE_INSTALL_PREFIX}/lib/${PROJECT_NAME}/qr_detector.py \${CMAKE_INSTALL_PREFIX}/lib/${PROJECT_NAME}/qr_detector)")
install(CODE "execute_process(COMMAND mv \${CMAKE_INSTALL_PREFIX}/lib/${PROJECT_NAME}/image_capturer.py \${CMAKE_INSTALL_PREFIX}/lib/${PROJECT_NAME}/image_capturer)")

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()