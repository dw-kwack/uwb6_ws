cmake_minimum_required(VERSION 3.8)
project(uwb_multianchor)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)

# Install core sensor URDF
install(DIRECTORY
  urdf
  DESTINATION share/${PROJECT_NAME}/
)

# Install examples (optional reference implementations)
install(DIRECTORY
  examples
  DESTINATION share/${PROJECT_NAME}/
)

# Install documentation
install(DIRECTORY
  docs
  DESTINATION share/${PROJECT_NAME}/
)

# Install worlds (if any)
install(DIRECTORY
  worlds
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
