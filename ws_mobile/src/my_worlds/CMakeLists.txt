cmake_minimum_required(VERSION 3.5)
project(my_worlds)

find_package(ament_cmake REQUIRED)

# Export dependencies (important for visibility in ament index)
ament_export_dependencies(ament_cmake)

# Install launch files
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# Install world files
install(
  DIRECTORY worlds
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
