cmake_minimum_required(VERSION 3.5)
project(marc)

# Find dependencies
find_package(ament_cmake REQUIRED)

# Install URDF files
install(
  DIRECTORY urdf/
  DESTINATION share/${PROJECT_NAME}/urdf
)

# Install mesh files
install(
  DIRECTORY meshes/
  DESTINATION share/${PROJECT_NAME}/meshes
)

# Install launch files if any
install(
  DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

install(
  PROGRAMS
   src/detector_click.py
  DESTINATION lib/${PROJECT_NAME}
)


# Export package
ament_package()

