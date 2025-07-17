cmake_minimum_required(VERSION 3.8)
project(homeplus_urdf)

find_package(ament_cmake REQUIRED)

# Install directories
install(
  DIRECTORY launch urdf meshes config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
