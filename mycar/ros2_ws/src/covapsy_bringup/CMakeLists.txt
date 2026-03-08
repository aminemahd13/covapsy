cmake_minimum_required(VERSION 3.8)
project(covapsy_bringup)

find_package(ament_cmake REQUIRED)

# Install launch files
install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch)

# Install config files
install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config)

# Install URDF files
install(DIRECTORY urdf/ DESTINATION share/${PROJECT_NAME}/urdf)

# Install maps directory
install(DIRECTORY maps/ DESTINATION share/${PROJECT_NAME}/maps)

ament_package()
