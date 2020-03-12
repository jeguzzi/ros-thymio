cmake_minimum_required(VERSION 3.5)
project(thymio_description)

find_package(urdf REQUIRED)
find_package(ament_cmake REQUIRED)

install(FILES meshes/base_simple.dae meshes/base_collision.dae meshes/wheel.dae
  DESTINATION share/${PROJECT_NAME}/meshes
)

install(DIRECTORY urdf launch
  DESTINATION share/${PROJECT_NAME}
)

install(FILES model.config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
