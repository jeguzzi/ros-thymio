cmake_minimum_required(VERSION 3.5)
project(epuck_description)

find_package(urdf REQUIRED)
find_package(ament_cmake REQUIRED)

install(FILES meshes/base.dae meshes/wheel.dae
  DESTINATION share/${PROJECT_NAME}/meshes
)

install(DIRECTORY urdf launch
  DESTINATION share/${PROJECT_NAME}
)

install(FILES model.config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
