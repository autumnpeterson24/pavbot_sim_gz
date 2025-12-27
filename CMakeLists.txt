cmake_minimum_required(VERSION 3.8)
project(pavbot_sim_gz)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY worlds
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY rviz
  DESTINATION share/${PROJECT_NAME}
)

# Optional, but recommended if you keep model:// in this package
install(DIRECTORY models
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
