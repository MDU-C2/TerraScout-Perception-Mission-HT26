cmake_minimum_required(VERSION 3.8)
project(object_localization)

find_package(ament_cmake REQUIRED)

install(PROGRAMS
  src/enemy_reader.py
  DESTINATION lib/${PROJECT_NAME}
)

install(FILES
  src/object_tracker.py
  src/simRealDepth.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
