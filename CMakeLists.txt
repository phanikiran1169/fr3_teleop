cmake_minimum_required(VERSION 3.5)
project(fr3_teleop)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  fr3_teleop/joystick_teleop.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME joystick_teleop
)

ament_package()
