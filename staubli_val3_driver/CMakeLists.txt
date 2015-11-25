cmake_minimum_required(VERSION 2.8.3)
project(staubli_val3_driver)

find_package(catkin REQUIRED)

catkin_package()

install(DIRECTORY val3_driver launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
