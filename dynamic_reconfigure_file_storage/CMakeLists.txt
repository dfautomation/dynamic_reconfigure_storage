cmake_minimum_required(VERSION 2.8.3)
project(dynamic_reconfigure_file_storage)

find_package(catkin REQUIRED COMPONENTS dynamic_reconfigure dynamic_reconfigure_storage_utils pluginlib roscpp)
find_package(Boost REQUIRED COMPONENTS filesystem)

catkin_python_setup()

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES dynamic_reconfigure_file_storage
  CATKIN_DEPENDS dynamic_reconfigure dynamic_reconfigure_storage_utils pluginlib)


###########
## Build ##
###########

include_directories(include ${Boost_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})

add_library(dynamic_reconfigure_file_storage src/file_storage.cpp)
target_link_libraries(dynamic_reconfigure_file_storage ${Boost_LIBRARIES} ${catkin_LIBRARIES})
add_dependencies(dynamic_reconfigure_file_storage dynamic_reconfigure_gencpp)


#############
## Install ##
#############

install(TARGETS dynamic_reconfigure_file_storage
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

install(FILES plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h")


#############
## Testing ##
#############

if (CATKIN_ENABLE_TESTING)
  find_package(catkin REQUIRED COMPONENTS roslint)
  set(ROSLINT_CPP_OPTS "--filter=-build/include_what_you_use,-runtime/references,-whitespace/braces,-whitespace/line_length")
  set(ROSLINT_PYTHON_OPTS "--max-line-length=199" "--ignore=E128")
  roslint_cpp()
  roslint_python()
  roslint_add_test()
endif()
