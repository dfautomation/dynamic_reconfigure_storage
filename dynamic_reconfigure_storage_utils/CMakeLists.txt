cmake_minimum_required(VERSION 2.8.3)
project(dynamic_reconfigure_storage_utils)

find_package(catkin REQUIRED COMPONENTS dynamic_reconfigure roscpp)

# pkg-config package
find_package(PkgConfig)
pkg_search_module(yamlcpp REQUIRED yaml-cpp)

catkin_python_setup()

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS dynamic_reconfigure
  DEPENDS yamlcpp)


###########
## Build ##
###########

include_directories(include ${catkin_INCLUDE_DIRS} ${yamlcpp_INCLUDE_DIRS})


#############
## Install ##
#############

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
