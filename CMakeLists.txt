cmake_minimum_required(VERSION 2.8.3)
project(ichthus_launcher)

## Compile as C++11, supported in ROS Kinetic and newer
## add_compile_options(-std=c++11)

find_package(autoware_build_flags REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  autoware_config_msgs
  roscpp
  std_msgs
  message_generation
  )

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS
    autoware_config_msgs
    roscpp
    std_msgs
    message_runtime
  )

# Resolve system dependency on yaml-cpp, which apparently does not
# provide a CMake find_package() module.
find_package(PkgConfig REQUIRED)
pkg_check_modules(YAML_CPP REQUIRED yaml-cpp)
find_path(YAML_CPP_INCLUDE_DIR NAMES yaml_cpp.h PATHS ${YAML_CPP_INCLUDE_DIRS})
find_library(YAML_CPP_LIBRARY NAMES YAML_CPP PATHS ${YAML_CPP_LIBRARY_DIRS})
link_directories(${YAML_CPP_LIBRARY_DIRS})
if(NOT ${YAML_CPP_VERSION} VERSION_LESS "0.5")
  add_definitions(-DHAVE_NEW_YAMLCPP)
endif(NOT ${YAML_CPP_VERSION} VERSION_LESS "0.5")

###########################
### build configuration ###
###########################

include_directories(
  ${catkin_INCLUDE_DIRS}
  include
  )

add_executable(ichthus_launcher
  src/ichthus_launcher_core.cpp
  src/ichthus_launcher_node.cpp
  )

target_link_libraries(ichthus_launcher
  ${catkin_LIBRARIES}
  )

# include header files
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(TARGETS ichthus_launcher
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )

install(DIRECTORY cfg/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/cfg
  )

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  )

install(DIRECTORY scripts/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/scripts
  )

install(DIRECTORY data/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/data
  )

