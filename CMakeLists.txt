cmake_minimum_required(VERSION 2.8.3)
project(gen3_compliant_controllers)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")

include_directories("include")

#==============================================================================
# Dependencies
#
find_package(catkin REQUIRED COMPONENTS
  control_msgs
  control_toolbox
  controller_interface
  hardware_interface
  eigen_conversions
  realtime_tools
  eigen_conversions
  dynamic_reconfigure
)
find_package(Threads REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(PINOCCHIO pinocchio REQUIRED)

include_directories(SYSTEM ${catkin_INCLUDE_DIRS})

if ("${controller_interface_VERSION}" VERSION_LESS 0.10.0)
  message(FATAL_ERROR "controller_interface package must be at least version 0.10.0, not packaged in Indigo. Please upgrade to Jade or Kinetic or build the ros_control packages from source.")
endif()

include_directories(
  include/
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${Boost_INCLUDE_DIRS}
  ${PINOCCHIO_INCLUDE_DIRS}
)

generate_dynamic_reconfigure_options(
  config/JointSpaceParams.cfg
  config/TaskSpaceParams.cfg
)

#==============================================================================
# Catkin Configuration
#
catkin_package(
  CATKIN_DEPENDS
    control_msgs
    control_toolbox
    controller_interface
    eigen_conversions
    hardware_interface
    realtime_tools
    dynamic_reconfigure
    DEPENDS PINOCCHIO
)

link_libraries(
  # ${catkin_LIBRARIES}
  Threads::Threads
  )
#==============================================================================
# Targets
#
add_library("${PROJECT_NAME}" SHARED
  src/helpers.cpp
  src/JointSpaceCompliantController.cpp
  src/TaskSpaceCompliantController.cpp
)
target_link_libraries("${PROJECT_NAME}"
  ${catkin_LIBRARIES}
)

# make sure configure headers are built before any node using them
add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_gencfg)

target_compile_definitions(${PROJECT_NAME} PRIVATE ${PINOCCHIO_CFLAGS_OTHER})
#==============================================================================
# Format
#

include(ClangFormat.cmake)
file(GLOB_RECURSE FILES
    ${PROJECT_SOURCE_DIR}/src/*.[ch]pp
)
set(FILES ${FILES} ${FILES_INCLUDE})


clang_format_setup(VERSION 10)

if (CLANG_FORMAT_EXECUTABLE)
  clang_format_add_sources(${FILES})

  clang_format_add_targets()
endif()
#==============================================================================
# Installation
#
install(DIRECTORY "include"
  DESTINATION "${CATKIN_PACKAGE_INCLUDE_DESTINATION}"
)
install(TARGETS "${PROJECT_NAME}"
  ARCHIVE DESTINATION "${CATKIN_PACKAGE_LIB_DESTINATION}"
  LIBRARY DESTINATION "${CATKIN_PACKAGE_LIB_DESTINATION}"
  RUNTIME DESTINATION "${CATKIN_PACKAGE_BIN_DESTINATION}"
)
install(FILES "gen3_compliant_controllers_plugin.xml"
  DESTINATION "${CATKIN_PACKAGE_SHARE_DESTINATION}"
)
