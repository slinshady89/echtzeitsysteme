cmake_minimum_required(VERSION 2.8.3)
project(echtzeitsysteme)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
        genmsg
        cv_bridge
        roscpp
        std_msgs
        sensor_msgs
        geometry_msgs
        message_generation
        tf
        nav_msgs
        dynamic_reconfigure
        image_transport
        )

find_package(OpenCV REQUIRED)
#add dynamic reconfigure api
#find_package(Alglib REQUIRED)

#set(CMAKE_AUTOMOC ON)
#set(CMAKE_AUTOUIC ON)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a run_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
add_message_files(
        FILES
        points.msg
        #   Message2.msg
)

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
generate_messages(
        DEPENDENCIES
        geometry_msgs
        #std_msgs
)

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
generate_dynamic_reconfigure_options(
        cfg/controller.cfg
        cfg/ImageProcessing.cfg
        #...
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
        #INCLUDE_DIRS include
        #LIBRARIES pses_lane_detector
        CATKIN_DEPENDS geometry_msgs roscpp std_msgs message_runtime
)

###########
## Build ##
###########
#FIND_LIBRARY(ALG_LIB /usr/include/libalglib)
LINK_DIRECTORIES(/usr/lib/x86_64-linux-gnu/)
#message(STATUS /usr/include/libalglib)

set(ALGLIB_INCLUDES
        /usr/include/libalglib/
        )

#LINK_DIRECTORIES(${CMAKE_BINARY_DIR}/usr/lib/x86_64-linux-gnu)


## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
        include
        include/lane_detection
        ${OpenCV_INCLUDE_DIRS}
        ${catkin_INCLUDE_DIRS}
        ${CMAKE_CURRENT_BINARY_DIR}
        ${ALGLIB_INCLUDES}
)

#set(hdrs
#  include/lane_detector/lane_detector.h
#)

## Declare a C++ library
#add_library(lane_detector_project
#   src/lane_detector.cpp
#   ${hdrs}
#)

add_library(controller src/trajectory_planner/controller.cpp)
add_library(vehModel src/trajectory_planner/VehicleModel.cpp)
add_library(safety src/trajectory_planner/safety.cpp)
add_library(trajectory src/trajectory_planner/trajectory.cpp)
add_library(image_processing src/lane_detection/image_processor.cpp)


## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
#add_executable(lane_detector src/lane_detector_node.cpp)

add_executable(lane_detection src/lane_detection.cpp
        src/lane_detection/CameraReader.cpp
        src/lane_detection/image_processor.cpp
        src/gui/color_selector.cpp
        src/lane_detection/lane_points_calculator.cpp
        )
add_executable(trajectory_planning
        src/trajectory_planner/trajectory_planning.cpp)

add_executable(camera_reading_test src/lane_detection/camera_reading_test_node.cpp
        src/lane_detection/CameraReader.cpp
        src/lane_detection/image_processor.cpp
        src/gui/color_selector.cpp)

add_executable(show_multiple_lane_points
        proof_of_concept/src/show_multiple_lane_points.cpp
        src/lane_detection/image_processor.cpp
        src/lane_detection/lane_points_calculator.cpp)

add_executable(webcam_publisher src/input_reading/webcam_publisher.cpp
        src/lane_detection/CameraReader.cpp
        )
add_executable(image_publisher src/input_reading/image_publisher.cpp)

add_executable(image_viewer src/monitoring/image_viewer.cpp)


# make sure configure headers are built before any node using them
# add_dependencies(trajectory_planning ${PROJECT_NAME}_gencfg)


## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
#add_dependencies(pses_lane_detector pses_ucbridge _gencpp ${catkin_EXPORTED_TARGETS})

#add_dependencies(lane_detector_project ${catkin_EXPORTED_TARGETS})
add_dependencies(lane_detection ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(trajectory_planning ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against

#target_link_libraries(lane_detector_project
#)

#target_link_libraries(lane_detector
#  lane_detector_project
#  ${catkin_LIBRARIES}
#  ${QT_LIBRARIES}
#)

target_link_libraries(lane_detection
        image_processing
        ${catkin_LIBRARIES}
        ${OpenCV_LIBS}
        )

target_link_libraries(trajectory_planning
        vehModel
        controller
        safety
        trajectory
        ${catkin_LIBRARIES}
        alglib
        )

target_link_libraries(camera_reading_test
        ${catkin_LIBRARIES}
        ${OpenCV_LIBS}
        )

target_link_libraries(show_multiple_lane_points
        ${catkin_LIBRARIES}
        ${OpenCV_LIBS})

target_link_libraries(webcam_publisher
        ${catkin_LIBRARIES}
        ${OpenCV_LIBS})

target_link_libraries(image_publisher
        ${catkin_LIBRARIES}
        ${OpenCV_LIBS})

target_link_libraries(image_viewer
        ${catkin_LIBRARIES}
        ${OpenCV_LIBS})

############
## Install ##
############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_pses_lane_detector.cpp)

if (CATKIN_ENABLE_TESTING)
    find_package(rostest REQUIRED)
    catkin_add_gtest(firstTest test/firstTest.cpp)
    catkin_add_gtest(lane_points_calculator_tests test/lane_points_calculator_tests.cpp)
    target_link_libraries(firstTest ${catkin_LIBRARIES})
    target_link_libraries(lane_points_calculator_tests ${catkin_LIBRARIES})
endif ()

# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)

#############
## Enabling C++11##
#############
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if (COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
elseif (COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
else ()
    message(STATUS "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif ()



