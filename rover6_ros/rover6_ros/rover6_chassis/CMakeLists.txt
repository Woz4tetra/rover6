cmake_minimum_required(VERSION 2.8.3)
project(rover6_chassis)

find_package(catkin REQUIRED COMPONENTS
    rospy
    std_msgs
    message_generation
    tf
    nav_msgs
    dynamic_reconfigure
)

catkin_python_setup() # added if there is a setup.py. In this case for the node

# generate_messages(
#     DEPENDENCIES
#     std_msgs
# )

generate_dynamic_reconfigure_options(
    cfg/Rover6Chassis.cfg
)

catkin_package(
    CATKIN_DEPENDS rospy std_msgs
)

# add_library(${PROJECT_NAME}_node src/${PROJECT_NAME}_node.py)
# add_executable(${PROJECT_NAME}_node src/${PROJECT_NAME}_node.py)
# add_dependencies(${PROJECT_NAME}_node ${PROJECT_NAME}_gencfg)
# set_target_properties(${PROJECT_NAME}_node PROPERTIES LINKER_LANGUAGE py)
# set_target_properties(${PROJECT_NAME}_node PROPERTIES LINKER_LANGUAGE CMAKE_py_LINK_EXECUTABLE)
