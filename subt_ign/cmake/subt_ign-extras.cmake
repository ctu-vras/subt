find_package(ignition-transport10 REQUIRED)

list(APPEND catkin_INCLUDE_DIRS ${ignition-transport10_INCLUDE_DIRS})
list(APPEND catkin_LIBRARIES ${ignition-transport10_LIBRARIES} ${Protobuf_LIBRARIES})
