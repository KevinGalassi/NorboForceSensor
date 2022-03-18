cmake_minimum_required(VERSION 3.5)
project(nordbo_lrs6)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    nodelet
    std_srvs
    geometry_msgs
    message_generation
    std_msgs
    )


add_service_files(
    FILES
    setDataRate.srv
    startSensor.srv
    stopSensor.srv
    tareSensor.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs  
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES NordboLRS6
  CATKIN_DEPENDS message_runtime
)

