cmake_minimum_required(VERSION 2.8.3)

project(drone)

find_package(catkin REQUIRED COMPONENTS 
    roscpp
    rospy
    std_msgs
    sensor_msgs
    image_transport
    gazebo_ros
    message_generation
    controller_manager 
    joint_state_controller 
    robot_state_publisher)

catkin_package(
   INCLUDE_DIRS include
   CATKIN_DEPENDS roscpp rospy image_transport std_msgs sensor_msgs geometry_msgs message_runtime gazebo_ros controller_manager joint_state_controller robot_state_publisher
)

find_package(roslaunch)
find_package(gazebo REQUIRED)
## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)
find_package(Protobuf REQUIRED)

link_directories(${GAZEBO_LIBRARY_DIRS} ${Boost_LIBRARIES} ${PROTOBUF_LIBRARIES} ${OGRE_LIBRARY_DIRS})
include_directories(${OpenCV_INCLUDE_DIR} 
                    ${Boost_INCLUDE_DIR}
                    ${catkin_INCLUDE_DIRS}
                    ${GAZEBO_INCLUDE_DIRS}
                    ${OGRE_INCLUDE_DIRS}
                    ${OGRE-Terrain_INCLUDE_DIRS}
                    include)

foreach(dir config launch meshes urdf)
	install(DIRECTORY ${dir}/
		DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")


install(DIRECTORY bin
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY include
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
  
install(DIRECTORY meshes
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY models
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY plugins
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
  
install(DIRECTORY scripts
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY src
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY worlds
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})


