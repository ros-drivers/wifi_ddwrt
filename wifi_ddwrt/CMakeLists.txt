# http://ros.org/doc/groovy/api/catkin/html/user_guide/supposed.html
cmake_minimum_required(VERSION 2.8.3)
project(wifi_ddwrt)

find_package(catkin REQUIRED COMPONENTS message_generation std_msgs)

add_message_files(
  FILES
  Network.msg
  SiteSurvey.msg
)

generate_messages(DEPENDENCIES std_msgs)

catkin_package(CATKIN_DEPENDS std_msgs)

install(PROGRAMS 
   nodes/ddwrt.py
   scripts/analysis
   scripts/ddwrt_sim.py
   scripts/monitor
   scripts/ddwrt_config.py
   scripts/wanip.py
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
