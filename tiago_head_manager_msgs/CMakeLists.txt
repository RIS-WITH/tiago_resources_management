cmake_minimum_required(VERSION 2.8.3)
project(tiago_head_manager_msgs)

find_package(catkin REQUIRED
  roscpp
  message_generation
  resource_management_msgs
  trajectory_msgs
  geometry_msgs
)

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  StateMachineStatePitchYaw.msg
  StateMachineStatePoint.msg
  StateMachineStatePrioritizedPitch.msg
  StateMachineStatePrioritizedYaw.msg
  StateMachineStatePrioritizedTrajectory.msg
  PitchYaw.msg
  Point.msg
  PrioritizedPitch.msg
  PrioritizedYaw.msg
  PrioritizedTrajectory.msg
  Pitch.msg
  Yaw.msg
  RawPitchYaw.msg
  StateMachine.msg)

## Generate services in the 'srv' folder
add_service_files(
  FILES
  StateMachineRegister.srv
  StateMachineExtract.srv
)

generate_messages(DEPENDENCIES resource_management_msgs trajectory_msgs geometry_msgs)

catkin_package(CATKIN_DEPENDS message_runtime trajectory_msgs geometry_msgs)
