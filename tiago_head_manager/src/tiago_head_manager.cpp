#include "tiago_head_manager/tiago_head_manager.h"

std::map<std::string,std::shared_ptr<resource_management::MessageAbstraction>> TiagoHeadManager::stateFromMsg(const tiago_head_manager_msgs::StateMachineRegister::Request &msg)
{
    std::map<std::string,std::shared_ptr<resource_management::MessageAbstraction>> states;

    for(auto x : msg.state_machine.states_PitchYaw){
        auto wrap = states[x.header.id] = std::make_shared<resource_management::MessageWrapper<tiago_head_manager_msgs::RawPitchYaw>>(x.data);
        wrap->setPriority(static_cast<resource_management::importance_priority_t>(msg.header.priority.value));
    }

    for(auto x : msg.state_machine.states_Point){
        auto wrap = states[x.header.id] = std::make_shared<resource_management::MessageWrapper<geometry_msgs::PointStamped>>(x.data);
        wrap->setPriority(static_cast<resource_management::importance_priority_t>(msg.header.priority.value));
    }

    for(auto x : msg.state_machine.states_PrioritizedPitch){
        auto wrap = states[x.header.id] = std::make_shared<resource_management::MessageWrapper<tiago_head_manager_msgs::Pitch>>(x.data);
        wrap->setPriority(static_cast<resource_management::importance_priority_t>(msg.header.priority.value));
    }

    for(auto x : msg.state_machine.states_PrioritizedYaw){
        auto wrap = states[x.header.id] = std::make_shared<resource_management::MessageWrapper<tiago_head_manager_msgs::Yaw>>(x.data);
        wrap->setPriority(static_cast<resource_management::importance_priority_t>(msg.header.priority.value));
    }

    for(auto x : msg.state_machine.states_PrioritizedTrajectory){
        auto wrap = states[x.header.id] = std::make_shared<resource_management::MessageWrapper<trajectory_msgs::JointTrajectory>>(x.data);
        wrap->setPriority(static_cast<resource_management::importance_priority_t>(msg.header.priority.value));
    }

    return states;
}

std::vector<std::tuple<std::string,std::string,resource_management_msgs::EndCondition>>
TiagoHeadManager::transitionFromMsg(const tiago_head_manager_msgs::StateMachine &msg)
{
    std::vector<std::tuple<std::string,std::string,resource_management_msgs::EndCondition>> transitions;

    for(auto x : msg.states_PitchYaw)
        std::transform(x.header.transitions.begin(), x.header.transitions.end(),
                      std::back_inserter(transitions),
                      [x](auto& t){return std::make_tuple<std::string,std::string,resource_management_msgs::EndCondition>(
                                                          std::string(x.header.id),
                                                          std::string(t.next_state),
                                                          resource_management_msgs::EndCondition(t.end_condition));});

    for(auto x : msg.states_Point)
        std::transform(x.header.transitions.begin(), x.header.transitions.end(),
                      std::back_inserter(transitions),
                      [x](auto& t){return std::make_tuple<std::string,std::string,resource_management_msgs::EndCondition>(
                                                          std::string(x.header.id),
                                                          std::string(t.next_state),
                                                          resource_management_msgs::EndCondition(t.end_condition));});

    for(auto x : msg.states_PrioritizedPitch)
        std::transform(x.header.transitions.begin(), x.header.transitions.end(),
                      std::back_inserter(transitions),
                      [x](auto& t){return std::make_tuple<std::string,std::string,resource_management_msgs::EndCondition>(
                                                          std::string(x.header.id),
                                                          std::string(t.next_state),
                                                          resource_management_msgs::EndCondition(t.end_condition));});

    for(auto x : msg.states_PrioritizedYaw)
        std::transform(x.header.transitions.begin(), x.header.transitions.end(),
                      std::back_inserter(transitions),
                      [x](auto& t){return std::make_tuple<std::string,std::string,resource_management_msgs::EndCondition>(
                                                          std::string(x.header.id),
                                                          std::string(t.next_state),
                                                          resource_management_msgs::EndCondition(t.end_condition));});

    for(auto x : msg.states_PrioritizedTrajectory)
        std::transform(x.header.transitions.begin(), x.header.transitions.end(),
                      std::back_inserter(transitions),
                      [x](auto& t){return std::make_tuple<std::string,std::string,resource_management_msgs::EndCondition>(
                                                          std::string(x.header.id),
                                                          std::string(t.next_state),
                                                          resource_management_msgs::EndCondition(t.end_condition));});

    return transitions;
}

tiago_head_manager_msgs::StateMachineRegister::Response TiagoHeadManager::generateResponseMsg(uint32_t id)
{
  tiago_head_manager_msgs::StateMachineRegister::Response res;
  res.id = id;
  return res;
}

void TiagoHeadManager::publishPitchYawMsg(tiago_head_manager_msgs::RawPitchYaw msg, bool is_new)
{
  if (is_new == false)
        return;

    control_msgs::FollowJointTrajectoryGoal goal;
    goal.trajectory.joint_names.push_back("head_1_joint");
    goal.trajectory.joint_names.push_back("head_2_joint");
    goal.trajectory.points.resize(1);

    // Positions
    goal.trajectory.points[0].positions.resize(2);
    goal.trajectory.points[0].positions[0] = msg.yaw;
    goal.trajectory.points[0].positions[1] = msg.pitch;
    // Velocities
    goal.trajectory.points[0].velocities.resize(2);
    goal.trajectory.points[0].velocities[0] = 0.0;
    goal.trajectory.points[0].velocities[1] = 0.0;
    // Accelerations
    goal.trajectory.points[0].accelerations.resize(2);
    goal.trajectory.points[0].accelerations[0] = 0.0;
    goal.trajectory.points[0].accelerations[1] = 0.0;

    // To be reached 1 second after starting along the trajectory
    goal.trajectory.points[0].time_from_start = ros::Duration(1.0);

    goal.goal_time_tolerance = ros::Duration(1.0);

    past_data_type_ = ePastDataType::RAW_PITCH_YAW;

    point_head_client_->sendGoal(goal);
}

void TiagoHeadManager::publishPointMsg(geometry_msgs::PointStamped msg, bool is_new)
{
  if (is_new == false)
      return;

  // TODO: Make IK to check if the reference frame of the point has moved

  tf::Point target_in_root;
  geometry_msgs::TransformStamped torso2frame;
  try
  {
      msg.header.stamp = ros::Time(0);
      torso2frame = tfBuffer.lookupTransform("torso_lift_link", msg.header.frame_id, ros::Time(0));
  }
  catch(const tf::TransformException &ex)
  {
      ROS_ERROR("Transform failure: %s", ex.what());
      ROS_ERROR("Timestamp %f", msg.header.stamp.toSec());
      return;
  }
  geometry_msgs::PointStamped target_in_root_msg;
  tf2::doTransform(msg, target_in_root_msg, torso2frame);
  tf::pointMsgToTF(target_in_root_msg.point, target_in_root);
  target_in_root.setZ(target_in_root.getZ() - 0.3);
  target_in_root -= {-0.01707, 0, 0.38145};
  double pitch = -asin(target_in_root.getZ() / target_in_root.length());
  double yaw = atan2(target_in_root.getY(), target_in_root.getX());

  control_msgs::FollowJointTrajectoryGoal goal;
  goal.trajectory.joint_names.push_back("head_1_joint");
  goal.trajectory.joint_names.push_back("head_2_joint");
  goal.trajectory.points.resize(1);

  // Positions
  goal.trajectory.points[0].positions.resize(2);
  goal.trajectory.points[0].positions[0] = yaw;
  goal.trajectory.points[0].positions[1] = pitch;
  // Velocities
  goal.trajectory.points[0].velocities.resize(2);
  goal.trajectory.points[0].velocities[0] = 0.0;
  goal.trajectory.points[0].velocities[1] = 0.0;

  // To be reached 1 second after starting along the trajectory
  goal.trajectory.points[0].time_from_start = ros::Duration(1.0);

  goal.goal_time_tolerance = ros::Duration(1.0);

  past_data_type_ = ePastDataType::POINT;

  ROS_INFO_STREAM("New point : " << target_in_root.x() << "," << target_in_root.y() << "," << target_in_root.z() << "\tPitch: " << pitch <<"\tYaw: "<<yaw);

  point_head_client_->sendGoal(goal);
}

void TiagoHeadManager::publishPrioritizedPitchMsg(tiago_head_manager_msgs::Pitch msg, bool is_new)
{
  if (is_new == false)
    return;

  control_msgs::FollowJointTrajectoryGoal goal;
  goal.trajectory.joint_names.push_back("head_1_joint");
  goal.trajectory.joint_names.push_back("head_2_joint");
  goal.trajectory.points.resize(1);

  // Positions
  goal.trajectory.points[0].positions.resize(2);
  goal.trajectory.points[0].positions[0] = current_pan_;
  goal.trajectory.points[0].positions[1] = msg.pitch;
  // Velocities
  goal.trajectory.points[0].velocities.resize(2);
  goal.trajectory.points[0].velocities[0] = 0.0;
  goal.trajectory.points[0].velocities[1] = 0.0;
  // Accelerations
  goal.trajectory.points[0].accelerations.resize(2);
  goal.trajectory.points[0].accelerations[0] = 0.0;
  goal.trajectory.points[0].accelerations[1] = 0.0;

  // To be reached 1 second after starting along the trajectory
  goal.trajectory.points[0].time_from_start = ros::Duration(1.0);

  goal.goal_time_tolerance = ros::Duration(1.0);

  past_data_type_ = ePastDataType::RAW_PITCH_YAW;

  point_head_client_->sendGoal(goal);
}

void TiagoHeadManager::publishPrioritizedYawMsg(tiago_head_manager_msgs::Yaw msg, bool is_new)
{
  if (is_new == false)
    return;

  control_msgs::FollowJointTrajectoryGoal goal;
  goal.trajectory.joint_names.push_back("head_1_joint");
  goal.trajectory.joint_names.push_back("head_2_joint");
  goal.trajectory.points.resize(1);

  // Positions
  goal.trajectory.points[0].positions.resize(2);
  goal.trajectory.points[0].positions[0] = msg.yaw;
  goal.trajectory.points[0].positions[1] = current_tilt_;
  // Velocities
  goal.trajectory.points[0].velocities.resize(2);
  goal.trajectory.points[0].velocities[0] = 0.0;
  goal.trajectory.points[0].velocities[1] = 0.0;
  // Accelerations
  goal.trajectory.points[0].accelerations.resize(2);
  goal.trajectory.points[0].accelerations[0] = 0.0;
  goal.trajectory.points[0].accelerations[1] = 0.0;

  // To be reached 1 second after starting along the trajectory
  goal.trajectory.points[0].time_from_start = ros::Duration(1.0);

  goal.goal_time_tolerance = ros::Duration(1.0);

  past_data_type_ = ePastDataType::RAW_PITCH_YAW;

  point_head_client_->sendGoal(goal);
}

void TiagoHeadManager::publishPrioritizedTrajectoryMsg(trajectory_msgs::JointTrajectory msg, bool is_new)
{
  if (is_new == false)
    return;

  control_msgs::FollowJointTrajectoryGoal goal;
  goal.trajectory = msg;

  goal.goal_time_tolerance = ros::Duration(1.0);

  past_data_type_ = ePastDataType::TRAJECTORY;

  point_head_client_->sendGoal(goal);
}

void TiagoHeadManager::checkGoalActive()
{
  auto head_state_ = point_head_client_->getState().state_;
  if(is_command_running_)
  {
    if (head_state_ != actionlib::SimpleClientGoalState::ACTIVE)
    {
      this->done();
      is_command_running_ = false;
    }
  }
  else if (head_state_ == actionlib::SimpleClientGoalState::ACTIVE)
      is_command_running_ = true;
}

void TiagoHeadManager::onWatchDog(const ros::TimerEvent&)
{
  checkGoalActive();
}

void TiagoHeadManager::onJointState(const sensor_msgs::JointStateConstPtr& msg)
{
  bool tilt_found = false;
  bool pan_found = false;
  for (size_t i=0; i < msg->name.size(); i++)
  {
    if (msg->name[i] == "head_pan_joint")
    {
      current_pan_ = msg->position[i];
      pan_found = true;
    }
    else if (msg->name[i] == "head_tilt_joint")
    {
      current_tilt_ = msg->position[i];
      tilt_found = true;
    }

    if (tilt_found && pan_found)
      break;
  }
}

