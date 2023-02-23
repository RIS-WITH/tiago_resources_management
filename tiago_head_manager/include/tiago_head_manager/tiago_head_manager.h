#include "tiago_head_manager_msgs/StateMachineRegister.h"
#include "tiago_head_manager_msgs/StateMachineExtract.h"
#include "tiago_head_manager_msgs/PitchYaw.h"
#include "tiago_head_manager_msgs/Point.h"
#include "tiago_head_manager_msgs/PrioritizedPitch.h"
#include "tiago_head_manager_msgs/PrioritizedYaw.h"
#include "tiago_head_manager_msgs/PrioritizedTrajectory.h"
#include "tiago_head_manager/ArtificialLife.h"

#include <resource_management/ReactiveInputs.h>
#include <resource_management/ResourceManager.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <thread>

#include <control_msgs/FollowJointTrajectoryAction.h>
#include <control_msgs/PointHeadAction.h>
#include <actionlib/client/simple_action_client.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/JointState.h>

#define NODE_NAME "TIAGO_HEAD_MANAGER"

class TiagoHeadManager : public resource_management::ResourceManager<tiago_head_manager_msgs::StateMachineRegister
      ,tiago_head_manager_msgs::StateMachineExtract
      ,tiago_head_manager_msgs::PitchYaw
      ,tiago_head_manager_msgs::Point
      ,tiago_head_manager_msgs::PrioritizedPitch
      ,tiago_head_manager_msgs::PrioritizedYaw
      ,tiago_head_manager_msgs::PrioritizedTrajectory
>
{
public:
    TiagoHeadManager(const ros::NodeHandlePtr &nh, std::vector<std::string>& plugins, bool synchronized = false):
        ResourceManager (std::move(nh),{"human_monitoring", "environment_monitoring", "acting", "speaking", "proximity"}, plugins, synchronized)
    {
        // this in lambda is necessary for gcc <= 5.1
        resource_management::MessageWrapper<tiago_head_manager_msgs::RawPitchYaw>::registerPublishFunction([this](auto data, auto is_new){ this->publishPitchYawMsg(data, is_new); });
        resource_management::MessageWrapper<geometry_msgs::PointStamped>::registerPublishFunction([this](auto data, auto is_new){ this->publishPointMsg(data, is_new); });
        resource_management::MessageWrapper<tiago_head_manager_msgs::Pitch>::registerPublishFunction([this](auto data, auto is_new){ this->publishPrioritizedPitchMsg(data, is_new); });
        resource_management::MessageWrapper<tiago_head_manager_msgs::Yaw>::registerPublishFunction([this](auto data, auto is_new){ this->publishPrioritizedYawMsg(data, is_new); });
        resource_management::MessageWrapper<trajectory_msgs::JointTrajectory>::registerPublishFunction([this](auto data, auto is_new){ this->publishPrioritizedTrajectoryMsg(data, is_new); });

        // Remove if your do not need artificial life
        _artificialLife = (std::make_shared<tiago_head_manager::ArtificialLife>(_artificialLifeBuffer));

	    point_head_client_ = new actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>(
            "/head_controller/follow_joint_trajectory", true);
    	joint_state_sub = _nh->subscribe("/joint_states", 1, &TiagoHeadManager::onJointState, this);

        ROS_INFO_STREAM(NODE_NAME << ": Waiting for head controller");
        point_head_client_->waitForServer();
        ROS_INFO_STREAM(NODE_NAME << ": Head controller found");
        tfListener = new tf2_ros::TransformListener(tfBuffer);
        watchdog = _nh->createTimer(ros::Duration(0.1), &TiagoHeadManager::onWatchDog, this);
        ROS_INFO_STREAM(NODE_NAME << ": Online.");
    }

    ~TiagoHeadManager()
    {
        delete point_head_client_;
        delete tfListener;
    }

private:
    std::map<std::string,std::shared_ptr<resource_management::MessageAbstraction>> stateFromMsg(const tiago_head_manager_msgs::StateMachineRegister::Request &msg) override;
    std::vector<std::tuple<std::string,std::string,resource_management_msgs::EndCondition>>
    transitionFromMsg(const tiago_head_manager_msgs::StateMachine &msg) override;
    tiago_head_manager_msgs::StateMachineRegister::Response generateResponseMsg(uint32_t id) override;

    void publishPitchYawMsg(tiago_head_manager_msgs::RawPitchYaw msg, bool is_new);
    void publishPointMsg(geometry_msgs::PointStamped msg, bool is_new);
    void publishPrioritizedPitchMsg(tiago_head_manager_msgs::Pitch msg, bool is_new);
    void publishPrioritizedYawMsg(tiago_head_manager_msgs::Yaw msg, bool is_new);
    void publishPrioritizedTrajectoryMsg(trajectory_msgs::JointTrajectory msg, bool is_new);

    void checkGoalActive();
    void onWatchDog(const ros::TimerEvent&);
    void onJointState(const sensor_msgs::JointStateConstPtr& msg);

    actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>* point_head_client_;
    ros::Subscriber joint_state_sub;
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener* tfListener;
    bool is_command_running_;
    ros::Timer watchdog;
    double current_tilt_, current_pan_;

    enum ePastDataType{
      NONE,
      RAW_PITCH_YAW,
      POINT,
      TRAJECTORY
    };
    ePastDataType past_data_type_;
};
