#pragma once

#include <string>
#include <actionlib/server/simple_action_server.h>

template<class ActionSpec>
class DummyActionServer
{
public:
  DummyActionServer(std::string name, ros::NodeHandle &nh) 
  : as_(nh, name, false)
  {
    as_.registerGoalCallback(boost::bind(&DummyActionServer::goalCallback, this));
    as_.registerPreemptCallback(boost::bind(&DummyActionServer::preemptCallback, this));
    as_.start();
  }
private:
  actionlib::SimpleActionServer<ActionSpec> as_;

  // ActionSpec::_action_goal_type::_goal_type::ConstPtr goal_;
  // ActionSpec::_action_feedback_type::_feedback_type feedback_;
  // ActionSpec::_action_result_type::_result_type result_;

  void goalCallback()
  {
    as_.acceptNewGoal();
    as_.setSucceeded();
  }

  void preemptCallback()
  {
    as_.setPreempted();
  }

  // assembly_msgs::AssembleMoveAction::_action_goal_type::_goal_type::ConstPtr goal_;
  // assembly_msgs::AssembleMoveAction::_action_feedback_type::_feedback_type feedback_;
  // assembly_msgs::AssembleMoveAction::_action_result_type::_result_type result_;
  // assembly_msgs::AssembleMoveAction::_action_goal_type::ConstPtr goal_;
};