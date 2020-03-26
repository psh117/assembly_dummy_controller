#pragma once

#include <iostream>
#include <Eigen/Dense>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <actionlib/server/simple_action_server.h>
#include "assembly_dummy_controller/dyros_math.h"

class DummyJointTrajectoryActionServer
{
public:
  DummyJointTrajectoryActionServer() = delete;
  DummyJointTrajectoryActionServer(const DummyJointTrajectoryActionServer&) = delete;
  DummyJointTrajectoryActionServer(std::string name, ros::NodeHandle &nh);
  
public:
  // void readData(const Eigen::VectorXd &position, const Eigen::VectorXd &velocity, const Eigen::VectorXd &torque);
  // void readData(const Eigen::VectorXd &position, const Eigen::VectorXd &velocity);
  void setDesiredPosition(const Eigen::VectorXd & q)
  {
    q_desired_ = q;
  }
  
  const Eigen::VectorXd & getDesiredPosition()
  {
    return q_desired_;
  }
  
  bool compute(ros::Time time);
  bool computeArm(ros::Time time, std::string active_arm);

private:
  actionlib::SimpleActionServer<control_msgs::FollowJointTrajectoryAction> as_;
  std::vector<trajectory_msgs::JointTrajectoryPoint> joint_trajectory_;

  control_msgs::FollowJointTrajectoryFeedback feedback_;
  control_msgs::FollowJointTrajectoryResult result_;
  control_msgs::FollowJointTrajectoryGoalConstPtr goal_;

  void goalCallback();
  void preemptCallback();
  
  std::string active_arm_;
  std::vector<std::string> arm_names_{"panda_left_joint", "panda_right_joint"};
  ros::Time start_time_;
  int feedback_header_stamp_ {0}; 

  Eigen::VectorXd q_desired_ {14};
};
