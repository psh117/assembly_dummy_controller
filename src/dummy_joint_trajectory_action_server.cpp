
#include "assembly_dummy_controller/dummy_joint_trajectory_action_server.h"

DummyJointTrajectoryActionServer::DummyJointTrajectoryActionServer(std::string name, ros::NodeHandle &nh)
    : as_(nh, name, false)
{
  as_.registerGoalCallback(boost::bind(&DummyJointTrajectoryActionServer::goalCallback, this));
  as_.registerPreemptCallback(boost::bind(&DummyJointTrajectoryActionServer::preemptCallback, this));
  as_.start();
}

void DummyJointTrajectoryActionServer::goalCallback()
{
  feedback_header_stamp_ = 0;
  goal_ = as_.acceptNewGoal();

  if (goal_->trajectory.joint_names.size() == 0)
  {
    ROS_INFO("[DummyJointTrajectoryActionServer::goalCallback] Joint trajectory goal but no data has been received. Just passing it.");
    as_.setAborted();
    return;
  }

  ROS_INFO("[DummyJointTrajectoryActionServer::goalCallback] Joint trajectory goal has been received.");

  start_time_ = ros::Time::now();

  bool find_arm = false;
  for (auto iter = arm_names_.begin(); iter != arm_names_.end(); iter++)
  {
    // Find 'panda_left' in 'panda_left_joint1'
    if (goal_->trajectory.joint_names[0].find(*iter) != std::string::npos)
    {
      active_arm_ = *iter;
      find_arm = true;
      // int index = std::distance(arm_names_.begin(), std::find(arm_names_.begin(), arm_names_.end(), active_arm_));
      // std::cout << "arm name : " << *iter << std::endl;
      // std::cout << "arm name2 : " << goal_->trajectory.joint_names[0] << std::endl;
      // std::cout << "arm index: " << index << std::endl;
    }
  }

  if (find_arm == false)
  {
    ROS_ERROR("[DummyJointTrajectoryActionServer::goalCallback] the name %s is not in the arm list.", goal_->trajectory.joint_names[0].c_str());
    as_.setAborted();
    return;
  }

  auto as_joint_size = goal_->trajectory.points[0].positions.size();

  feedback_.joint_names.resize(as_joint_size);
  feedback_.actual.positions.resize(as_joint_size);
  feedback_.actual.velocities.resize(as_joint_size);
  feedback_.actual.accelerations.resize(as_joint_size);
}

void DummyJointTrajectoryActionServer::preemptCallback()
{
}

bool DummyJointTrajectoryActionServer::compute(ros::Time time)
{
  if (!as_.isActive())
      return false; 
  
  computeArm(time, active_arm_);
  return false;
}

bool DummyJointTrajectoryActionServer::computeArm(ros::Time time, std::string active_arm)
{
  int index = std::distance(arm_names_.begin(), std::find(arm_names_.begin(), arm_names_.end(), active_arm));
  Eigen::Matrix<double, 7, 1> q_desired, qd_desired, qdd_desired, tau_cmd;
  auto total_time = goal_->trajectory.points.back().time_from_start;
  ros::Duration passed_time = time - start_time_;

  Eigen::Vector3d position_now;

  bool value_updated = false;
  for (int j = 0; j < 7; j++){ 
    for (int i = 0; i < goal_->trajectory.points.size()-1; i++)
    {
      auto & cur_point = goal_->trajectory.points[i];
      auto & next_point = goal_->trajectory.points[i+1];
      if ((passed_time >= cur_point.time_from_start) && (passed_time <= next_point.time_from_start))
      {
        position_now = dyros_math::quinticSpline(passed_time.toSec(), cur_point.time_from_start.toSec(), next_point.time_from_start.toSec(),  
        cur_point.positions[j], cur_point.velocities[j], cur_point.accelerations[j], 
        next_point.positions[j], next_point.velocities[j], next_point.accelerations[j]);

        value_updated = true;
        break;
      }
    }

    if (value_updated == false)
    {
      position_now(0) = goal_->trajectory.points.back().positions[j];
      if(goal_->trajectory.points.back().velocities.size() == 0)
      {
        position_now(1) = 0.0;
        position_now(2) = 0.0;
      }
      else
      {
        position_now(1) = goal_->trajectory.points.back().velocities[j];
        position_now(2) = goal_->trajectory.points.back().accelerations[j];
      }
      
    }
    
    q_desired(j) = position_now(0);
    qd_desired(j) = position_now(1);
    qdd_desired(j) = position_now(2);

    feedback_.actual.positions[j] = position_now(0);
    feedback_.actual.velocities[j] = position_now(1);
    feedback_.actual.accelerations[j] = position_now(2);
  }

  feedback_.actual.time_from_start = passed_time;
  feedback_.header.seq=feedback_header_stamp_;
  feedback_header_stamp_++;
  as_.publishFeedback(feedback_);
  
  if(time.toSec() > (start_time_.toSec() +  total_time.toSec() + 0.5))
  {
    as_.setSucceeded();
  }

  q_desired_.segment<7>(index*7) = q_desired;

  return true;
}
