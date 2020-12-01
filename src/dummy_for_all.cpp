#include <iostream>
#include <string>
#include <ros/ros.h>

#include "assembly_dummy_controller/linux_terminal_tool.h"
#include "assembly_dummy_controller/dummy_action_server.h"
#include "assembly_dummy_controller/dummy_service_server.h"

#include <franka_gripper/GraspAction.h>
#include <franka_gripper/MoveAction.h>

#include <assembly_msgs/AssembleApproachAction.h>
#include <assembly_msgs/AssembleSpiralAction.h>
#include <assembly_msgs/AssembleExertForceAction.h>
#include <assembly_msgs/AssembleVerifyCompletionAction.h>
#include <assembly_msgs/AssembleMoveAction.h>
#include <assembly_msgs/AssembleRotationAction.h>
#include <assembly_msgs/AssembleTripleRecoveryAction.h>
#include <assembly_msgs/AssembleDualArmSpiralAction.h>
#include <assembly_msgs/AssembleDualArmApproachAction.h>
#include <assembly_msgs/AssembleApproachBoltAction.h>
#include <assembly_msgs/AssembleRetreatBoltAction.h>
#include <assembly_msgs/AssembleProbeEdgeAction.h>
#include <assembly_msgs/AssembleTripleMoveAction.h>
#include <assembly_msgs/AssembleApproachHipAction.h>
#include <assembly_msgs/AssembleKittingAction.h>
#include <assembly_msgs/AssembleBackForthAction.h>
#include <assembly_msgs/AssembleBoltingReadyAction.h>
#include <assembly_msgs/TaskSpaceMoveAction.h>
#include <assembly_msgs/IdleControl.h>
#include <assembly_msgs/SetTrajectoryFollowerGain.h>
#include <assembly_dxl_gripper/Move.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>

#include <franka_msgs/ErrorRecoveryAction.h>
#include <franka_msgs/SetLoad.h>
#include <controller_manager_msgs/SwitchController.h>
#include <control_msgs/FollowJointTrajectoryAction.h>

using namespace std;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "assembly_dummy_controller");

  ros::NodeHandle node_handle;


  ros::ServiceServer idle_server = node_handle.advertiseService("/assembly_dual_controller/idle_control", 
  dummyServiceFunction<assembly_msgs::IdleControl>);

  ros::ServiceServer dxl_server = node_handle.advertiseService("/assembly_dxl_gripper/move", 
  dummyServiceFunction<assembly_dxl_gripper::Move>);

  ros::ServiceServer gain_server = node_handle.advertiseService("/assembly_dual_controller/joint_trajectory_control_gain_set", 
  dummyServiceFunction<assembly_msgs::SetTrajectoryFollowerGain>);

  ros::ServiceServer switch_controller_server = node_handle.advertiseService("/panda_dual/controller_manager/switch_controller", 
  dummyServiceFunction<controller_manager_msgs::SwitchController>);

  ros::ServiceServer set_load_server1 = node_handle.advertiseService("/panda_dual/panda_left/set_load", 
  dummyServiceFunction<franka_msgs::SetLoad>);

  ros::ServiceServer set_load_server2 = node_handle.advertiseService("/panda_dual/panda_right/set_load", 
  dummyServiceFunction<franka_msgs::SetLoad>);

  ros::ServiceServer set_load_server3 = node_handle.advertiseService("/panda_dual/panda_top/set_load", 
  dummyServiceFunction<franka_msgs::SetLoad>);

  DummyActionServer<franka_msgs::ErrorRecoveryAction> error_recovery_action_server
  ("/panda_dual/error_recovery", node_handle);

  DummyActionServer<assembly_msgs::AssembleApproachAction> assemble_approach_action_server
    ("/assembly_dual_controller/assemble_approach_control", node_handle);
    
  DummyActionServer<assembly_msgs::AssembleSpiralAction> assemble_spiral_action_server
    ("/assembly_dual_controller/assemble_spiral_control", node_handle);
    
  DummyActionServer<assembly_msgs::AssembleExertForceAction> assemble_insert_action_server
    ("/assembly_dual_controller/assemble_exert_force_control", node_handle);
    
  DummyActionServer<assembly_msgs::AssembleVerifyCompletionAction> assemble_verify_action_server
    ("/assembly_dual_controller/assemble_verify_completion_control", node_handle);
    
  DummyActionServer<control_msgs::FollowJointTrajectoryAction> joint_trajectory_action_server
    ("/assembly_dual_controller/joint_trajectory_control", node_handle);
    
  DummyActionServer<assembly_msgs::AssembleMoveAction> assemble_move_action_server
    ("/assembly_dual_controller/assemble_move_control", node_handle);
    
  DummyActionServer<assembly_msgs::AssembleDualArmSpiralAction> assemble_dual_spiral_action_server
    ("/assembly_dual_controller/assemble_dual_spiral_control", node_handle);
    
  DummyActionServer<assembly_msgs::AssembleDualArmApproachAction> assemble_dual_approach_action_server
    ("/assembly_dual_controller/assemble_dual_approach_control", node_handle);
    
  DummyActionServer<assembly_msgs::AssembleRotationAction> assemble_rotation_action_server
    ("/assembly_dual_controller/assemble_rotation_control", node_handle);
    
  DummyActionServer<assembly_msgs::AssembleTripleRecoveryAction> assemble_triple_recovery_action_server
    ("/assembly_dual_controller/assemble_triple_recovery_control", node_handle);
    
  DummyActionServer<assembly_msgs::AssembleApproachBoltAction> assemble_approach_bolt_action_server
    ("/assembly_dual_controller/assemble_approach_bolt_control", node_handle);
    
  DummyActionServer<assembly_msgs::AssembleRetreatBoltAction> assemble_retreat_bolt_action_server
    ("/assembly_dual_controller/assemble_retreat_bolt_control", node_handle);  
    
  DummyActionServer<assembly_msgs::TaskSpaceMoveAction> task_space_move_action_server
    ("/assembly_dual_controller/task_space_move", node_handle);
    
  DummyActionServer<assembly_msgs::AssembleProbeEdgeAction> assemble_probe_edge_action_server
    ("/assembly_dual_controller/assemble_probe_edge_control", node_handle);
    
  DummyActionServer<assembly_msgs::AssembleTripleMoveAction> assemble_triple_move_action_server
    ("/assembly_dual_controller/assemble_triple_move_control", node_handle);
    
  DummyActionServer<assembly_msgs::AssembleApproachHipAction> assemble_approach_hip_action_server
    ("/assembly_dual_controller/assemble_approach_hip_control", node_handle);
    
  DummyActionServer<assembly_msgs::AssembleKittingAction> assemble_kitting_action_server
    ("/assembly_dual_controller/assemble_kitting_control", node_handle);
    
  DummyActionServer<assembly_msgs::AssembleBackForthAction> assemble_back_forth_action_server
    ("/assembly_dual_controller/assemble_back_forth_control", node_handle);
    
  DummyActionServer<assembly_msgs::AssembleBoltingReadyAction> assemble_bolting_ready_action_server
    ("/assembly_dual_controller/assemble_bolting_ready_control", node_handle);


  DummyActionServer<franka_gripper::MoveAction> franka_gripper_move_left
    ("/panda_left_gripper/move", node_handle);
  DummyActionServer<franka_gripper::GraspAction> franka_gripper_grasp_left
    ("/panda_left_gripper/grasp", node_handle);

  ros::spin();

  return 0;
}
