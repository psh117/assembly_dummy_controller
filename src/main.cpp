#include <iostream>
#include <string>
#include <ros/ros.h>
#include "assembly_dummy_controller/vrep_bridge.h"
// #include "assembly_dummy_controller/controller.h"
#include "assembly_dummy_controller/linux_terminal_tool.h"
#include "assembly_dummy_controller/dummy_action_server.h"
#include "assembly_dummy_controller/dummy_joint_trajectory_action_server.h"
#include "assembly_dummy_controller/dummy_service_server.h"
#include <franka_gripper/GraspAction.h>
#include <franka_gripper/MoveAction.h>


#include <assembly_msgs/AssembleApproachAction.h>
#include <assembly_msgs/AssembleSpiralAction.h>
#include <assembly_msgs/AssembleExertForceAction.h>
#include <assembly_msgs/AssembleVerifyCompletionAction.h>
#include <assembly_msgs/AssembleParallelAction.h>
#include <assembly_msgs/AssembleMoveAction.h>
#include <assembly_msgs/IdleControl.h>
#include <assembly_msgs/SetSpiralGain.h>


using namespace std;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "assembly_dummy_controller");
  VRepBridge vb(VRepBridge::CTRL_POSITION); // Torque controlled
  // VRepBridge vb(VRepBridge::CTRL_POSITION); // Position controlled
  const double hz = 1000;
  // ArmController ac(hz);
  bool is_simulation_run = true;
  bool is_first = true;

  ros::NodeHandle node_handle;


  ros::ServiceServer idle_server = node_handle.advertiseService("/assembly_dual_controller/idle_control", 
  dummyServiceFunction<assembly_msgs::IdleControl>);

  ros::ServiceServer spiral_gain_server = node_handle.advertiseService("/assembly_dual_controller/assemble_spiral_control_gain_set", 
  dummyServiceFunction<assembly_msgs::SetSpiralGain>);

  DummyJointTrajectoryActionServer joint_trajectory_action_server
  ("/assembly_dual_controller/joint_trajectory_control", node_handle);

  DummyActionServer<assembly_msgs::AssembleApproachAction> assemble_approach_action_server
  ("/assembly_dual_controller/assemble_approach_control", node_handle);
  DummyActionServer<assembly_msgs::AssembleSpiralAction> assemble_spiral_action_server
  ("/assembly_dual_controller/assemble_spiral_control", node_handle);
  DummyActionServer<assembly_msgs::AssembleExertForceAction> assemble_insert_action_server
  ("/assembly_dual_controller/assemble_exert_force_control", node_handle);
  DummyActionServer<assembly_msgs::AssembleVerifyCompletionAction> assemble_verify_action_server
  ("/assembly_dual_controller/assemble_verify_completion_control", node_handle);
  DummyActionServer<assembly_msgs::AssembleParallelAction> assemble_parallel_action_server 
  ("/assembly_dual_controller/assemble_parallel_control", node_handle);
  DummyActionServer<assembly_msgs::AssembleExertForceAction> assemble_move_action_server
  ("/assembly_dual_controller/assemble_press_control", node_handle);


  DummyActionServer<franka_gripper::MoveAction> franka_gripper_move_left
  ("/panda_left_gripper/move", node_handle);
  DummyActionServer<franka_gripper::MoveAction> franka_gripper_move_right
  ("/panda_right_gripper/move", node_handle);
  DummyActionServer<franka_gripper::GraspAction> franka_gripper_grasp_left
  ("/panda_left_gripper/grasp", node_handle);
  DummyActionServer<franka_gripper::GraspAction> franka_gripper_grasp_right
  ("/panda_right_gripper/grasp", node_handle);

  while (vb.simConnectionCheck() && ros::ok())
  {
    ros::spinOnce();
    vb.read();
    // ac.readData(vb.getPosition(), vb.getVelocity());
    if (is_first)
    {
      vb.simLoop();
      vb.read();
      vb.setDesiredPosition(vb.getPosition());
      joint_trajectory_action_server.setDesiredPosition(vb.getPosition());
      // ac.readData(vb.getPosition(), vb.getVelocity());
      cout << "Initial q: " << vb.getPosition().transpose() << endl;
      is_first = false;
      // ac.initPosition();
    }

    // 	if (kbhit())
    // 	{
    // 		int key = getchar();
    // 		switch (key)
    // 		{
    // 			// Implement with user input
    //   MODE('i', "joint_ctrl_init")
    // 		case '\t':
    // 			if (is_simulation_run) {
    // 				cout << "Simulation Pause" << endl;
    // 				is_simulation_run = false;
    // 			}
    // 			else {
    // 				cout << "Simulation Run" << endl;
    // 				is_simulation_run = true;
    // 			}
    // 			break;
    // 		case 'q':
    // 			is_simulation_run = false;
    // 			exit_flag = true;
    // 			break;
    // 		default:
    // 			break;
    // 		}
    // 	}

    if (is_simulation_run)
    {
      // ac.compute();
      // vb.setDesiredPosition(ac.getDesiredPosition());
      // vb.setDesiredTorque(ac.getDesiredTorque());
      ros::Time time = ros::Time::now();
      joint_trajectory_action_server.compute(time);
      vb.setDesiredPosition(joint_trajectory_action_server.getDesiredPosition());
      vb.write();
      vb.simLoop();
    }
  }

  return 0;
}
