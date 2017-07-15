#include <collision_avoidance_pick_and_place/pick_and_place.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

/* MOVING ARM TO WAIT POSITION
  Goal:
    - Use the "move_group" interface to move the robot to the "wait" target.
    - Observe how we verify that the move was completed

  Hints:
    - "cfg.WAIT_POSE_NAME" contains the name of the wait target.
    - Once the target is set you can call the "move" method in order to go to that target.
*/

void collision_avoidance_pick_and_place::PickAndPlace::move_to_wait_position()
{
  //ROS_ERROR_STREAM("move_to_wait_position is not implemented yet.  Aborting."); exit(1);

  // task variables
  bool success; // saves the move result

  /* Fill Code:
   * Goal:
   * - Set robot wait target
   * Hints:
   * - Use the "setNamedTarget" method in the "move_group" object.
   * - Look in the "cfg.WAIT_POSE_NAME" object for the name of the target.
   */
  move_group_ptr->setNamedTarget(cfg.WAIT_POSE_NAME);

  // set allowed planning time
  move_group_ptr->setPlanningTime(60.0f);

  /* Fill Code:
   * Goal:
   * - Move the robot
   * Hints:
   * - Use the "move" method in the "move_group" object and save the result
   *  in the "success" variable
   */
  success = move_group_ptr->move();
  if(success)
  {
    ROS_INFO_STREAM("Move " << cfg.WAIT_POSE_NAME<< " Succeeded");
  }
  else
  {
    ROS_ERROR_STREAM("Move " << cfg.WAIT_POSE_NAME<< " Failed");
    exit(1);
  }


//  ros::NodeHandle node_handle;


//  moveit::planning_interface::MoveGroup group(cfg.ARM_GROUP_NAME);


//  group.setPlanningTime(60.0f);

//    geometry_msgs::Pose target_pose1;
//    target_pose1.orientation.w = 1.0;
//    target_pose1.position.x = 0.28;
//    target_pose1.position.y = -0.2;
//    target_pose1.position.z = 1.0;
//    group.setPoseTarget(target_pose1);
//    group.move();

//    moveit::planning_interface::MoveGroup::Plan my_plan;
//    bool succes = group.plan(my_plan);

//    ROS_INFO("Visualizing plan 1 (pose goal) %s",succes?"":"FAILED");
//    /* Sleep to give Rviz time to visualize the plan. */
//    // Planning with Path Constraints
//    // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
//    //
//    // Path constraints can easily be specified for a link on the robot.
//    // Let's specify a path constraint and a pose goal for our group.
//    // First define the path constraint.
//    moveit_msgs::OrientationConstraint ocm;
//    ocm.link_name = cfg.WRIST_LINK_NAME;
//    ocm.header.frame_id = "world_frame";
//    ocm.orientation.w = 1.0;
//    ocm.absolute_x_axis_tolerance = 0.1;
//    ocm.absolute_y_axis_tolerance = 0.1;
//    ocm.absolute_z_axis_tolerance = 0.1;
//    ocm.weight = 1.0;

//    // Now, set it as the path constraint for the group.
//    moveit_msgs::Constraints test_constraints;
//    test_constraints.orientation_constraints.push_back(ocm);
//    group.setPathConstraints(test_constraints);

//    // We will reuse the old goal that we had and plan to it.
//    // Note that this will only work if the current state already
//    // satisfies the path constraints. So, we need to set the start
//    // state to a new pose.
//    robot_state::RobotState start_state(*group.getCurrentState());
//    geometry_msgs::Pose start_pose2;
//    start_pose2.orientation.w = 1.0;
//    start_pose2.position.x = 0.55;
//    start_pose2.position.y = -0.05;
//    start_pose2.position.z = 0.8;
//       const robot_state::JointModelGroup *joint_model_group =
//                       start_state.getJointModelGroup(group.getName());
//       start_state.setFromIK(joint_model_group, start_pose2);
//       group.setStartState(start_state);


//    // Now we will plan to the earlier pose target from the new
//    // start state that we have just created.
//    group.setPoseTarget(target_pose1);
//    succes = group.plan(my_plan);

//    ROS_INFO("Visualizing plan 3 (constraints) %s",succes?"":"FAILED");
//    group.move();
}


