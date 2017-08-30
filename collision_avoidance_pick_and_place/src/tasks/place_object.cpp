#include <collision_avoidance_pick_and_place/pick_and_place.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>

/* MOVE ARM THROUGH PLACE POSES
  Goal:
    - Move the robot through the entire place motion.
    - Open gripper after reaching the release pose.
  Hints:
    - Use the methods seen so far such as "move", "sendGoal", "waitForResult" whenever needed.
*/

void collision_avoidance_pick_and_place::PickAndPlace::place_object(std::vector<geometry_msgs::Pose>& place_poses,
		const geometry_msgs::Pose& box_pose)
{
  //ROS_ERROR_STREAM("place_box is not implemented yet.  Aborting."); exit(1);

  // task variables
  bool success;

  /* Fill Code:
   * Goal:
   * - Set the ReferenceFrame and EndEffectorLink
   * Hints:
   * - Use the "setEndEffectorLink" and "setPoseReferenceFrame" methods of "move_group_ptr"
   */
  move_group_ptr->setEndEffectorLink(cfg.WRIST_LINK_NAME);
  move_group_ptr->setPoseReferenceFrame(cfg.WORLD_FRAME_ID);

  // set allowed planning time, default is 1.0s
  //move_group_ptr->setPlanningTime(1.0f);
  //move_group_ptr->setMaxVelocityScalingFactor(0.1);
  //move_group_ptr->setMaxAccelerationScalingFactor(0.1);



  // move the robot to each wrist place pose
  for(unsigned int i = 0; i < place_poses.size(); i++)
  {
    std::vector<geometry_msgs::Pose> waypoints;
  	moveit_msgs::RobotState robot_state;
  	if(i==0 || i == 1)
  	{
      // attaching box
      set_attached_object(true,box_pose,robot_state);
      show_box(true);

  	}
  	else
  	{
      // detaching box
      set_attached_object(false,geometry_msgs::Pose(),robot_state);
      show_box(false);
  	}
    waypoints.push_back(place_poses[i]);

  	// create motion plan
    moveit::planning_interface::MoveGroup::Plan plan;
    //success = create_motion_plan(place_poses[i],robot_state,plan,1) && move_group_ptr->execute(plan);

    moveit_msgs::RobotTrajectory trajectory_msg;

    double fraction = move_group_ptr->computeCartesianPath(waypoints,
                                                 0.01,  // eef_step
                                                 0.0,   // jump_threshold
                                                 trajectory_msg, false);

    // The trajectory needs to be modified so it will include velocities as well.
    // First to create a RobotTrajectory object
    robot_trajectory::RobotTrajectory rt(move_group_ptr->getCurrentState()->getRobotModel(), "manipulator");

    // Second get a RobotTrajectory from trajectory
    rt.setRobotTrajectoryMsg(*move_group_ptr->getCurrentState(), trajectory_msg);

    // Thrid create a IterativeParabolicTimeParameterization object
    trajectory_processing::IterativeParabolicTimeParameterization iptp;

    // Fourth compute computeTimeStamps
    bool succes = iptp.computeTimeStamps(rt);
    ROS_INFO("Computed time stamp %s",succes?"SUCCEDED":"FAILED");

    // Get RobotTrajectory_msg from RobotTrajectory
    rt.getRobotTrajectoryMsg(trajectory_msg);

    // Finally plan and execute the trajectory
    plan.trajectory_ = trajectory_msg;
    ROS_INFO("Visualizing plan 4 (cartesian path) (%.2f%% acheived)",fraction * 100.0);
    move_group_ptr->execute(plan);

    if(fraction == 1.0)
    //if(success)
    {
      ROS_INFO_STREAM("Place Move " << i <<" Succeeded");
    }
    else
    {
      ROS_ERROR_STREAM("Place Move " << i <<" Failed");
      set_gripper(false);
      exit(1);
    }


    if(i == 1)
    {
	/* Fill Code:
	 * Goal:
	 * - Turn off gripper suction after the release pose is reached.
	 * Hints:
	 * - Call the "set_gripper" function to turn on suction.
	 * - The input to the set_gripper method takes a "true" or "false"
	 * 	  boolean argument.
	 */
      set_gripper(false);

    }

  }
}



