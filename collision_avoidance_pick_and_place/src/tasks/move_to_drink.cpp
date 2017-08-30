#include <collision_avoidance_pick_and_place/pick_and_place.h>

#include <moveit/trajectory_processing/iterative_time_parameterization.h>

/* MOVE ARM THROUGH DRINK POSES
  Goal:
    - Move the robot through the entire place motion.
    - Open gripper after reaching the release pose.
  Hints:
    - Use the methods seen so far such as "move", "sendGoal", "waitForResult" whenever needed.
*/

void collision_avoidance_pick_and_place::PickAndPlace::move_to_drink(std::vector<geometry_msgs::Pose>& place_poses,
    const geometry_msgs::Pose& box_pose)
{
  //ROS_ERROR_STREAM("place_box is not implemented yet.  Aborting."); exit(1);

  // task variables
  bool success;
  bool orientation_constrained = true;

  /* Fill Code:
   * Goal:
   * - Set the ReferenceFrame and EndEffectorLink
   * Hints:
   * - Use the "setEndEffectorLink" and "setPoseReferenceFrame" methods of "move_group_ptr"
   */
  move_group_ptr->setEndEffectorLink(cfg.WRIST_LINK_NAME);
  move_group_ptr->setPoseReferenceFrame(cfg.WORLD_FRAME_ID);

  move_group_ptr->setMaxVelocityScalingFactor(1.0);
  move_group_ptr->setMaxAccelerationScalingFactor(1.0);

//  move_group_ptr->setPlannerId("RRTConnectkConfigDefault");

  // set allowed planning time
  //move_group_ptr->setPlanningTime(20.0f);

  std::vector<geometry_msgs::Pose> waypoints;
  int count =0;

  // move the robot to each wrist place pose
  for(unsigned int i = 0; i < place_poses.size(); i++)
  {
    moveit_msgs::RobotState robot_state;    

    // attaching box
    show_box(true);
    set_attached_object(true,box_pose,robot_state);

    // create motion plan
    moveit::planning_interface::MoveGroup::Plan plan;
    while(!success) {
      count++;
      success = create_motion_plan(place_poses[i],robot_state,plan,orientation_constrained) && move_group_ptr->execute(plan);
      if(success)
      {
        ROS_INFO_STREAM("Drink Move " << i <<" Succeeded");
      }
      else
      {
        ROS_ERROR_STREAM("Drink Move " << i <<" Failed");
      }
      ROS_INFO_STREAM("Count: " << count);
    }
    success = false;
//    if (cfg.selected_object.compare("mug")== 0)
    orientation_constrained = false;

  }
}



