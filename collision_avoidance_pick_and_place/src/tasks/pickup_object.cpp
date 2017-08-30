#include <collision_avoidance_pick_and_place/pick_and_place.h>

/* MOVE ARM THROUGH PICK POSES
  Goal:
    - Use the "move_group" object to set the wrist as the end-effector link
    - Use the "move_group" object to set the planning reference frame.
    - Move the robot through the pick motion.
    - Close the gripper when appropriate.

  Hints:
    - The "move_group" interface has useful methods such as "setEndEffectorLink"
    	and "setPoseReferenceFrame" that can be used to prepare the robot for planning.
    - The "setPoseTarget" method allows you to set a "pose" as your target
    	to move the robot.
*/
void collision_avoidance_pick_and_place::PickAndPlace::pickup_object(std::vector<geometry_msgs::Pose>& pick_poses)
{
    //ROS_ERROR_STREAM("pickup_box is not implemented yet.  Aborting."); exit(1);

	  // task variables
    bool success = false;


	  /* Fill Code:
	   * Goal:
	   * - Set the wrist as the end-effector link
	     	 - The robot will try to move this link to the specified pose
	     	 - If not specified, moveit will use the last link in the arm group.
	   * Hints:
	   * - Use the "setEndEffectorLink" in the "move_group_ptr" object.
	   * - The WRIST_LINK_NAME field in the "cfg" configuration member contains
	   * 	the name for the arm's wrist link.
	   */

    move_group_ptr->setEndEffectorLink(cfg.WRIST_LINK_NAME);

	  // set allowed planning time
    //move_group_ptr->setPlanningTime(1.0f);


	  /* Fill Code:
	   * Goal:
	   * - Set world frame as the reference
	    	- The target position is specified relative to this frame
	   	  	- If not specified, MoveIt will use the parent frame of the SRDF "Virtual Joint"
	   * Hints:
	   * - Use the "setPoseReferenceFrame" in the "move_group_ptr" object.
	   * - The WORLD_FRAME_ID in the "cfg" configuration member contains the name
	   * 	for the reference frame.
	   */
    move_group_ptr->setPoseReferenceFrame(cfg.WORLD_FRAME_ID);
    //move_group_ptr->setMaxVelocityScalingFactor(0.1);
    //move_group_ptr->setMaxAccelerationScalingFactor(0.1);

	  // move the robot to each wrist pick pose
	  for(unsigned int i = 0; i < pick_poses.size(); i++)
	  {
	  	moveit_msgs::RobotState robot_state;

     /* - Look in the "set_attached_object()" method to understand
        how to attach a payload using moveit.  */
      set_attached_object(false,geometry_msgs::Pose(),robot_state);

      /* - Look in the "create_motion_plan()" method to observe how an
        entire moveit motion plan is created.  */
      moveit::planning_interface::MoveGroup::Plan plan;      

      while(!success) {

        success = create_motion_plan(pick_poses[i],robot_state,plan,0) && move_group_ptr->execute(plan);
        if(success)
        {
          ROS_INFO_STREAM("Pick Move " << i <<" Succeeded");
        }
        else
        {
          ROS_ERROR_STREAM("Pick Move " << i <<" Failed");
        }
      }
      success = false;
	    // verifying move completion
//	    if(success)
//	    {
//	      ROS_INFO_STREAM("Pick Move " << i <<" Succeeded");
//        ROS_INFO_STREAM("pick position: " << pick_poses[i].position);
//	    }
//	    else
//	    {
//	      ROS_ERROR_STREAM("Pick Move " << i <<" Failed");
//        bool again = create_motion_plan(pick_poses[i],robot_state,plan,0) && move_group_ptr->execute(plan);
//        if (again)
//          continue;
//        else
//        {
//          ROS_ERROR_STREAM("Again Pick Move " << i <<" Failed");
//          set_gripper(false);
//          exit(1);
//        }
//	    }

	    if(i == 0)
      {
     //Turn on gripper suction after approach pose is reached.
       set_gripper(true);

	    }

	  }

}
