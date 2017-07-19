#include <collision_avoidance_pick_and_place/pick_and_place.h>

/* CREATE MOTION PLAN
  Goal:
    - Creates a motion plan request using the desired end-effector pose and the current
    	robot state (joint configuration and payload status)
    - Calls the moveit motion planning service and returns the motion plan if a valid one is
    	found.

  Hints:

*/

namespace collision_avoidance_pick_and_place
{

bool PickAndPlace::create_motion_plan(const geometry_msgs::Pose &pose_target,
    const moveit_msgs::RobotState &start_robot_state,moveit::planning_interface::MoveGroup::Plan &plan, bool drink)
{
	// constructing motion plan goal constraints
  std::vector<double> position_tolerances(3,0.03f);
  std::vector<double> orientation_tolerances(3,0.03f);
	geometry_msgs::PoseStamped p;
	p.header.frame_id = cfg.WORLD_FRAME_ID;
	p.pose = pose_target;
  moveit_msgs::Constraints pose_goal = kinematic_constraints::constructGoalConstraints(cfg.WRIST_LINK_NAME,p,position_tolerances,
      orientation_tolerances);

  moveit_msgs::Constraints path_constraints;

  if (drink){
    path_constraints= create_path_orientation_constraints(p.pose, 0.3,0.3,0.3,cfg.WRIST_LINK_NAME);
    //move_group_ptr->setPathConstraints(path_constraints);
  }
  // creating motion plan request
  moveit_msgs::GetMotionPlan motion_plan;
  moveit_msgs::MotionPlanRequest &req = motion_plan.request.motion_plan_request;
  moveit_msgs::MotionPlanResponse &res = motion_plan.response.motion_plan_response;
  req.start_state = start_robot_state;
  req.start_state.is_diff = true; //related to clearing attached object
  req.group_name = cfg.ARM_GROUP_NAME;
  req.goal_constraints.push_back(pose_goal);
  req.allowed_planning_time = 4.0f;
  req.num_planning_attempts = 8;
  req.planner_id = "RRTConnectkConfigDefault";


  if(drink){
    req.path_constraints = path_constraints;
    //req.goal_constraints.push_back(path_constraints);
  }



	// request motion plan
	bool success = false;
  if(motion_plan_client.call(motion_plan) && res.error_code.val == res.error_code.SUCCESS)
	{    

		// saving motion plan results
		plan.start_state_ = res.trajectory_start;
		plan.trajectory_ = res.trajectory;
		success = true;
	}

	return success;
}

}




