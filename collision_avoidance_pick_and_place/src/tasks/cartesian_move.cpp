#include <collision_avoidance_pick_and_place/pick_and_place.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>

/* MOVE ARM THROUGH PLACE POSES
  Goal:
    - Move the robot through the entire place motion.
    - Open gripper after reaching the release pose.
  Hints:
    - Use the methods seen so far such as "move", "sendGoal", "waitForResult" whenever needed.
*/

void collision_avoidance_pick_and_place::PickAndPlace::cartesian_move()
{
  std::vector<geometry_msgs::Pose> cartesian_poses;
  tf::StampedTransform bowl_tf, still_tf;
  tf::StampedTransform tcp_to_wrist_tf;
  std::vector<geometry_msgs::Pose> tcp_place_poses, wrist_place_poses;

  try
  {
    transform_listener_ptr->waitForTransform("world_frame", "bowl", ros::Time(0.0f), ros::Duration(3.0f));
    transform_listener_ptr->lookupTransform("world_frame", "bowl", ros::Time(0.0f), bowl_tf);

  }
  catch (tf::TransformException ex){
    ROS_ERROR("%s",ex.what());
  }


  tf::Transform pre_pour_tf, world_to_tcp_tf;
  geometry_msgs::Pose pre_pour_pose, pose;
  tf::poseTFToMsg(bowl_tf, pre_pour_pose);
  ROS_INFO_STREAM("Bowl pose: " << pre_pour_pose);
  // Setting pour position, right of bowl
  pre_pour_pose.position.z = pre_pour_pose.position.z + 0.2;
  pre_pour_pose.position.y = pre_pour_pose.position.y - 0.2;
  pre_pour_pose.position.x = pre_pour_pose.position.x - cfg.CEREAL_SIZE[2]/2;
  ROS_INFO_STREAM("Pour pose: " << pre_pour_pose);
  tf::poseMsgToTF(pre_pour_pose, pre_pour_tf);
  try{
    transform_listener_ptr->waitForTransform("world_frame", "still_tf", ros::Time(0.0f), ros::Duration(3.0f));
    transform_listener_ptr->lookupTransform("world_frame", "still_tf", ros::Time(0.0f), still_tf);

  }
  catch (tf::TransformException ex){
    ROS_ERROR("%s",ex.what());
  }

  world_to_tcp_tf.setOrigin(pre_pour_tf.getOrigin());
  world_to_tcp_tf.setRotation(still_tf.getRotation());
  transform_broadcaster->sendTransform(tf::StampedTransform(world_to_tcp_tf, ros::Time::now(), "world_frame", "pre_pour"));
  tf::poseTFToMsg(world_to_tcp_tf, pose);
  cartesian_poses.push_back(pose);

  move_group_ptr->setEndEffectorLink(cfg.WRIST_LINK_NAME);
  move_group_ptr->setPoseReferenceFrame(cfg.WORLD_FRAME_ID);

  // set allowed planning time, default is 1.0s
  //move_group_ptr->setPlanningTime(1.0f);
  move_group_ptr->setMaxVelocityScalingFactor(1.0);
  move_group_ptr->setMaxAccelerationScalingFactor(1.0);

  transform_listener_ptr->waitForTransform(cfg.TCP_LINK_NAME, cfg.WRIST_LINK_NAME, ros::Time(0.0f), ros::Duration(3.0f));
  transform_listener_ptr->lookupTransform(cfg.TCP_LINK_NAME, cfg.WRIST_LINK_NAME, ros::Time(0.0f), tcp_to_wrist_tf);

  wrist_place_poses = transform_from_tcp_to_wrist(tcp_to_wrist_tf, cartesian_poses);

  // move the robot to each wrist place pose
  for(unsigned int i = 0; i < wrist_place_poses.size(); i++)
  {
    std::vector<geometry_msgs::Pose> waypoints;

    waypoints.push_back(wrist_place_poses[i]);

    moveit::planning_interface::MoveGroup::Plan plan;
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
    bool success = iptp.computeTimeStamps(rt);
    ROS_INFO("Computed time stamp %s",success?"SUCCEDED":"FAILED");

    // Get RobotTrajectory_msg from RobotTrajectory
    rt.getRobotTrajectoryMsg(trajectory_msg);

    // Finally plan and execute the trajectory
    plan.trajectory_ = trajectory_msg;
    ROS_INFO("Visualizing plan 4 (cartesian path) (%.2f%% acheived)",fraction * 100.0);
    move_group_ptr->execute(plan);

    if(fraction == 1.0)
    {
      ROS_INFO_STREAM("Place Move " << i <<" Succeeded");
    }
    else
    {
      ROS_ERROR_STREAM("Place Move " << i <<" Failed");
      set_gripper(false);
      exit(1);
    }



  }
}



