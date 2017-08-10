#include <collision_avoidance_pick_and_place/pick_and_place.h>

/* CREATE PICK MOVES
  Goal:
    - Set the pose for the tcp at the box pick.
    - Create tcp poses for the pick motions (approach, target, retreat).
    - Find transform of the wrist in tcp coordinates
    - Convert tcp pick poses to wrist poses.
         * Moveit's kinematics require the target position to be specified relative to
           one of the kinematic links of the manipulator arm (as defined in the SRDF)

  Hints:
    - You can manipulate the "world_to_tcp_tf" transform through the "setOrigin" and "setRotation".
    - Look into the "create_manipulation_poses" function and observe how each pick pose is created.
    - Use the "transform_from_tcp_to_wrist" function to populate the "wrist_pick_poses" array.
*/

std::vector<geometry_msgs::Pose> collision_avoidance_pick_and_place::PickAndPlace::create_pick_moves(geometry_msgs::Pose &box_pose)
{
  //ROS_ERROR_STREAM("create_pick_moves is not implemented yet.  Aborting."); exit(1);

  // task variables
  tf::Transform world_to_tcp_tf;
  tf::Transform world_to_box_tf;
  tf::StampedTransform tcp_to_wrist_tf;
  std::vector<geometry_msgs::Pose> tcp_pick_poses, wrist_pick_poses;  

  /* Fill Code:
   * Goal:
   * - Create tcp pose at box pick.
   * Hints:
   * - Use the "setOrigin" to set the position of "world_to_tcp_tf".
   */
  tf::poseMsgToTF(box_pose,world_to_box_tf);
  tf::Vector3 box_position(box_pose.position.x, box_pose.position.y, box_pose.position.z);
  world_to_tcp_tf.setOrigin(box_position);


  /* Setting tcp orientation - lines up the ee to object pose
	   * Inverting the approach direction so that the tcp points towards the box instead of
	   * away from it.*/
  world_to_tcp_tf.setRotation(world_to_box_tf.getRotation()* tf::Quaternion(tf::Vector3(1,0,0),M_PI));
  transform_broadcaster->sendTransform(tf::StampedTransform(world_to_tcp_tf, ros::Time::now(), "world_frame", "world_to_tcp"));


  // create all the poses for tcp's pick motion (approach, pick and retreat)
  tcp_pick_poses = create_pick_poses(cfg.RETREAT_DISTANCE, cfg.APPROACH_DISTANCE, world_to_tcp_tf);
  if (tcp_pick_poses.size() == 0){
    return tcp_pick_poses;
  }
  /* Fill Code:
   * Goal:
   * - Find transform from tcp to wrist.
   * Hints:
   * - Use the "lookupTransform" method in the transform listener.
   */
  transform_listener_ptr->waitForTransform(cfg.TCP_LINK_NAME, cfg.WRIST_LINK_NAME,ros::Time(0.0f),ros::Duration(3.0f));
  transform_listener_ptr->lookupTransform(cfg.TCP_LINK_NAME, cfg.WRIST_LINK_NAME,ros::Time(0.0f),tcp_to_wrist_tf);


  /* Fill Code:
   * Goal:
   * - Transform list of pick poses from tcp frame to wrist frame
   * Hint:
   * - Use the "transform_from_tcp_to_wrist" function and save results into
   * 	"wrist_pick_poses".
   * - The "tcp_to_wrist_tf" is the transform that will help convert "tcp_pick_poses"
   * 	into "wrist_pick_poses".
   */
  wrist_pick_poses = transform_from_tcp_to_wrist(tcp_to_wrist_tf, tcp_pick_poses);

  // printing some results
  ROS_INFO_STREAM("tcp position at pick: " << world_to_tcp_tf.getOrigin());
  ROS_INFO_STREAM("tcp z direction at pick: " << world_to_tcp_tf.getBasis().getColumn(2));
  ROS_INFO_STREAM("wrist position at pick: " << wrist_pick_poses[1].position);


  return wrist_pick_poses;

}

std::vector<geometry_msgs::Pose> collision_avoidance_pick_and_place::PickAndPlace::create_pick_poses(double retreat_dis,double approach_dis,const tf::Transform &target_tf)
{
  geometry_msgs::Pose start_pose, target_pose, still_pose;
  std::vector<geometry_msgs::Pose> poses;
  tf::Transform prepare;


  prepare.setOrigin(tf::Vector3(0,0,-0.05));
  //prepare.setRotation(tf::Quaternion(tf::Vector3(1,0,0),M_PI));
  prepare.setRotation(tf::Quaternion::getIdentity());
  //prepare.setRotation(target_tf.getRotation());
  transform_broadcaster->sendTransform(tf::StampedTransform(prepare, ros::Time::now(), "world_to_tcp", "pre_tf"));
  //world_to_tcp is in world_frame

  ROS_INFO("transforming");

  tf::StampedTransform pre_to_ORK_tf, ORK_to_kinect_tf, kinect_to_world_tf, pre_to_kinect_tf, kinect_world_tf;
  //tf::StampedTransform pre_to_world_tf;
  try{
    transform_listener_ptr->waitForTransform("world_to_tcp", "pre_tf", ros::Time(0.0f), ros::Duration(3.0f));
    transform_listener_ptr->lookupTransform("world_to_tcp", "pre_tf", ros::Time(0.0f), pre_to_ORK_tf);
    //        transform_listener_ptr->waitForTransform("pre_tf", "ORK", ros::Time(0.0f), ros::Duration(3.0f));
    //        transform_listener_ptr->lookupTransform("pre_tf", "ORK", ros::Time(0.0f), pre_to_ORK_tf);
    //        transform_listener_ptr->waitForTransform("ORK", "kinect2_rgb_optical_frame", ros::Time(0.0f), ros::Duration(3.0f));
    //        transform_listener_ptr->lookupTransform("ORK", "kinect2_rgb_optical_frame", ros::Time(0.0f), pre_to_ORK_tf);
    //        transform_listener_ptr->waitForTransform("kinect2_rgb_optical_frame", "world_frame", ros::Time(0.0f), ros::Duration(3.0f));
    //        transform_listener_ptr->lookupTransform("kinect2_rgb_optical_frame", "world_frame", ros::Time(0.0f), pre_to_ORK_tf);
//    transform_listener_ptr->waitForTransform("world_frame", "pre_tf", ros::Time(0.0f), ros::Duration(3.0f));
//    transform_listener_ptr->lookupTransform("world_frame", "pre_tf", ros::Time(0.0f), pre_to_world_tf);

  }
  catch (tf::TransformException ex){
    ROS_ERROR("%s",ex.what());
    //exit(1);
    return poses;
  }

  //tf::Transform pre_to_world_tf = pre_to_ORK_tf * ORK_to_kinect_tf * kinect_to_world_tf;
  tf::Transform pre_to_world_tf = target_tf * pre_to_ORK_tf;
  transform_broadcaster->sendTransform(tf::StampedTransform(pre_to_world_tf, ros::Time::now(), "world_frame", "pre_tf2"));

  // converting target pose
  tf::poseTFToMsg(target_tf,target_pose);
  tf::poseTFToMsg(pre_to_world_tf,start_pose);
  tf::Transform still_tf;
  still_tf.setOrigin(pre_to_world_tf.getOrigin());
  //still_tf.setRotation(tf::Quaternion(tf::Vector3(1,0,0),M_PI));
  still_tf.setRotation(tf::Quaternion(M_PI, 0, M_PI_2));
  tf::poseTFToMsg(still_tf,still_pose);
  transform_broadcaster->sendTransform(tf::StampedTransform(still_tf, ros::Time::now(), "world_frame", "still_tf"));

  // creating start pose by applying a translation along +z by approach distance
  //tf::poseTFToMsg(Transform(Quaternion::getIdentity(),Vector3(0,0,approach_dis))*target_tf,start_pose);


  poses.clear();
  poses.push_back(start_pose);
  poses.push_back(target_pose);
  poses.push_back(start_pose);
  poses.push_back(still_pose);

  return poses;
}


