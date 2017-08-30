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

std::vector<geometry_msgs::Pose> collision_avoidance_pick_and_place::PickAndPlace::create_pick_moves(geometry_msgs::Pose &object_pose)
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
  tf::poseMsgToTF(object_pose,world_to_box_tf);
  tf::Vector3 object_position(object_pose.position.x, object_pose.position.y, object_pose.position.z);
  world_to_tcp_tf.setOrigin(object_position);


  /* Setting tcp orientation - lines up the ee to object pose
	   * Inverting the approach direction so that the tcp points towards the box instead of
	   * away from it.*/
  if (cfg.selected_object.compare("coke") == 0){
    world_to_tcp_tf.setRotation(world_to_box_tf.getRotation()* tf::Quaternion(tf::Vector3(1,0,0),M_PI));
    ROS_INFO("no action for coke");
    exit(1);
  }
  else if (cfg.selected_object.compare("cereal") == 0){
    //world_to_tcp_tf.setRotation(world_to_box_tf.getRotation()* tf::Quaternion(tf::Vector3(0,1,0),M_PI_2));
    world_to_tcp_tf.setRotation(world_to_box_tf.getRotation()* tf::Quaternion(tf::Vector3(1,0,0),0));
  }
  else if (cfg.selected_object.compare("mug") == 0){
    world_to_tcp_tf.setRotation(world_to_box_tf.getRotation()* tf::Quaternion(tf::Vector3(1,0,0),M_PI));
  }
  else if (cfg.selected_object.compare("bowl") == 0){
    ROS_INFO("no action for bowl");
    exit(1);
  }
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
  geometry_msgs::Pose start_pose, target_pose, still_pose, obj_pose;
  std::vector<geometry_msgs::Pose> poses;
  tf::Transform prepare, obj_surface;
  double coke_top = cfg.COKE_SIZE[0] / 2;
  double cerealwidth = cfg.CEREAL_SIZE[2] / 2;
  double mug_top = cfg.MUG_SIZE[0] / 2;

  tf::Transform pre_to_world_tf;
  tf::Transform surf_to_world_tf;


  if (cfg.selected_object.compare("coke") == 0){
    obj_surface.setOrigin(tf::Vector3(0,0,-coke_top));
    prepare.setOrigin(tf::Vector3(0,0,-cfg.APPROACH_DISTANCE-coke_top));
  }
  else if (cfg.selected_object.compare("cereal") == 0){
    obj_surface.setOrigin(tf::Vector3(0,0,-cerealwidth));
    prepare.setOrigin(tf::Vector3(0,0,-cfg.APPROACH_DISTANCE-cerealwidth));
    obj_surface.setRotation(tf::Quaternion::getIdentity());
    transform_broadcaster->sendTransform(tf::StampedTransform(obj_surface, ros::Time::now(), "world_to_tcp", "obj_surf"));

    prepare.setRotation(tf::Quaternion::getIdentity());
    transform_broadcaster->sendTransform(tf::StampedTransform(prepare, ros::Time::now(), "world_to_tcp", "pre_tf"));
    //world_to_tcp is in world_frame


    tf::StampedTransform pre_to_ORK_tf, surf_to_ORK_tf;
    try{
      transform_listener_ptr->waitForTransform("world_to_tcp", "obj_surf", ros::Time(0.0f), ros::Duration(3.0f));
      transform_listener_ptr->lookupTransform("world_to_tcp", "obj_surf", ros::Time(0.0f), surf_to_ORK_tf);
      transform_listener_ptr->waitForTransform("world_to_tcp", "pre_tf", ros::Time(0.0f), ros::Duration(3.0f));
      transform_listener_ptr->lookupTransform("world_to_tcp", "pre_tf", ros::Time(0.0f), pre_to_ORK_tf);
    }
    catch (tf::TransformException ex){
      ROS_ERROR("%s",ex.what());
      //exit(1);
      return poses;
    }
    surf_to_world_tf = target_tf * surf_to_ORK_tf;
    pre_to_world_tf = target_tf * pre_to_ORK_tf;
    transform_broadcaster->sendTransform(tf::StampedTransform(pre_to_world_tf, ros::Time::now(), "world_frame", "pre_tf2"));


  }
  else if (cfg.selected_object.compare("mug") == 0){
//    obj_surface.setOrigin(tf::Vector3(0,0,-mug_top));
//    prepare.setOrigin(tf::Vector3(0,0,-cfg.APPROACH_DISTANCE-mug_top));

    tf::poseTFToMsg(target_tf, obj_pose);
    pre_to_world_tf.setRotation(tf::Quaternion(M_PI, 0, -M_PI_2));
    surf_to_world_tf.setRotation(tf::Quaternion(M_PI, 0, -M_PI_2));
    pre_to_world_tf.setOrigin(tf::Vector3(obj_pose.position.x,obj_pose.position.y,obj_pose.position.z + cfg.APPROACH_DISTANCE + mug_top));
    //10mm cup thickness from bottom, 5mm magnet, 5mm buffer +20mm offset
    surf_to_world_tf.setOrigin(tf::Vector3(obj_pose.position.x,obj_pose.position.y,obj_pose.position.z - mug_top +0.04));
    transform_broadcaster->sendTransform(tf::StampedTransform(pre_to_world_tf, ros::Time::now(), "world_frame", "pre_tf2"));
    transform_broadcaster->sendTransform(tf::StampedTransform(surf_to_world_tf, ros::Time::now(), "world_frame", "obj_surf"));

  }

  // converting target pose
  tf::poseTFToMsg(surf_to_world_tf,target_pose);
  tf::poseTFToMsg(pre_to_world_tf,start_pose);
  tf::Transform still_tf;


  //still_tf.setRotation(tf::Quaternion(tf::Vector3(1,0,0),M_PI));
  //Keep object parallel with a plane
  if (cfg.selected_object.compare("cereal") == 0){
    still_tf.setOrigin(pre_to_world_tf.getOrigin());
    still_tf.setRotation(tf::Quaternion(M_PI_2, 0, -M_PI_2));
    tf::poseTFToMsg(still_tf,still_pose);
    still_pose.position.z = still_pose.position.z + 0.05;
  }
  //if object is mug or coke
  else {
    still_tf.setOrigin(surf_to_world_tf.getOrigin());
    still_tf.setRotation(tf::Quaternion(M_PI, 0, -M_PI_2));
    tf::poseTFToMsg(still_tf,still_pose);
    still_pose.position.z = still_pose.position.z +0.02;

  }

  transform_broadcaster->sendTransform(tf::StampedTransform(still_tf, ros::Time::now(), "world_frame", "still_tf"));

  // creating start pose by applying a translation along +z by approach distance
  //tf::poseTFToMsg(Transform(Quaternion::getIdentity(),Vector3(0,0,approach_dis))*target_tf,start_pose);


  poses.clear();
  poses.push_back(start_pose);
  poses.push_back(target_pose);
//  poses.push_back(start_pose);
  poses.push_back(still_pose);

  return poses;
}


