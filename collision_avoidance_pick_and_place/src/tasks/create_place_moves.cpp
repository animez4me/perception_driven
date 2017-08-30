#include <collision_avoidance_pick_and_place/pick_and_place.h>

/*    CREATE PLACE MOVES
  Goal:
    - Set the pose of the tcp at the box place pose
    - Create tcp poses for the place motion (approach, release, retreat).
    - Find transform of the wrist in tcp coordinates
    - Convert tcp pick poses to wrist poses.
         * MoveIt's kinematics require the target position to be specified relative to
           one of the kinematic links of the manipulator arm (as defined in the SRDF)

  Hints:
    - You can manipulate the "world_to_tcp_tf" transform through the "setOrigin" and "setRotation".
    - Use the "create_manipulation_poses" function to create the tcp poses between each place move
    - Use the "transform_from_tcp_to_wrist" function to populate the "wrist_place_poses" array.
*/

std::vector<geometry_msgs::Pose> collision_avoidance_pick_and_place::PickAndPlace::create_place_moves()
{
  //ROS_ERROR_STREAM("create_place_moves is not implemented yet.  Aborting."); exit(1);

  // task variables
  tf::Transform world_to_tcp_tf;
  tf::StampedTransform tcp_to_wrist_tf;
  std::vector<geometry_msgs::Pose> tcp_place_poses, wrist_place_poses;

  if (cfg.selected_object.compare("cereal") == 0) {
    tf::StampedTransform pre_pour_tf;
    geometry_msgs::Pose final_pose;
    transform_listener_ptr->lookupTransform("world_frame", "pre_pour", ros::Time(0.0f), pre_pour_tf);
    tf::poseTFToMsg(pre_pour_tf, final_pose);
    final_pose.position.x = final_pose.position.x - 0.05;
    final_pose.position.z = final_pose.position.z - cfg.CEREAL_SIZE[0]/2;
    world_to_tcp_tf.setOrigin(tf::Vector3(final_pose.position.x, final_pose.position.y, final_pose.position.z));
    world_to_tcp_tf.setRotation(pre_pour_tf.getRotation());
  }
  else
  {

    world_to_tcp_tf.setOrigin(cfg.COKE_PLACE_TF.getOrigin());
    world_to_tcp_tf.setRotation(tf::Quaternion(M_PI, 0, -M_PI/2.0f));
//    world_to_tcp_tf.setRotation(tf::Quaternion(M_PI, 0, M_PI/2.0f));
//    world_to_tcp_tf.setRotation(tf::Quaternion(tf::Vector3(1,0,0),M_PI));
  }


  /* Fill Code:
   * Goal:
   * - Create place poses for tcp.   *
   * Hints:
   * - Use the "create_manipulation_poses" and save results to "tcp_place_poses".
   * - Look in the "cfg" object to find the corresponding retreat and approach distance
   * 	values.
   */
  tcp_place_poses = create_place_poses(cfg.RETREAT_DISTANCE, cfg.APPROACH_DISTANCE, world_to_tcp_tf);


  /* Fill Code:
   * Goal:
   * - Find transform from tcp to wrist.
   * Hints:
   * - Use the "lookupTransform" method in the transform listener.
   * */
  transform_listener_ptr->waitForTransform(cfg.TCP_LINK_NAME, cfg.WRIST_LINK_NAME, ros::Time(0.0f), ros::Duration(3.0f));
  transform_listener_ptr->lookupTransform(cfg.TCP_LINK_NAME, cfg.WRIST_LINK_NAME, ros::Time(0.0f), tcp_to_wrist_tf);


  /* Fill Code:
   * Goal:
   * - Transform list of place poses from the tcp to the wrist coordinate frame.
   * Hints:
   * - Use the "transform_from_tcp_to_wrist" function and save results into
   * 	"wrist_place_poses".
   * - The "tcp_to_wrist_tf" is the transform that will help convert "tcp_place_poses"
   * 	into "wrist_place_poses".
   */
  wrist_place_poses = transform_from_tcp_to_wrist(tcp_to_wrist_tf, tcp_place_poses);

  // printing results
  ROS_INFO_STREAM("tcp position at place: " << world_to_tcp_tf.getOrigin());
  ROS_INFO_STREAM("wrist position at place: "<<wrist_place_poses[1].position);

  return wrist_place_poses;
}

std::vector<geometry_msgs::Pose> collision_avoidance_pick_and_place::PickAndPlace::create_place_poses(double retreat_dis,double approach_dis,const tf::Transform &target_tf)
{
  geometry_msgs::Pose start_pose, target_pose, end_pose;
  std::vector<geometry_msgs::Pose> poses;
  tf::Transform prepare;

  if (cfg.selected_object.compare("cereal") == 0) {
    // converting target pose
    tf::poseTFToMsg(target_tf,target_pose);
    poses.clear();
    poses.push_back(target_pose);
  }
  else {
//    prepare.setOrigin(tf::Vector3(0,0,-retreat_dis));
//    prepare.setRotation(tf::Quaternion::getIdentity());
//    transform_broadcaster->sendTransform(tf::StampedTransform(prepare, ros::Time::now(), "world_to_tcp", "pre_tf"));

//    tf::StampedTransform pre_to_ORK_tf;
//    try{
//      transform_listener_ptr->waitForTransform("pre_tf", "world_to_tcp", ros::Time(0.0f), ros::Duration(3.0f));
//      transform_listener_ptr->lookupTransform("pre_tf", "world_to_tcp", ros::Time(0.0f), pre_to_ORK_tf);

//    }
//    catch (tf::TransformException ex){
//      ROS_ERROR("%s",ex.what());
//      exit(1);
//    }

//    tf::Transform pre_to_world_tf = pre_to_ORK_tf * target_tf;
    tf::Transform pre_to_world_tf;
    pre_to_world_tf.setOrigin(target_tf.getOrigin());
    pre_to_world_tf.setRotation(target_tf.getRotation());


    // converting target pose
    tf::poseTFToMsg(target_tf,target_pose);
    tf::poseTFToMsg(pre_to_world_tf,start_pose);
    start_pose.position.z = start_pose.position.z + 0.02;
    //    tf::Transform still_tf;
    //    still_tf.setOrigin(pre_to_world_tf.getOrigin());
    end_pose = start_pose;
    end_pose.position.z = end_pose.position.z + cfg.MUG_SIZE[0];

    poses.clear();
    poses.push_back(start_pose);
    poses.push_back(target_pose);
    poses.push_back(end_pose);
  }


  return poses;
}



