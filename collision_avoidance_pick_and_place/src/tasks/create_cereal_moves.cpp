#include <collision_avoidance_pick_and_place/pick_and_place.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/transform_listener.h>

std::vector<geometry_msgs::Pose> collision_avoidance_pick_and_place::PickAndPlace::create_cereal_poses(const tf::Transform &target_tf)
{
  geometry_msgs::Pose pre_pour_pose, pour_pose;
  std::vector<geometry_msgs::Pose> poses;

  tf::Transform pour_tf, end_tf;

  // creating start pose by applying a translation along +z by approach distance

  transform_broadcaster->sendTransform(tf::StampedTransform(target_tf, ros::Time::now(), "world_frame", "pre_pour"));
  tf::poseTFToMsg(target_tf,pre_pour_pose);

  // converting target pose
  //tf::poseTFToMsg(Transform(Quaternion(-M_PI_4,0,0),Vector3(0,0,0))*pre_drink_tf,drink_pose);
  pour_tf.setOrigin(target_tf.getOrigin());
  pour_tf.setRotation(tf::Quaternion(M_PI_2, 0, -M_PI));
  tf::poseTFToMsg(pour_tf,pour_pose);
//  tf::poseMsgToTF(drink_pose, drink_tf);
  // creating end pose by applying a translation along +z by retreat distance
  //tf::poseTFToMsg(Transform(Quaternion::getIdentity(),Vector3(0,retreat_dis,0))*target_tf,end_pose);
//  broadc.sendTransform(tf::StampedTransform(pour_tf, ros::Time::now(), "world_frame", "pour"));
  transform_broadcaster->sendTransform(tf::StampedTransform(pour_tf, ros::Time::now(), "world_frame", "pour"));

  tf::StampedTransform table_tf;
  geometry_msgs::Pose end_pose, table_pose, final_pose;
  transform_listener_ptr->lookupTransform("world_frame", "table", ros::Time(0.0f), table_tf);
  tf::poseTFToMsg(table_tf, table_pose);
  end_tf.setOrigin(target_tf.getOrigin());
  end_tf.setRotation(target_tf.getRotation());
  tf::poseTFToMsg(end_tf, end_pose);
  end_pose.position.z = table_pose.position.z + cfg.CEREAL_SIZE[0]/2;
//  final_pose = end_pose;
//  final_pose.position.x = end_pose.position.x - 0.1;

  poses.clear();
//  poses.push_back(middle_pose);
  poses.push_back(pre_pour_pose);
  poses.push_back(pour_pose);
  poses.push_back(pre_pour_pose);
  poses.push_back(end_pose);
//  poses.push_back(final_pose);

  return poses;
}

std::vector<geometry_msgs::Pose> collision_avoidance_pick_and_place::PickAndPlace::create_cereal_moves()
{
  // task variables  
//  tf::StampedTransform tcp_to_wrist_tf, world_to_tcp_tf;
//  std::vector<geometry_msgs::Pose> tcp_place_poses, wrist_place_poses;

//  try{
//    transform_listener_ptr->waitForTransform("world_frame", "pre_pour", ros::Time(0.0f), ros::Duration(3.0f));
//    transform_listener_ptr->lookupTransform("world_frame", "pre_pour", ros::Time(0.0f), world_to_tcp_tf);

//  }
//  catch (tf::TransformException ex){
//    ROS_ERROR("%s",ex.what());
//  }
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
  geometry_msgs::Pose pre_pour_pose;
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


  /* Fill Code:
   * Goal:
   * - Create place poses for tcp.   *
   * Hints:
   * - Use the "create_manipulation_poses" and save results to "tcp_place_poses".
   * - Look in the "cfg" object to find the corresponding retreat and approach distance
   * 	values.
   */
  tcp_place_poses = create_cereal_poses(world_to_tcp_tf);


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
  ROS_INFO_STREAM("Wrist picking orientationtcp x: " << tcp_place_poses[0].orientation.x);
  ROS_INFO_STREAM("Wrist picking orientationtcp y: " << tcp_place_poses[0].orientation.y);
  ROS_INFO_STREAM("Wrist picking orientationtcp z: " << tcp_place_poses[0].orientation.z);
  ROS_INFO_STREAM("Wrist picking orientationtcp w: " << tcp_place_poses[0].orientation.w);
  ROS_INFO_STREAM("Wrist picking orientation1 x: " << wrist_place_poses[0].orientation.x);
  ROS_INFO_STREAM("Wrist picking orientation1 y: " << wrist_place_poses[0].orientation.y);
  ROS_INFO_STREAM("Wrist picking orientation1 z: " << wrist_place_poses[0].orientation.z);
  ROS_INFO_STREAM("Wrist picking orientation1 w: " << wrist_place_poses[0].orientation.w);

  return wrist_place_poses;
}
