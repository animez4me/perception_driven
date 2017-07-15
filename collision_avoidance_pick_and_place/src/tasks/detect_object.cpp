//#include <collision_avoidance_pick_and_place/pick_and_place.h>

//#include <object_recognition_msgs/RecognizedObjectArray.h>
//#include <object_recognition_msgs/ObjectRecognitionAction.h>
//#include <ros/ros.h>

////#include <tf/transform_broadcaster.h>
//#include <tf2/LinearMath/Quaternion.h>
//#include <tf2_ros/transform_broadcaster.h>
//#include <geometry_msgs/TransformStamped.h>
//#include <tf2_ros/transform_listener.h>
//#include <tf2/transform_datatypes.h>

//std::string Object_id = "";
//double Object_confidence = 0;
//geometry_msgs::PoseStamped Object_pose;
//bool firstCB = false;


//void objectCallback(const object_recognition_msgs::RecognizedObjectArray objects_msg)
//{
//  double confident = 0;
//  int id = -1;


//  if (firstCB == false && (int)objects_msg.objects.size() == 1) {
//    Object_id.assign(objects_msg.objects[0].type.key.c_str());
//    ROS_INFO("info");
//    ROS_INFO_STREAM(objects_msg.objects[0].type.key.c_str());
//    firstCB = true;

//  }

//  for (int i = 0; i < objects_msg.objects.size(); ++i) {
//    if (Object_id.compare(objects_msg.objects[i].type.key.c_str()) == 0) {
//      if (objects_msg.objects[i].confidence > confident) {
//        confident = objects_msg.objects[i].confidence;
//        id = i;

//      }
//    }
//  }

//  if (id >= 0) {

//    Object_pose.pose = objects_msg.objects[id].pose.pose.pose;
//    Object_confidence = objects_msg.objects[id].confidence;
//  }
//  else {
//    confident = 0;

//  }
//  ROS_INFO("Position x is: %f", Object_pose.pose.position.x);
//  ROS_INFO("Position y is: %f", Object_pose.pose.position.y);
//  ROS_INFO("Position z is: %f", Object_pose.pose.position.z);
//  ROS_INFO("Pose w is: %f", Object_pose.pose.orientation.w);
//  ROS_INFO("Pose x is: %f", Object_pose.pose.orientation.x);
//  ROS_INFO("Pose y is: %f", Object_pose.pose.orientation.y);
//  ROS_INFO("Pose z is: %f", Object_pose.pose.orientation.z);

//  static tf2_ros::TransformBroadcaster br;
//  geometry_msgs::TransformStamped transformStamped;

//  transformStamped.header.stamp = ros::Time::now();
//  transformStamped.header.frame_id = "kinect2_rgb_optical_frame";
//  transformStamped.child_frame_id = "ORK";
//  transformStamped.transform.translation.x = Object_pose.pose.position.x;
//  transformStamped.transform.translation.y = Object_pose.pose.position.y;
//  transformStamped.transform.translation.z = Object_pose.pose.position.z;
//  transformStamped.transform.rotation.x = Object_pose.pose.orientation.x;
//  transformStamped.transform.rotation.y = Object_pose.pose.orientation.y;
//  transformStamped.transform.rotation.z = Object_pose.pose.orientation.z;
//  transformStamped.transform.rotation.w = Object_pose.pose.orientation.w;

//  br.sendTransform(transformStamped);


////  tf::Transform point;
////  tf::TransformBroadcaster broadc;

////  point.setOrigin(tf::Vector3(Object_pose.pose.position.x , Object_pose.pose.position.y,
////                                        Object_pose.pose.position.z));
////  point.setRotation(tf::Quaternion(Object_pose.pose.orientation.x, Object_pose.pose.orientation.y,
////                                     Object_pose.pose.orientation.z, Object_pose.pose.orientation.w));
////  broadc.sendTransform(tf::StampedTransform(point, ros::Time::now(), "world_frame", "ORK"));



//}

//geometry_msgs::Pose collision_avoidance_pick_and_place::PickAndPlace::detect_object()
//{
//  ros::NodeHandle nh;
//  //tf::TransformBroadcaster bro;

//  ros::Subscriber object_sub = nh.subscribe("/recognized_object_array", 1, &objectCallback);
//  ros::Duration(5).sleep();
//  ros::spinOnce();

////  //system("rosrun object_recognition_ros client");
////  // Running actionlib client, the same as executing same command as above
////  actionlib::SimpleActionClient<object_recognition_msgs::ObjectRecognitionAction> ac("recognize_objects", true);

////  ROS_INFO("Waiting for action server to start.");
////  // wait for the action server to start
////  ac.waitForServer(); //will wait for infinite time

////  ROS_INFO("Action server started, sending goal.");
////  // send a goal to the action
////  object_recognition_msgs::ObjectRecognitionGoal goal;
////  ac.sendGoal(goal);

////  //wait for the action to return
////  bool finished_before_timeout = ac.waitForResult(ros::Duration(30.0));

////  if (finished_before_timeout)
////  {
////    actionlib::SimpleClientGoalState state = ac.getState();
////    ROS_INFO("Action finished: %s",state.toString().c_str());
////    ros::Subscriber object_sub = nh.subscribe("/recognized_object_array", 1, &objectCallback);
////    ros::Duration(5).sleep();
////    ros::spinOnce();
////  }
////  else
////    ROS_INFO("Action did not finish before the time out.");


//  // Convert object pose to tf
//  geometry_msgs::Pose box_pose, tf_pose;
//  tf::Transform box_tf;
//  tf::poseMsgToTF(Object_pose.pose,box_tf);

//  // Exit the program if object is not found
//  if (!Object_pose.pose.position.x){
//    ROS_ERROR_STREAM("None or too many objects detected");
//    exit(1);
//  }

//  // Lookup transformation between world frame and kinect frame
//  tf2_ros::Buffer tfBuffer;
//  tf2_ros::TransformListener tfListener(tfBuffer);
//  geometry_msgs::TransformStamped transformStamped;
//  try{
//    ROS_INFO("Waiting for transform");
//    transformStamped = tfBuffer.lookupTransform("world_frame", "kinect2_rgb_optical_frame", ros::Time(0), ros::Duration(10.0));
//  }
//  catch (tf2::TransformException &ex) {
//    ROS_WARN("%s",ex.what());
//  }
//  tf_pose.position.x = transformStamped.transform.translation.x;
//  tf_pose.position.y = transformStamped.transform.translation.y;
//  tf_pose.position.z = transformStamped.transform.translation.z;
//  tf_pose.orientation.w = transformStamped.transform.rotation.w;
//  tf_pose.orientation.x = transformStamped.transform.rotation.x;
//  tf_pose.orientation.y = transformStamped.transform.rotation.y;
//  tf_pose.orientation.z = transformStamped.transform.rotation.z;

//  // Perform frame transformation to find object pose in world_frame
//  tf::Transform tf_tf, transformed;
//  tf::poseMsgToTF(tf_pose, tf_tf);
//  transformed = tf_tf*box_tf;
//  tf::poseTFToMsg(transformed,box_pose);

//  ROS_INFO("Pose X is: %f", box_pose.position.x);
//  ROS_INFO("Pose Y is: %f", box_pose.position.y);
//  ROS_INFO("Pose Z is: %f", box_pose.position.z);
//  ROS_INFO("Orientation W is: %f", box_pose.orientation.w);
//  ROS_INFO("Orientation X is: %f", box_pose.orientation.x);
//  ROS_INFO("Orientation Y is: %f", box_pose.orientation.y);
//  ROS_INFO("Orientation Z is: %f", box_pose.orientation.z);

//  // updating box marker for visualization in rviz
//  visualization_msgs::Marker marker = cfg.MARKER_MESSAGE;
//  cfg.MARKER_MESSAGE.header.frame_id = cfg.WORLD_FRAME_ID;
//  cfg.MARKER_MESSAGE.pose = box_pose;
//  // offset the box marker so the top surface is aligned with the axis rather than the centroid
//  //cfg.MARKER_MESSAGE.pose.position.z = box_pose.position.z - 0.5f*cfg.BOX_SIZE.z();
//  //cfg.MARKER_MESSAGE.pose.position.z = box_pose.position.z + 0.5f*cfg.BOX_SIZE.z();

//  show_box(true);

//  return box_pose;
//}

