/*
 * pick_and_place_utilities.cpp
 *
 *  Created on: Jun 3, 2013
 *      Author: ros-industrial
 */

#include <collision_avoidance_pick_and_place/pick_and_place_utilities.h>
#include <moveit/kinematic_constraints/utils.h>
#include <tf/transform_listener.h>
#include <boost/assign/std/vector.hpp>
#include <iostream>
#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf/transform_broadcaster.h>


using namespace tf;
using namespace boost::assign;



// =============================== Utility functions ===============================

//std::vector<geometry_msgs::Pose> create_manipulation_poses(double retreat_dis,double approach_dis,const tf::Transform &target_tf)
//{
//  geometry_msgs::Pose start_pose, target_pose, end_pose, pre_pose;
//  std::vector<geometry_msgs::Pose> poses;
//  tf::Transform pre_tf;
//  //tf::TransformListener listener;

////  tf::Transform tcp;
////  tf::TransformBroadcaster broadc;
////  broadc.sendTransform(tf::StampedTransform(target_tf, ros::Time::now(), "ORK", "pre_tf"));
////  ROS_INFO("transforming");

//  tf::StampedTransform pre_to_ORK_tf, ORK_to_kinect_tf, kinect_to_world_tf;
//  try{
////    transform_listener_ptrr->waitForTransform("pre_tf", "ORK", ros::Time(0.0f), ros::Duration(3.0f));
////    transform_listener_ptrr->lookupTransform("pre_tf", "ORK", ros::Time(0.0f), pre_to_ORK_tf);

////    tf_listener.waitForTransform("pre_tf", "ORK",
////                              ros::Time::now(), ros::Duration(10.0));
////    tf_listener.lookupTransform("pre_tf", "ORK",
////                             ros::Time::now(), pre_to_ORK_tf);
////    listener.waitForTransform("ORK", "kinect2_rgb_optical_frame",
////                              ros::Time(0), ros::Duration(10.0));
////    listener.lookupTransform("ORK", "kinect2_rgb_optical_frame",
////                             ros::Time(0), ORK_to_kinect_tf);
////    listener.waitForTransform("kinect2_rgb_optical_frame", "world_frame",
////                              ros::Time(0), ros::Duration(10.0));
////    listener.lookupTransform("kinect2_rgb_optical_frame", "world_frame",
////                             ros::Time(0), kinect_to_world_tf);
//  }
//  catch (tf::TransformException ex){
//    ROS_ERROR("%s",ex.what());
//    //ros::Duration(1.0).sleep();
//  }

////  tf::Transform pre_to_world_tf = pre_to_ORK_tf*ORK_to_kinect_tf*kinect_to_world_tf;

//  // converting target pose
//  tf::poseTFToMsg(target_tf,target_pose);
////  tf::poseTFToMsg(pre_to_world_tf,start_pose);

////  static tf2_ros::TransformBroadcaster br;
////  geometry_msgs::TransformStamped transformStamped;

////  transformStamped.header.stamp = ros::Time::now();
////  transformStamped.header.frame_id = "pre_tf";
////  transformStamped.child_frame_id = "ORK";
////  transformStamped.transform.translation.x = target_pose.position.x;
////  transformStamped.transform.translation.y = target_pose.position.y;
////  transformStamped.transform.translation.z = target_pose.position.z + 0.05;
////  transformStamped.transform.rotation.x = target_pose.orientation.x;
////  transformStamped.transform.rotation.y = target_pose.orientation.y;
////  transformStamped.transform.rotation.z = target_pose.orientation.z;
////  transformStamped.transform.rotation.w = target_pose.orientation.w;

////  br.sendTransform(transformStamped);

////  ros::Duration(5.0).sleep();
////  ros::spinOnce();

////  tf2_ros::Buffer tfBuffer;
////  tf2_ros::TransformListener tfListener(tfBuffer);
////  geometry_msgs::TransformStamped transformStamped2;
////  try{
////    transformStamped2 = tfBuffer.lookupTransform("pre_tf", "world_frame",
////                                                ros::Time(0), ros::Duration(10.0));
////  }
////  catch (tf2::TransformException &ex) {
////    ROS_WARN("%s",ex.what());
////  }




//  // creating start pose by applying a translation along +z by approach distance
//  tf::poseTFToMsg(Transform(Quaternion::getIdentity(),Vector3(0,0,approach_dis))*target_tf,start_pose);
//  //tf::poseTFToMsg(pre_transform,start_pose);

//  // creating end pose by applying a translation along +z by retreat distance
//  //tf::poseTFToMsg(Transform(Quaternion::getIdentity(),Vector3(0,0,retreat_dis))*target_tf,end_pose);
//  //tf::poseTFToMsg(pre_transform,end_pose);

////  tf::poseMsgToTF(end_pose,still_tf);
////  still_tf.setRotation(Quaternion(M_PI, 0, M_PI_2));
////  broadc.sendTransform(tf::StampedTransform(still_tf, ros::Time::now(), "world_frame", "still"));
////  tf::poseTFToMsg(still_tf,still_pose);

//  poses.clear();
//  poses.push_back(start_pose);
//  poses.push_back(target_pose);
//  poses.push_back(start_pose);

//  return poses;
//}



std::vector<geometry_msgs::Pose> create_drink_poses(const tf::Transform &target_tf)
{
  geometry_msgs::Pose pre_drink_pose, drink_pose;
  std::vector<geometry_msgs::Pose> poses;

  tf::Transform pre_drink_tf, drink_tf;
  tf::Transform tcp;
  tf::TransformBroadcaster broadc;


  // creating start pose by applying a translation along +z by approach distance
  tf::poseTFToMsg(target_tf,pre_drink_pose);
  tf::poseMsgToTF(pre_drink_pose, pre_drink_tf);
  //tf::poseTFToMsg(target_tf,start_pose);
  broadc.sendTransform(tf::StampedTransform(pre_drink_tf, ros::Time::now(), "world_frame", "pre_drink"));

//  static tf::Transform test;
//  test.setOrigin(Vector3(0,0,0));
//  test.setRotation(Quaternion(1,0,0,0));
//  broadc.sendTransform(tf::StampedTransform(test, ros::Time::now(), "world_frame", "test"));

  // converting target pose
  //tf::poseTFToMsg(Transform(Quaternion(-M_PI_4,0,0),Vector3(0,0,0))*pre_drink_tf,drink_pose);
  pre_drink_tf.setRotation(Quaternion(M_PI-M_PI_4, 0, M_PI_2));
  tf::poseTFToMsg(pre_drink_tf,drink_pose);
  tf::poseMsgToTF(drink_pose, drink_tf);
  // creating end pose by applying a translation along +z by retreat distance
  //tf::poseTFToMsg(Transform(Quaternion::getIdentity(),Vector3(0,retreat_dis,0))*target_tf,end_pose);
  broadc.sendTransform(tf::StampedTransform(drink_tf, ros::Time::now(), "world_frame", "drink"));

  ROS_INFO("Predrink position x %f", pre_drink_pose.position.x);
  ROS_INFO("Predrink position y %f", pre_drink_pose.position.y);
  ROS_INFO("Predrink position z %f", pre_drink_pose.position.z);
  ROS_INFO("Predrink orientation w %f", pre_drink_pose.orientation.w);
  ROS_INFO("Predrink orientation x %f", pre_drink_pose.orientation.x);
  ROS_INFO("Predrink orientation y %f", pre_drink_pose.orientation.y);
  ROS_INFO("Predrink orientation z %f", pre_drink_pose.orientation.z);
  ROS_INFO("Drink position x %f", drink_pose.position.x);
  ROS_INFO("Drink position y %f", drink_pose.position.y);
  ROS_INFO("Drink position z %f", drink_pose.position.z);
  ROS_INFO("Drink orientation w %f", drink_pose.orientation.w);
  ROS_INFO("Drink orientation x %f", drink_pose.orientation.x);
  ROS_INFO("Drink orientation y %f", drink_pose.orientation.y);
  ROS_INFO("Drink orientation z %f", drink_pose.orientation.z);

  geometry_msgs::Pose middle_pose;
//  middle_pose.position.x = 0.3;
//  middle_pose.position.y = -0.2;
//  middle_pose.position.z = 0.2;
//  middle_pose.orientation.w = 0;
//  middle_pose.orientation.x = 0.707107;
//  middle_pose.orientation.y = 0.707107;
//  middle_pose.orientation.z = 0;

  poses.clear();
//  poses.push_back(middle_pose);
  poses.push_back(pre_drink_pose);
  //poses.push_back(drink_pose);
  //poses.push_back(pre_drink_pose);

  return poses;
}

std::vector<geometry_msgs::Pose> transform_from_tcp_to_wrist(tf::Transform tcp_to_wrist_tf,const std::vector<geometry_msgs::Pose> tcp_poses)
{
  // array for poses of the wrist
  std::vector<geometry_msgs::Pose> wrist_poses;
  wrist_poses.resize(tcp_poses.size());

  // applying transform to each tcp poses
  tf::Transform world_to_wrist_tf, world_to_tcp_tf;
  wrist_poses.resize(tcp_poses.size());
  for(unsigned int i = 0; i < tcp_poses.size(); i++)
  {
    tf::poseMsgToTF(tcp_poses[i],world_to_tcp_tf);
    world_to_wrist_tf = world_to_tcp_tf * tcp_to_wrist_tf;
    tf::poseTFToMsg(world_to_wrist_tf,wrist_poses[i]);
  }

  return wrist_poses;
}

moveit_msgs::Constraints create_path_orientation_constraints(const geometry_msgs::Pose &goal_pose,
    float x_tolerance,float y_tolerance,float z_tolerance,std::string link_name)
{
	moveit_msgs::Constraints path_constraints = moveit_msgs::Constraints();
	path_constraints.name = "tcp_orientation_constraint";

	// setting constraint properties
	moveit_msgs::OrientationConstraint orientation_constraint = moveit_msgs::OrientationConstraint();
	orientation_constraint.header.frame_id="world_frame";
  orientation_constraint.orientation = goal_pose.orientation;
//  orientation_constraint.orientation.x = 0.707;
//  orientation_constraint.orientation.y = 0;
//  orientation_constraint.orientation.z = -0.707;
//  orientation_constraint.orientation.w = 0;
  orientation_constraint.absolute_x_axis_tolerance = x_tolerance;
  orientation_constraint.absolute_y_axis_tolerance = y_tolerance;
  orientation_constraint.absolute_z_axis_tolerance = z_tolerance;
	orientation_constraint.weight = 1.0f;
	orientation_constraint.link_name = link_name;

	// adding orientation constraint to path_constraints object
	path_constraints.orientation_constraints.push_back(orientation_constraint);
	return path_constraints;
}

std::ostream& operator<<(std::ostream& os, const tf::Vector3 vec)
{
  return os << "[" << vec.getX() << ", " << vec.getY() << ", " << vec.getZ() << "]";
}

std::ostream& operator<<(std::ostream& os, const geometry_msgs::Point pt)
{
  return os << "[" << pt.x << ", " << pt.y << ", " << pt.z << "]";
}

bool pick_and_place_config::init()
{
  ros::NodeHandle nh("~");
  double w, l, h, x, y, z, x_pre_drink, y_pre_drink, z_pre_drink, coke_height, coke_radius;
  XmlRpc::XmlRpcValue list;

  if(nh.getParam("arm_group_name",ARM_GROUP_NAME)
      && nh.getParam("tcp_link_name",TCP_LINK_NAME)
      && nh.getParam("wrist_link_name",WRIST_LINK_NAME)
      && nh.getParam("attached_object_link",ATTACHED_OBJECT_LINK_NAME)
      && nh.getParam("world_frame_id",WORLD_FRAME_ID)
      && nh.getParam("home_pose_name",HOME_POSE_NAME)
      && nh.getParam("wait_pose_name",WAIT_POSE_NAME)
      && nh.getParam("ar_frame_id",AR_TAG_FRAME_ID)
      && nh.getParam("box_width",w)
      && nh.getParam("box_length",l)
      && nh.getParam("box_height",h)
      && nh.getParam("box_place_x",x)
      && nh.getParam("box_place_y",y)
      && nh.getParam("box_place_z",z)
      && nh.getParam("touch_links",list)
      && nh.getParam("retreat_distance",RETREAT_DISTANCE)
      && nh.getParam("approach_distance",APPROACH_DISTANCE)
      && nh.getParam("pre_drink_x",x_pre_drink)
      && nh.getParam("pre_drink_y",y_pre_drink)
      && nh.getParam("pre_drink_z",z_pre_drink)
      && nh.getParam("coke_height",coke_height)
      && nh.getParam("coke_radius",coke_radius))
  {
    BOX_SIZE = Vector3(l,w,h);
    BOX_PLACE_TF.setOrigin(Vector3(x,y,z));
    PRE_DRINK_TF.setOrigin(Vector3(x_pre_drink,y_pre_drink,z_pre_drink));
    COKE_SIZE.push_back(coke_height);
    COKE_SIZE.push_back(coke_radius);

    // parsing touch links list
    for(int32_t i =0 ; i < list.size();i++)
    {
    	if(list[i].getType() == XmlRpc::XmlRpcValue::TypeString)
    	{
    		TOUCH_LINKS.push_back(static_cast<std::string>(list[i]));
    	}
    	else
    	{
    		ROS_ERROR_STREAM("touch links list contains invalid data.  Provide an array of strings 'touch_links:{link_a,link_b,link_c ..}'");
    	}
    }

    // building geometric primitive for target
    shape_msgs::SolidPrimitive shape;
    //shape.type = shape_msgs::SolidPrimitive::BOX;
    shape.type = shape_msgs::SolidPrimitive::CYLINDER;
    shape.dimensions.resize(3);
    //shape.dimensions.resize(2);
    shape.dimensions[0] = BOX_SIZE.getX();
    shape.dimensions[1] = BOX_SIZE.getY();
    shape.dimensions[2] = BOX_SIZE.getZ();

    // setting pose of object relative to tcp
    tf::poseTFToMsg(tf::Transform::getIdentity(),TCP_TO_BOX_POSE);
    TCP_TO_BOX_POSE.position.x = 0;
    TCP_TO_BOX_POSE.position.y = 0;
    TCP_TO_BOX_POSE.position.z = 0.5f* BOX_SIZE.getZ();

    // creating visual object
    MARKER_MESSAGE.header.frame_id = TCP_LINK_NAME;
    MARKER_MESSAGE.type = visualization_msgs::Marker::CYLINDER;
    MARKER_MESSAGE.pose = TCP_TO_BOX_POSE;
    MARKER_MESSAGE.id = 0;
    MARKER_MESSAGE.color.r = 0;
    MARKER_MESSAGE.color.g = 0;
    MARKER_MESSAGE.color.b = 1;
    MARKER_MESSAGE.color.a = 0.5f;
    MARKER_MESSAGE.lifetime = ros::Duration(100); // persists forever
    MARKER_MESSAGE.frame_locked = true;
    MARKER_MESSAGE.scale.x = shape.dimensions[0];
    MARKER_MESSAGE.scale.y = shape.dimensions[1];
    MARKER_MESSAGE.scale.z = shape.dimensions[2];

    // create attached object
    ATTACHED_OBJECT.header.frame_id = TCP_LINK_NAME;
    ATTACHED_OBJECT.id=ATTACHED_OBJECT_LINK_NAME;
    ATTACHED_OBJECT.primitives.push_back(shape);
    ATTACHED_OBJECT.primitive_poses.push_back(TCP_TO_BOX_POSE);

    return true;
  }
  else
  {
    return false;
  }

}
