#include <collision_avoidance_pick_and_place/pick_and_place.h>

#include "object_recognition_msgs/RecognizedObjectArray.h"
#include "ros/ros.h"

#include "tf/transform_broadcaster.h"

std::string Object_id = "";
double Object_confidence = 0;
geometry_msgs::PoseStamped Object_pose;
bool firstCB = false;


void objectCallback(const object_recognition_msgs::RecognizedObjectArray objects_msg)
{
  double confident = 0;
  int id = -1;

  if (firstCB == false && (int)objects_msg.objects.size() == 1) {
    Object_id.assign(objects_msg.objects[0].type.key.c_str());
    firstCB = true;
  }

  for (int i = 0; i < objects_msg.objects.size(); ++i) {
    if (Object_id.compare(objects_msg.objects[i].type.key.c_str()) == 0) {
      if (objects_msg.objects[i].confidence > confident) {        
        confident = objects_msg.objects[i].confidence;
        id = i;
      }
    }
  }

  if (id >= 0) {

    Object_pose.pose = objects_msg.objects[id].pose.pose.pose;
    Object_confidence = objects_msg.objects[id].confidence;
  }
  else {
    confident = 0;

  }
  ROS_INFO("Pose x is: %f", Object_pose.pose.position.x);
  ROS_INFO("Pose y is: %f", Object_pose.pose.position.y);
  ROS_INFO("Pose z is: %f", Object_pose.pose.position.z);

  tf::Transform point;
  tf::TransformBroadcaster broadc;

  point.setOrigin(tf::Vector3(Object_pose.pose.position.x , Object_pose.pose.position.y,
                                        Object_pose.pose.position.z));
  point.setRotation(tf::Quaternion(Object_pose.pose.orientation.x, Object_pose.pose.orientation.y,
                                     Object_pose.pose.orientation.z, Object_pose.pose.orientation.w));
  broadc.sendTransform(tf::StampedTransform(point, ros::Time::now(), "world_frame", "ORK"));

}

geometry_msgs::Pose collision_avoidance_pick_and_place::PickAndPlace::detect_object()
{
  ros::NodeHandle nh;
  tf::TransformBroadcaster bro;
  ros::Subscriber object_sub = nh.subscribe("/recognized_object_array", 1, &objectCallback);
  ros::Duration(5).sleep();
  ros::spinOnce();
  geometry_msgs::Pose box_pose;

  box_pose = Object_pose.pose;

  ROS_INFO("Pose X is: %f", Object_pose.pose.position.x);
  ROS_INFO("Pose Y is: %f", Object_pose.pose.position.y);
  ROS_INFO("Pose Z is: %f", Object_pose.pose.position.z);
  ROS_INFO("Orientation w is: %f", box_pose.orientation.w);
  ROS_INFO("Orientation x is: %f", box_pose.orientation.x);
  ROS_INFO("Orientation y is: %f", box_pose.orientation.y);
  ROS_INFO("Orientation z is: %f", box_pose.orientation.z);




  // updating box marker for visualization in rviz
  visualization_msgs::Marker marker = cfg.MARKER_MESSAGE;
  cfg.MARKER_MESSAGE.header.frame_id = cfg.WORLD_FRAME_ID;
  cfg.MARKER_MESSAGE.pose = box_pose;
  // offset the box marker so the top surface is aligned with the axis rather than the centroid
  cfg.MARKER_MESSAGE.pose.position.z = box_pose.position.z - 0.5f*cfg.BOX_SIZE.z();
  //cfg.MARKER_MESSAGE.pose.position.z = box_pose.position.z + 0.5f*cfg.BOX_SIZE.z();

  show_box(true);

  return box_pose;
}

