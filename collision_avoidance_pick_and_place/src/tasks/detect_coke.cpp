#include <collision_avoidance_pick_and_place/pick_and_place.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/ros/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/common/centroid.h>
#include <pcl_ros/transforms.h>
#include <pcl/segmentation/extract_polygonal_prism_data.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/common/common.h>
#include <boost/make_shared.hpp>
#include <tf_conversions/tf_eigen.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <collision_avoidance_pick_and_place/GetTargetPose.h>
#include <math.h>
#include <object_recognition_msgs/RecognizedObjectArray.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/transform_listener.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <object_recognition_msgs/ObjectRecognitionAction.h>


sensor_msgs::PointCloud2 sensor_cloud_msg_;
bool gotpt, gotcloud;
bool gotobject;
geometry_msgs::Pose selected_object_pose;

ros::Publisher cloud_publisher;
float clickedx;
float clickedy;
float clickedz;

double Object_confidence = 0;
std::vector<geometry_msgs::Pose> Object_poses;
std::vector<std::string> Object_ids;
std::string Object_id;
std::string object;

double coke_top, cerealwidth;

void point_cloud_callback(const sensor_msgs::PointCloud2ConstPtr msg)
{
  ROS_INFO("GOT PT CLOUD");
  sensor_cloud_msg_ = sensor_msgs::PointCloud2(*msg);

  if(sensor_cloud_msg_.data.size() == 0)
  {
    ROS_ERROR_STREAM("Cloud message is invalid, returning detection failure");
    //exit(1);
  }
  gotcloud = true;
}

void click_callback(const geometry_msgs::PointStamped pt)
{
  clickedx = pt.point.x;
  clickedy = pt.point.y;
  clickedz = pt.point.z;
  ROS_INFO("Clicked x: %f", clickedx);
  ROS_INFO("Clicked y: %f", clickedy);
  ROS_INFO("Clicked z: %f", clickedz);

  // Lookup transformation between world frame and kinect frame
  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener(tfBuffer);
  geometry_msgs::TransformStamped transformStamped;
  geometry_msgs::Pose tf_pose;
  try{
    transformStamped = tfBuffer.lookupTransform("world_frame", "kinect2_rgb_optical_frame", ros::Time(0), ros::Duration(10.0));
  }
  catch (tf2::TransformException &ex) {
    ROS_WARN("%s",ex.what());
  }
  tf_pose.position.x = transformStamped.transform.translation.x;
  tf_pose.position.y = transformStamped.transform.translation.y;
  tf_pose.position.z = transformStamped.transform.translation.z;
  tf_pose.orientation.w = transformStamped.transform.rotation.w;
  tf_pose.orientation.x = transformStamped.transform.rotation.x;
  tf_pose.orientation.y = transformStamped.transform.rotation.y;
  tf_pose.orientation.z = transformStamped.transform.rotation.z;

  // Perform frame transformation to find cloud pose in world_frame
  tf::Transform tf_tf;
  tf::poseMsgToTF(tf_pose, tf_tf);

  sensor_msgs::PointCloud2 transformed_cloud;
  pcl_ros::transformPointCloud("world_frame",tf_tf,sensor_cloud_msg_,transformed_cloud);

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg (transformed_cloud, *cloud);

  pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
  kdtree.setInputCloud (cloud);

  pcl::PointXYZ searchPoint;
  searchPoint.x = clickedx;
  searchPoint.y = clickedy;
  searchPoint.z = clickedz;

  float radius = 0.01;
  //int K = 10;
  //    std::vector<int> pointIdxNKNSearch(K);
  //    std::vector<float> pointNKNSquaredDistance(K);
  std::vector<int> pointIdxRadiusSearch;
  std::vector<float> pointRadiusSquaredDistance;
  //    10 closest neighbours
  //    if(kdtree.nearestKSearch(searchPoint,K,pointIdxNKNSearch, pointNKNSquaredDistance) > 0 ){
  //      for (size_t i = 0; i < pointIdxNKNSearch.size (); ++i){
  //        ROS_WARN("Match");
  //      }
  //    }

  //if there are points that are within the radius
  if ( kdtree.radiusSearch (searchPoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0 )
  {
    gotpt = true;
  }




}

void collision_avoidance_pick_and_place::PickAndPlace::objectCallback(const object_recognition_msgs::RecognizedObjectArray objects_msg)
{
  float oldDistance = 999.0; 
  float bowl_distance = 999.0;
  float bowl_confidence = 0.0;
  bool no_bowl = true;
  std::string mug_id = "73fad4bb83b7da00028b34e9e0002a61";
//  std::string coke_id = "b6fa4d407a0d33f20f5bcc0849000dfc";
  std::string coke_id = "11111111111111111111111111111";
  std::string cereal_id = "254ca5b81b84be710da3d698780080ea";
  std::string bowl_id = "b6fa4d407a0d33f20f5bcc0849003d52";


//  tf::StampedTransform tf_tf;
//  TransformListenerPtr transform_listener_ptr;
//  try{
////    transform_listener_ptr->waitForTransform("world_frame", "kinect2_rgb_optical_frame", ros::Time(0.0f), ros::Duration(3.0));
////    transform_listener_ptr->lookupTransform("world_frame", "kinect2_rgb_optical_frame", ros::Time(0.0f), tf_tf);

//  }
//  catch (tf::TransformException ex){
//    ROS_ERROR("%s",ex.what());
//    exit(1);
//  }
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);
    geometry_msgs::TransformStamped transformStamped;
    try{
      ROS_INFO("Waiting for transform");
      transformStamped = tfBuffer.lookupTransform("world_frame", "kinect2_rgb_optical_frame", ros::Time(0), ros::Duration(10.0));

    }
    catch (tf2::TransformException &ex) {
      ROS_WARN("%s",ex.what());
    }
    geometry_msgs::Pose tf_pose, fixate_world_pose;
    tf_pose.position.x = transformStamped.transform.translation.x;
    tf_pose.position.y = transformStamped.transform.translation.y;
    tf_pose.position.z = transformStamped.transform.translation.z;
    tf_pose.orientation.w = transformStamped.transform.rotation.w;
    tf_pose.orientation.x = transformStamped.transform.rotation.x;
    tf_pose.orientation.y = transformStamped.transform.rotation.y;
    tf_pose.orientation.z = transformStamped.transform.rotation.z;
    tf::Transform tf_tf;
    tf::poseMsgToTF(tf_pose, tf_tf);

    //Converting 3D fixation from kinect frame to world frame
//    tf::Transform fixate_camera_tf;
//    fixate_camera_tf.setOrigin(tf::Vector3(clickedx , clickedy,
//                                          clickedz));
//    fixate_camera_tf.setRotation(tf::Quaternion::getIdentity());
//    tf::Transform fixate_world_tf = tf_tf*fixate_camera_tf;
//    tf::poseTFToMsg(fixate_world_tf,fixate_world_pose);

    for (int i = 0; i < objects_msg.objects.size(); ++i) {

      // Load detected objects into array
      Object_poses.push_back(objects_msg.objects[i].pose.pose.pose);
      Object_ids.push_back(objects_msg.objects[i].type.key.c_str());

      // Convert object pose to tf
      tf::Transform object_tf;
      tf::poseMsgToTF(Object_poses[i],object_tf);

      // Exit the program if object is not found
      if (!Object_poses[i].position.x){
        ROS_ERROR_STREAM("Objects inside robot detected");
        return void();
      } 

      geometry_msgs::Pose Object_pose;
      tf::Transform transformed = tf_tf*object_tf;
      tf::poseTFToMsg(transformed,Object_pose);

      ROS_INFO("Pose X is: %f", Object_pose.position.x);
      ROS_INFO("Pose Y is: %f", Object_pose.position.y);
      ROS_INFO("Pose Z is: %f", Object_pose.position.z);
      ROS_INFO("Orientation W is: %f", Object_pose.orientation.w);
      ROS_INFO("Orientation X is: %f", Object_pose.orientation.x);
      ROS_INFO("Orientation Y is: %f", Object_pose.orientation.y);
      ROS_INFO("Orientation Z is: %f", Object_pose.orientation.z);


      float objectx = Object_pose.position.x;
      float objecty = Object_pose.position.y;
      float objectz = Object_pose.position.z;      

      //Compare distance of objects pose to the clicked point
      float distance = pow((clickedx - objectx),2) + pow((clickedy - objecty),2)
          + pow((clickedz - objectz),2);
//      float distance = pow((fixate_world_pose.position.x - objectx),2) + pow((fixate_world_pose.position.y - objecty),2)
//          + pow((fixate_world_pose.position.z - objectz),2);

      ROS_INFO("Distance: %f", distance);
      ROS_INFO("%d", i);
      ROS_INFO_STREAM("id " << objects_msg.objects[i].type.key.c_str());
      if (distance < oldDistance){
        Object_id = objects_msg.objects[i].type.key.c_str();


        // Label the objects and assign their offset for grasping
        if (coke_id.compare(objects_msg.objects[i].type.key.c_str()) == 0){
          object = "coke";
          ROS_INFO("Coke Distance: %f", distance);
        }
        else if (cereal_id.compare(objects_msg.objects[i].type.key.c_str()) == 0){
          object = "cereal";
          ROS_INFO("Cereal Distance: %f", distance);
          selected_object_pose = Object_pose;
        }
        else if (mug_id.compare(objects_msg.objects[i].type.key.c_str()) == 0){
          object = "mug";
          ROS_INFO("Mug Distance: %f", distance);
          selected_object_pose = Object_pose;
        }
        else if (bowl_id.compare(objects_msg.objects[i].type.key.c_str()) == 0){
          object = "bowl";
          ROS_INFO("Bowl Distance: %f", distance);
        }
        oldDistance = distance;
      }      

      ROS_INFO("Selected Object: %s", object.c_str());


      tf::Transform bowl_tf;
      if (bowl_id.compare(objects_msg.objects[i].type.key.c_str()) == 0){        
        if (objects_msg.objects[i].confidence > bowl_confidence && distance < bowl_distance) {

          bowl_confidence = objects_msg.objects[i].confidence;

          bowl_tf.setOrigin(tf::Vector3(Object_pose.position.x , Object_pose.position.y,
                                        Object_pose.position.z));
          bowl_tf.setRotation(tf::Quaternion(Object_pose.orientation.x, Object_pose.orientation.y,
                                             Object_pose.orientation.z, Object_pose.orientation.w));
          transform_broadcaster->sendTransform(tf::StampedTransform(bowl_tf, ros::Time::now(), "world_frame", "bowl"));

          bowl_distance = distance;
          no_bowl = false;
        }
      }      
    }
    if (no_bowl && object.compare("cereal") == 0) {
      ROS_INFO("BOWL NOT DETECTED FOR CEREAL");
      exit(1);
    }

    tf::Transform point;

    point.setOrigin(tf::Vector3(selected_object_pose.position.x , selected_object_pose.position.y,
                                selected_object_pose.position.z));
    point.setRotation(tf::Quaternion(selected_object_pose.orientation.x, selected_object_pose.orientation.y,
                                     selected_object_pose.orientation.z, selected_object_pose.orientation.w));
    transform_broadcaster->sendTransform(tf::StampedTransform(point, ros::Time::now(), "world_frame", "ORK"));
    ROS_INFO("ORK Pose X is: %f", selected_object_pose.position.x);
    ROS_INFO("ORK Pose Y is: %f", selected_object_pose.position.y);
    ROS_INFO("ORK Pose Z is: %f", selected_object_pose.position.z);
    gotobject = true;
}

void collision_avoidance_pick_and_place::PickAndPlace::fixation_callback(const geometry_msgs::Point pt)
{
  if (gotcloud) {

    clickedx = pt.x;
    clickedy = pt.y;
    clickedz = pt.z;
    ROS_INFO("Clicked x: %f", clickedx);
    ROS_INFO("Clicked y: %f", clickedy);
    ROS_INFO("Clicked z: %f", clickedz);
    ROS_INFO("FIXATION");
    // Lookup transformation between world frame and kinect frame
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);
    geometry_msgs::TransformStamped transformStamped;
    geometry_msgs::Pose tf_pose;
    try{
      transformStamped = tfBuffer.lookupTransform("world_frame", "kinect2_rgb_optical_frame", ros::Time(0), ros::Duration(10.0));
    }
    catch (tf2::TransformException &ex) {
      ROS_WARN("%s",ex.what());
    }
    tf_pose.position.x = transformStamped.transform.translation.x;
    tf_pose.position.y = transformStamped.transform.translation.y;
    tf_pose.position.z = transformStamped.transform.translation.z;
    tf_pose.orientation.w = transformStamped.transform.rotation.w;
    tf_pose.orientation.x = transformStamped.transform.rotation.x;
    tf_pose.orientation.y = transformStamped.transform.rotation.y;
    tf_pose.orientation.z = transformStamped.transform.rotation.z;

    // Perform frame transformation to find cloud pose in world_frame
    tf::Transform tf_tf;
    tf::poseMsgToTF(tf_pose, tf_tf);

    sensor_msgs::PointCloud2 transformed_cloud;
    pcl_ros::transformPointCloud("world_frame",tf_tf,sensor_cloud_msg_,transformed_cloud);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg (transformed_cloud, *cloud);

    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    kdtree.setInputCloud (cloud);


    //Converting 3D fixation from kinect frame to world frame
    geometry_msgs::Pose fixate_world_pose;
    tf::Transform fixate_camera_tf;
    fixate_camera_tf.setOrigin(tf::Vector3(clickedx , clickedy,
                                           clickedz));
    fixate_camera_tf.setRotation(tf::Quaternion::getIdentity());
    tf::Transform fixate_world_tf = tf_tf*fixate_camera_tf;
    transform_broadcaster->sendTransform(tf::StampedTransform(fixate_world_tf, ros::Time::now(), "world_frame", "fixate_world"));
    tf::poseTFToMsg(fixate_world_tf,fixate_world_pose);

    pcl::PointXYZ searchPoint;
    searchPoint.x = fixate_world_pose.position.x;
    searchPoint.y = fixate_world_pose.position.y;
    searchPoint.z = fixate_world_pose.position.z;

    float radius = 0.05;
    std::vector<int> pointIdxRadiusSearch;
    std::vector<float> pointRadiusSquaredDistance;

    ROS_INFO("here");
    //if there are points that are within the radius
    if ( kdtree.radiusSearch (searchPoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0 )
    {
      gotpt = true;
      ROS_INFO("IN");
    }

  }
}

geometry_msgs::Pose collision_avoidance_pick_and_place::PickAndPlace::detect_coke()
{
  ros::NodeHandle nh;
  tf::TransformBroadcaster bro;
  //cloud_publisher = nh.advertise<sensor_msgs::PointCloud2>("points",1);

  gotpt = false;
  gotcloud = false;
  gotobject = false;

//  while (!gotcloud) {
//    ros::Subscriber ork_cloud_subscriber = nh.subscribe("/real_icpin_ref",1,&point_cloud_callback);
//    ros::spinOnce();
//    ros::Duration(0.5).sleep();
//  }
  while(!gotpt){
    ros::Subscriber ork_cloud_subscriber = nh.subscribe("/real_icpin_ref",1,&point_cloud_callback);
//    ros::Subscriber fixation_3d_subscriber = nh.subscribe("/fixation_3d", 1, &collision_avoidance_pick_and_place::PickAndPlace::fixation_callback, this);
            ros::Subscriber clicked_point_subscriber = nh.subscribe("/clicked_point",1,&click_callback);
    ros::spinOnce();
    ros::Duration(0.5).sleep();



  }


  while (!gotobject) {
    ros::Subscriber object_sub = nh.subscribe("/recognized_object_array", 1, &collision_avoidance_pick_and_place::PickAndPlace::objectCallback, this);
    ros::spinOnce();
    ros::Duration(0.5).sleep();

  }  

  // updating box marker for visualization in rviz
  visualization_msgs::Marker marker = cfg.MARKER_MESSAGE;
  cfg.MARKER_MESSAGE.header.frame_id = cfg.WORLD_FRAME_ID;
  cfg.MARKER_MESSAGE.pose = selected_object_pose;


  // if object is coke, set global variable to coke
  if (object.compare("coke") == 0) {
   cfg.selected_object = "coke";
   // offset the box marker so the top surface is aligned with the axis rather than the centroid
//   cfg.MARKER_MESSAGE.pose.position.z = selected_object_pose.position.z - coke_top;
  }
  else if (object.compare("cereal") == 0) {
    cfg.selected_object = "cereal";
//    cfg.MARKER_MESSAGE.pose.position.x = selected_object_pose.position.x + cerealwidth;
  }
  else if (object.compare("mug") == 0) {
    cfg.selected_object = "mug";
  }
  else if (object.compare("bowl") == 0) {
    cfg.selected_object = "bowl";
  }

//    //cfg.MARKER_MESSAGE.pose.position.z = selected_object_pose.position.z + 0.5f*cfg.BOX_SIZE.z();

    show_box(true);

    return selected_object_pose;

}



//geometry_msgs::Pose collision_avoidance_pick_and_place::PickAndPlace::detect_coke()
//{
//  ros::NodeHandle nh;
//  //tf::TransformBroadcaster bro;



//  //  //system("rosrun object_recognition_ros client");
//  //  // Running actionlib client, the same as executing same command as above
//  //  actionlib::SimpleActionClient<object_recognition_msgs::ObjectRecognitionAction> ac("recognize_objects", true);

//  //  ROS_INFO("Waiting for action server to start.");
//  //  // wait for the action server to start
//  //  ac.waitForServer(); //will wait for infinite time

//  //  ROS_INFO("Action server started, sending goal.");
//  //  // send a goal to the action
//  //  object_recognition_msgs::ObjectRecognitionGoal goal;
//  //  ac.sendGoal(goal);

//  //  //wait for the action to return
//  //  bool finished_before_timeout = ac.waitForResult(ros::Duration(30.0));

//  //  if (finished_before_timeout)
//  //  {
//  //    actionlib::SimpleClientGoalState state = ac.getState();
//  //    ROS_INFO("Action finished: %s",state.toString().c_str());
//  //    ros::Subscriber object_sub = nh.subscribe("/recognized_object_array", 1, &objectCallback);
//  //    ros::Duration(5).sleep();
//  //    ros::spinOnce();
//  //  }
//  //  else
//  //    ROS_INFO("Action did not finish before the time out.");



//  show_box(true);

//  return selected_object_pose;
//}

//  if (firstCB == false && (int)objects_msg.objects.size() == 1) {
//    Object_id.assign(objects_msg.objects[0].type.key.c_str());
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












