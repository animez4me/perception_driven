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
bool gotpt = false;
bool gotobject = false;
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

float coke_top;

void point_cloud_callback(const sensor_msgs::PointCloud2ConstPtr msg)
{
  ROS_INFO("GOT PT CLOUD");
  sensor_cloud_msg_ = sensor_msgs::PointCloud2(*msg);

  if(sensor_cloud_msg_.data.size() == 0)
  {
    ROS_ERROR_STREAM("Cloud message is invalid, returning detection failure");
    //exit(1);
  }
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

  // Perform frame transformation to find object pose in world_frame
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
    //      for (size_t i = 0; i < pointIdxRadiusSearch.size (); ++i){
    //        ROS_WARN("Match");
    //        ROS_INFO("X: %f", cloud->points[ pointIdxRadiusSearch[i] ].x);
    //        ROS_INFO("%d", i);
    //        ROS_INFO("Clicked x: %f", clickedx);
    //      }

  }


  // convert from message to point cloud
  //  Cloud::Ptr sensor_cloud_ptr(new Cloud());
  //  pcl::fromROSMsg<pcl::PointXYZ>(msg,*sensor_cloud_ptr);

  //  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  //  pcl::fromROSMsg (transformed_cloud, *cloud);



}

void objectCallback(const object_recognition_msgs::RecognizedObjectArray objects_msg)
{
  float oldDistance = 999.0; 

  std::string coke_id = "2c536957edd6b006c361083178002c21";
  std::string mug_id = "ad1d4a1a8ef2e426daa1247db2001226";
  std::string juice_id = "9b516faff37644e7793290391807a29d";
  std::string book_id = "928e269d54edd1190dc741928604953c";

//  tf::StampedTransform tf_tf;
//  TransformListenerPtr transform_listener_ptr;
//  try{
//    transform_listener_ptr->waitForTransform("world_frame", "kinect2_rgb_optical_frame", ros::Time(0.0f), ros::Duration(3.0));
//    transform_listener_ptr->lookupTransform("world_frame", "kinect2_rgb_optical_frame", ros::Time(0.0f), tf_tf);

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
    geometry_msgs::Pose tf_pose;
    tf_pose.position.x = transformStamped.transform.translation.x;
    tf_pose.position.y = transformStamped.transform.translation.y;
    tf_pose.position.z = transformStamped.transform.translation.z;
    tf_pose.orientation.w = transformStamped.transform.rotation.w;
    tf_pose.orientation.x = transformStamped.transform.rotation.x;
    tf_pose.orientation.y = transformStamped.transform.rotation.y;
    tf_pose.orientation.z = transformStamped.transform.rotation.z;
    tf::Transform tf_tf;
    tf::poseMsgToTF(tf_pose, tf_tf);


    geometry_msgs::Pose Object_pose;
    tf::Transform point;
    tf::TransformBroadcaster broadc;
    for (int i = 0; i < objects_msg.objects.size(); ++i) {

      // Load detected objects into array
      Object_poses.push_back(objects_msg.objects[i].pose.pose.pose);
      Object_ids.push_back(objects_msg.objects[i].type.key.c_str());

      // Convert object pose to tf
      tf::Transform object_tf;
      tf::poseMsgToTF(Object_poses[i],object_tf);

      // Exit the program if object is not found
      if (!Object_poses[i].position.x){
        ROS_ERROR_STREAM("None or too many objects detected");
        exit(1);
      }

      // Lookup transformation between world frame and kinect frame
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

      // Perform frame transformation to find object pose in world_frame
      //  tf::Transform tf_tf, transformed;

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
      float object_height;

      //Compare distance of objects pose to the clicked point
      float distance = pow((clickedx - objectx),2) + pow((clickedy - objecty),2) + pow((clickedz - objectz),2);

      if (distance < oldDistance){
        Object_id = objects_msg.objects[i].type.key.c_str();
        selected_object_pose = Object_pose;

        if (coke_id.compare(objects_msg.objects[i].type.key.c_str()) == 0){
          object = "coke";
          selected_object_pose.position.z = selected_object_pose.position.z + coke_top;
        }
        else if (mug_id.compare(objects_msg.objects[i].type.key.c_str()) == 0){
          object = "mug";
        }
        else if (juice_id.compare(objects_msg.objects[i].type.key.c_str()) == 0){
          object = "juice";
        }
        else if (book_id.compare(objects_msg.objects[i].type.key.c_str()) == 0){
          object = "book";
        }
        oldDistance = distance;
      }
      ROS_INFO("%d", i);
      ROS_INFO("Selected Object: %s", object.c_str());
      ROS_INFO("Distance: %f", distance);
    }
    point.setOrigin(tf::Vector3(selected_object_pose.position.x , selected_object_pose.position.y,
                                selected_object_pose.position.z));
    point.setRotation(tf::Quaternion(selected_object_pose.orientation.x, selected_object_pose.orientation.y,
                                     selected_object_pose.orientation.z, selected_object_pose.orientation.w));
    broadc.sendTransform(tf::StampedTransform(point, ros::Time::now(), "world_frame", "ORK"));
    ROS_INFO("ORK Pose X is: %f", selected_object_pose.position.x);
    ROS_INFO("ORK Pose Y is: %f", selected_object_pose.position.y);
    ROS_INFO("ORK Pose Z is: %f", selected_object_pose.position.z);
    gotobject = true;
}

geometry_msgs::Pose collision_avoidance_pick_and_place::PickAndPlace::detect_coke()
{
  ros::NodeHandle nh;
  tf::TransformBroadcaster bro;
  //cloud_publisher = nh.advertise<sensor_msgs::PointCloud2>("points",1);  
  coke_top = cfg.COKE_SIZE[0] / 2;

  while(!gotpt){
    ros::Subscriber clicked_point_subscriber = nh.subscribe("/clicked_point",1,&click_callback);

    ros::Subscriber ork_cloud_subscriber = nh.subscribe("/real_icpin_ref",1,&point_cloud_callback);
    ros::Duration(3).sleep();
    ros::spinOnce();
  }


  while (!gotobject) {
    ros::Subscriber object_sub = nh.subscribe("/recognized_object_array", 1, &objectCallback);

    ros::spinOnce();
    ros::Duration(3).sleep();

  }  

  // if object is coke, set global variable to coke
  if (object.compare("coke") == 0){
   cfg.selected_object = "coke";
  }

    // updating box marker for visualization in rviz
    visualization_msgs::Marker marker = cfg.MARKER_MESSAGE;
    cfg.MARKER_MESSAGE.header.frame_id = cfg.WORLD_FRAME_ID;
    cfg.MARKER_MESSAGE.pose = selected_object_pose;
    // offset the box marker so the top surface is aligned with the axis rather than the centroid
    //cfg.MARKER_MESSAGE.pose.position.z = selected_object_pose.position.z - 0.5f*cfg.BOX_SIZE.z();
    //cfg.MARKER_MESSAGE.pose.position.z = selected_object_pose.position.z + 0.5f*cfg.BOX_SIZE.z();

    show_box(true);

    return selected_object_pose;

}





//geometry_msgs::Pose collision_avoidance_pick_and_place::PickAndPlace::detect_coke()
//{
//  ros::NodeHandle nh;
//  //tf::TransformBroadcaster bro;

//  ros::Subscriber object_sub = nh.subscribe("/recognized_object_array", 1, &objectCallback);
//  //ros::Duration(5).sleep();
//  ros::spinOnce();

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

//  for (int i = 0; i < cloud->points.size(); ++i) {
//    if (clickedx == cloud->points[i].x){
//      ROS_WARN("MATCH");
//      exit(1);
//                if (clickedy == cloud->points[i].y){
//                  if (clickedz == cloud->points[i].z){
//                    ROS_INFO("x : %f", cloud->points[i].x);
//                    ROS_INFO("y : %f", cloud->points[i].y);
//                    ROS_INFO("z : %f", cloud->points[i].z);
//                  }
//                }
//    }
//  }










