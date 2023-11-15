#include "ros/ros.h"
#include "std_msgs/String.h"

#include <sstream>
#include "std_srvs/Trigger.h"
#include "std_srvs/SetBool.h"
#include "osrf_gear/Order.h"
#include "osrf_gear/GetMaterialLocations.h"
#include "osrf_gear/LogicalCameraImage.h"

// Transformation header files
#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "geometry_msgs/TransformStamped.h"

// Declaring a vector of data type.
std::vector<osrf_gear::Order> orders_vector;

// Callback function
void ordersCallback(const osrf_gear::Order::ConstPtr& msg)
{
  orders_vector.push_back(*msg);
}


osrf_gear::LogicalCameraImage agv1;
void cameraCallback_agv1(const osrf_gear::LogicalCameraImage::ConstPtr& msg)
{
  agv1 = *msg;
}

osrf_gear::LogicalCameraImage agv2;
void cameraCallback_agv2(const osrf_gear::LogicalCameraImage::ConstPtr& msg)
{
  agv2 = *msg;
}

osrf_gear::LogicalCameraImage bin1;
void cameraCallback_bin1(const osrf_gear::LogicalCameraImage::ConstPtr& msg)
{
  bin1 = *msg;
}

osrf_gear::LogicalCameraImage bin2;
void cameraCallback_bin2(const osrf_gear::LogicalCameraImage::ConstPtr& msg)
{
  bin2 = *msg;
}

osrf_gear::LogicalCameraImage bin3;
void cameraCallback_bin3(const osrf_gear::LogicalCameraImage::ConstPtr& msg)
{
  bin3 = *msg;
}

osrf_gear::LogicalCameraImage bin4;
void cameraCallback_bin4(const osrf_gear::LogicalCameraImage::ConstPtr& msg)
{
  bin4 = *msg;
}

osrf_gear::LogicalCameraImage bin5;
void cameraCallback_bin5(const osrf_gear::LogicalCameraImage::ConstPtr& msg)
{
  bin5 = *msg;
}

osrf_gear::LogicalCameraImage bin6;
void cameraCallback_bin6(const osrf_gear::LogicalCameraImage::ConstPtr& msg)
{
  bin6 = *msg;
}

osrf_gear::LogicalCameraImage sen1;
void cameraCallback_sen1(const osrf_gear::LogicalCameraImage::ConstPtr& msg)
{
  sen1 = *msg;
}

osrf_gear::LogicalCameraImage sen2;
void cameraCallback_sen2(const osrf_gear::LogicalCameraImage::ConstPtr& msg)
{
  sen2 = *msg;
}

/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */
int main(int argc, char **argv)
{
  /**
   * The ros::init() function needs to see argc and argv so that it can perform
   * any ROS arguments and name remapping that were provided at the command line.
   * For programmatic remappings you can use a different version of init() which takes
   * remappings directly, but for most command-line programs, passing argc and argv is
   * the easiest way to do it.  The third argument to init() is the name of the node.
   *
   * You must call one of the versions of ros::init() before using any other
   * part of the ROS system.
   */
  ros::init(argc, argv, "talker");

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
  ros::NodeHandle n;
  // Declare the transformation buffer to maintain a list of transformations
  tf2_ros::Buffer tfBuffer;
  // Instantiate a listener that listens to the tf and tf_static topics and to update the buffer.
  tf2_ros::TransformListener tfListener(tfBuffer);

  /**
   * The advertise() function is how you tell ROS that you want to
   * publish on a given topic name. This invokes a call to the ROS
   * master node, which keeps a registry of who is publishing and who
   * is subscribing. After this advertise() call is made, the master
   * node will notify anyone who is trying to subscribe to this topic name,
   * and they will in turn negotiate a peer-to-peer connection with this
   * node.  advertise() returns a Publisher object which allows you to
   * publish messages on that topic through a call to publish().  Once
   * all copies of the returned Publisher object are destroyed, the topic
   * will be automatically unadvertised.
   *
   * The second parameter to advertise() is the size of the message queue
   * used for publishing messages.  If messages are published more quickly
   * than we can send them, the number here specifies how many messages to
   * buffer up before throwing some away.
   */
  //ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);

  ros::Rate loop_rate(10);

  // Service to begin compition
  std_srvs::Trigger begin_comp;
  // Create the service client.
  ros::ServiceClient begin_client = n.serviceClient<std_srvs::Trigger>("/ariac/start_competition");

  // Subscribe to orders cameras
  ros::Subscriber sub1 = n.subscribe("/ariac/logical_camera_agv1", 1000, cameraCallback_agv1);
  ros::Subscriber sub2 = n.subscribe("/ariac/logical_camera_agv2", 1000, cameraCallback_agv2);
  ros::Subscriber sub3 = n.subscribe("/ariac/logical_camera_bin1", 1000, cameraCallback_bin1);
  ros::Subscriber sub4 = n.subscribe("/ariac/logical_camera_bin2", 1000, cameraCallback_bin2);
  ros::Subscriber sub5 = n.subscribe("/ariac/logical_camera_bin3", 1000, cameraCallback_bin3);
  ros::Subscriber sub6 = n.subscribe("/ariac/logical_camera_bin4", 1000, cameraCallback_bin4);
  ros::Subscriber sub7 = n.subscribe("/ariac/logical_camera_bin5", 1000, cameraCallback_bin5);
  ros::Subscriber sub8 = n.subscribe("/ariac/logical_camera_bin6", 1000, cameraCallback_bin6);
  ros::Subscriber sub9 = n.subscribe("/ariac/quality_control_sensor_1", 1000, cameraCallback_sen1);
  ros::Subscriber sub10 = n.subscribe("/ariac/quality_control_sensor_2", 1000, cameraCallback_sen2);
  

  // Try to start competition
  if (begin_client.call(begin_comp)){
    // If service call returns, check the output to make sure it actually started
    if (begin_comp.response.success){
        ROS_INFO("Competition service called successfully: %s", \
            begin_comp.response.message.c_str());
    }
    else {
        ROS_WARN("Competition service returned failure: %s", \
            begin_comp.response.message.c_str());
    }
  }
  else {
    ROS_ERROR("Competition service call failed! Goodness Gracious!!");
  }

  // Subscribe to orders topic
  ros::Subscriber sub = n.subscribe("/ariac/orders", 1000, ordersCallback);

  // Clearing/initializing vector
  orders_vector.clear();

  // Wait for first order to come in
  while (orders_vector.empty()){
    ros::spinOnce();
    loop_rate.sleep();
  }

  // Get part of the first order
  std::string type;
  type = orders_vector[0].shipments[0].products[0].type;
  ROS_INFO("First order type: %s", type.c_str());


  // Service to begin get material
  osrf_gear::GetMaterialLocations materialLocationsType;
  // Create the service client.
  ros::ServiceClient materialLocations = n.serviceClient<osrf_gear::GetMaterialLocations>("/ariac/material_locations");


  // Get the location of the first part
  materialLocationsType.request.material_type = type;
  std::string unit;
  if (materialLocations.call(materialLocationsType)) {
    unit = materialLocationsType.response.storage_units[0].unit_id;
    ROS_INFO("First storage unit: %s", unit.c_str());
  }


  // Getting pose of part
  // Check correct camera
  std::vector<osrf_gear::Model> models;
  if (unit.compare("agv1") == 0) {
    models = agv1.models;
  }
  else if (unit.compare("agv2") == 0) {
    models = agv2.models;
  }
  else if (unit.compare("bin1") == 0) {
    models = bin1.models;
  }
  else if (unit.compare("bin2") == 0) {
    models = bin2.models;
  }
  else if (unit.compare("bin3") == 0) {
    models = bin3.models;
  }
   else if (unit.compare("bin4") == 0) {
    models = bin4.models;
  }
  else if (unit.compare("bin5") == 0) {
    models = bin5.models;
  }
  else if (unit.compare("bin6") == 0) {
    models = bin6.models;
  }
  else if (unit.compare("sen1") == 0) {
    models = sen1.models;
  }
  else if (unit.compare("sen2") == 0) {
    models = sen2.models;
  }
  // Get pose of part in frame of correct camera
  geometry_msgs::Pose pose;
  for (osrf_gear::Model model : models){
    if (model.type == type){
      pose = model.pose;
      ROS_INFO("Pose of object in camera's reference frame (w,x,y,z),(X,Y,Z): (%f,%f,%f,%f),(%f,%f,%f)",\
      pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z, \
        pose.position.x, pose.position.y, pose.position.z);
      break;
    }
  }


  //Retrieve the transformation
  geometry_msgs::TransformStamped tfStamped;
  try {
    tfStamped = tfBuffer.lookupTransform("arm1_base_link", "logical_camera_bin4_frame",
    ros::Time(0.0), ros::Duration(1.0));
    ROS_DEBUG("Transform to [%s] from [%s]", tfStamped.header.frame_id.c_str(),
    tfStamped.child_frame_id.c_str());
  } catch (tf2::TransformException &ex) {
    ROS_ERROR("%s", ex.what());
  }
  //tf2_ross::Buffer.lookupTransform("to_frame", "from_frame", "how_recent", "how_long_to_wait_for_transform");

  // Create variables
  geometry_msgs::PoseStamped part_pose, goal_pose;
  // Copy pose from the logical camera.
  part_pose.pose = pose;
  tf2::doTransform(part_pose, goal_pose, tfStamped);

  // Add height to the goal pose.
  goal_pose.pose.position.z += 0.10; // 10 cm above the part
  // Tell the end effector to rotate 90 degrees around the y-axis (in quaternions...).
  goal_pose.pose.orientation.w = 0.707;
  goal_pose.pose.orientation.x = 0.0;
  goal_pose.pose.orientation.y = 0.707;
  goal_pose.pose.orientation.z = 0.0;


  return 0;
}