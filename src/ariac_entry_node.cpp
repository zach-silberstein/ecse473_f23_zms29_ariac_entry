#include "ros/ros.h"
#include "std_msgs/String.h"

#include <sstream>
#include "std_srvs/Trigger.h"
#include "std_srvs/SetBool.h"
#include "osrf_gear/Order.h"
#include "osrf_gear/GetMaterialLocations.h"
#include "osrf_gear/LogicalCameraImage.h"



// Declaring a vector of data type.
std::vector<osrf_gear::Order> orders_vector;

// Callback function
void ordersCallback(const osrf_gear::Order::ConstPtr& msg)
{
  osrf_gear::Order temp;
  temp.order_id = msg->order_id;
  temp.shipments = msg->shipments;
  orders_vector.push_back(temp);
}


osrf_gear::LogicalCameraImage agv1;
void cameraCallback_agv1(const osrf_gear::LogicalCameraImage::ConstPtr& msg)
{
  agv1.models = msg->models;
  agv1.pose = msg->pose;
}

osrf_gear::LogicalCameraImage agv2;
void cameraCallback_agv2(const osrf_gear::LogicalCameraImage::ConstPtr& msg)
{
  agv2.models = msg->models;
  agv2.pose = msg->pose;
}

osrf_gear::LogicalCameraImage bin1;
void cameraCallback_bin1(const osrf_gear::LogicalCameraImage::ConstPtr& msg)
{
  bin1.models = msg->models;
  bin1.pose = msg->pose;
}

osrf_gear::LogicalCameraImage bin2;
void cameraCallback_bin2(const osrf_gear::LogicalCameraImage::ConstPtr& msg)
{
  bin2.models = msg->models;
  bin2.pose = msg->pose;
}

osrf_gear::LogicalCameraImage bin3;
void cameraCallback_bin3(const osrf_gear::LogicalCameraImage::ConstPtr& msg)
{
  bin3.models = msg->models;
  bin3.pose = msg->pose;
}

osrf_gear::LogicalCameraImage bin4;
void cameraCallback_bin4(const osrf_gear::LogicalCameraImage::ConstPtr& msg)
{
  bin4.models = msg->models;
  bin4.pose = msg->pose;
}

osrf_gear::LogicalCameraImage bin5;
void cameraCallback_bin5(const osrf_gear::LogicalCameraImage::ConstPtr& msg)
{
  bin5.models = msg->models;
  bin5.pose = msg->pose;
}

osrf_gear::LogicalCameraImage bin6;
void cameraCallback_bin6(const osrf_gear::LogicalCameraImage::ConstPtr& msg)
{
  bin6.models = msg->models;
  bin6.pose = msg->pose;
}

osrf_gear::LogicalCameraImage sen1;
void cameraCallback_sen1(const osrf_gear::LogicalCameraImage::ConstPtr& msg)
{
  sen1.models = msg->models;
  sen1.pose = msg->pose;
}

osrf_gear::LogicalCameraImage sen2;
void cameraCallback_sen2(const osrf_gear::LogicalCameraImage::ConstPtr& msg)
{
  sen2.models = msg->models;
  sen2.pose = msg->pose;
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


  // Service to begin get material
  osrf_gear::GetMaterialLocations materialLocationsType;
  // Create the service client.
  ros::ServiceClient materialLocations = n.serviceClient<osrf_gear::GetMaterialLocations>("/ariac/material_locations");

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
  

  /**
   * A count of how many messages we have sent. This is used to create
   * a unique string for each message.
   */
  int count = 0;
  std::string type;
  std::string unit;
  std::vector<osrf_gear::Model> models;
  geometry_msgs::Pose pose;
  while (ros::ok())
  {


    //check if vector is not empty
    if (not orders_vector.empty() and begin_comp.response.success){
      type = orders_vector[0].shipments[0].products[0].type.c_str();
        ROS_INFO("First order type: %s", type);
        // get element
        materialLocationsType.request.material_type = type;
        
        if (materialLocations.call(materialLocationsType)) {
          unit = materialLocationsType.response.storage_units[0].unit_id.c_str();
            ROS_INFO("First storage unit: %s", unit);

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

          for (osrf_gear::Model model : models){
            if (model.type == type){
              pose = model.pose;
              ROS_INFO("Pose of object in camera's reference frame: %f", pose.orientation.w);
            }
          }

        }
    }
    
    /**
     * This is a message object. You stuff it with data, and then publish it.
     */
    //std_msgs::String msg;

    //std::stringstream ss;
    //ss << "hello world " << count;
   // msg.data = ss.str();

    //ROS_INFO("%s", msg.data.c_str());

    /**
     * The publish() function is how you send messages. The parameter
     * is the message object. The type of this object must agree with the type
     * given as a template parameter to the advertise<>() call, as was done
     * in the constructor above.
     */
    //chatter_pub.publish(msg);

    ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }


  return 0;
}