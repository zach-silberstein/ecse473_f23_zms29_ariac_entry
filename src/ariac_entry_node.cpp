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
// Include the header for retrieving the current state of the robot joints.
#include "sensor_msgs/JointState.h"

// Include the header for generating a joint trajectory.
#include "trajectory_msgs/JointTrajectory.h"

#include "ik_service/PoseIK.h"
#include <cstdlib>
#include "geometry_msgs/Pose.h"

#include "ur_kinematics/ur_kin.h"

// Declaring a vector of data type.
std::vector<osrf_gear::Order> orders_vector;

// Callback function
void ordersCallback(const osrf_gear::Order::ConstPtr& msg)
{
  orders_vector.push_back(*msg);
}

// Callback function for joint states
sensor_msgs::JointState joint_states;
void jointStatesCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
  joint_states = *msg;
}

osrf_gear::LogicalCameraImage cameras [10];
void cameraCallback_agv1(const osrf_gear::LogicalCameraImage::ConstPtr& msg)
{
  cameras[0] = *msg;
}

void cameraCallback_agv2(const osrf_gear::LogicalCameraImage::ConstPtr& msg)
{
  cameras[1] = *msg;
}

void cameraCallback_bin1(const osrf_gear::LogicalCameraImage::ConstPtr& msg)
{
  cameras[2] = *msg;
}

void cameraCallback_bin2(const osrf_gear::LogicalCameraImage::ConstPtr& msg)
{
  cameras[3] = *msg;
}

void cameraCallback_bin3(const osrf_gear::LogicalCameraImage::ConstPtr& msg)
{
  cameras[4] = *msg;
}

void cameraCallback_bin4(const osrf_gear::LogicalCameraImage::ConstPtr& msg)
{
  cameras[5] = *msg;
}

void cameraCallback_bin5(const osrf_gear::LogicalCameraImage::ConstPtr& msg)
{
  cameras[6] = *msg;
}

void cameraCallback_bin6(const osrf_gear::LogicalCameraImage::ConstPtr& msg)
{
  cameras[7] = *msg;
}

void cameraCallback_sen1(const osrf_gear::LogicalCameraImage::ConstPtr& msg)
{
  cameras[8] = *msg;
}

void cameraCallback_sen2(const osrf_gear::LogicalCameraImage::ConstPtr& msg)
{
  cameras[9] = *msg;
}

std::string getOrderType(){
  // Get part of the first order
  std::string type;
  type = orders_vector[0].shipments[0].products[0].type;
  ROS_INFO_THROTTLE(5, "First order type: %s", type.c_str());
  return type;
}

std::string getPartLocation(std::string type, ros::ServiceClient materialLocations) {
  osrf_gear::GetMaterialLocations materialLocationsType;
  // Get the location of the first part
  materialLocationsType.request.material_type = type;
  std::string unit;
  if (materialLocations.call(materialLocationsType)) {
    int i = 0;
    std::string belt = "belt";
    while (belt.compare(materialLocationsType.response.storage_units[i].unit_id) == 0) {
      i++;
    }
    unit = materialLocationsType.response.storage_units[i].unit_id;
    ROS_INFO_THROTTLE(5, "First storage unit: %s", unit.c_str());
  }
  return unit;

}

geometry_msgs::Pose getCameraPose(std::string type, std::string unit) {
  // Getting pose of part
  // Check correct camera
  std::vector<osrf_gear::Model> models;
  if (unit.compare("agv1") == 0) {
    models = cameras[0].models;
  }
  else if (unit.compare("agv2") == 0) {
    models = cameras[1].models;
  }
  else if (unit.compare("bin1") == 0) {
    models = cameras[2].models;
  }
  else if (unit.compare("bin2") == 0) {
    models = cameras[3].models;
  }
  else if (unit.compare("bin3") == 0) {
    models = cameras[4].models;
  }
   else if (unit.compare("bin4") == 0) {
    models = cameras[5].models;
  }
  else if (unit.compare("bin5") == 0) {
    models = cameras[6].models;
  }
  else if (unit.compare("bin6") == 0) {
    models = cameras[7].models;
  }
  else if (unit.compare("sen1") == 0) {
    models = cameras[8].models;
  }
  else if (unit.compare("sen2") == 0) {
    models = cameras[9].models;
  }

  // Get pose of part in frame of correct camera
  geometry_msgs::Pose pose;
  for (osrf_gear::Model model : models){
    if (model.type == type){
      pose = model.pose;
      ROS_INFO_THROTTLE(5, "Pose of object in camera's reference frame (w,x,y,z),(X,Y,Z): (%f,%f,%f,%f),(%f,%f,%f)",\
      pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z, \
        pose.position.x, pose.position.y, pose.position.z);
      break;
    }
  }
  return pose;
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
  // Create the service client to start competetion.
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
  
  // Subscribe to joint states topic
  ros::Subscriber joint_states_h = n.subscribe("/ariac/arm1/joint_states", 1000, jointStatesCallback);

  // Add ik_service
  ros::service::waitForService("pose_ik",-1);
  ros::ServiceClient ik_client = n.serviceClient<ik_service::PoseIK>("pose_ik");

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

  /*
  // Wait for first order to come in
  while (orders_vector.empty()){
    ros::spinOnce();
    loop_rate.sleep();
  }
  */

  // Service to get material
  osrf_gear::GetMaterialLocations materialLocationsType;
  // Create the service client.
  ros::ServiceClient materialLocations = n.serviceClient<osrf_gear::GetMaterialLocations>("/ariac/material_locations");

  

  /*
  // Add height to the goal pose.
  goal_pose.pose.position.z += 0.10; // 10 cm above the part
  // Tell the end effector to rotate 90 degrees around the y-axis (in quaternions...).
  goal_pose.pose.orientation.w = 0.707;
  goal_pose.pose.orientation.x = 0.0;
  goal_pose.pose.orientation.y = 0.707;
  goal_pose.pose.orientation.z = 0.0;
  */

// Publisher for follow_joint_trajectory
 ros::Publisher follow_joint_trajectory = n.advertise<trajectory_msgs::JointTrajectory>("ariac/arm/command", 1000);


  int count = 0;
  ros::AsyncSpinner spinner(1); // Use 1 thread
  spinner.start(); // A spinner makes calling ros::spin() unnecessary.
  while (ros::ok()) {

    if(!orders_vector.empty()){
      // Get the type of the part wanted
      std::string type = getOrderType();
      
      // Get the location of the wanted part
      std::string unit = getPartLocation(type, materialLocations);

      // Get pose of part in camera's frame
      geometry_msgs::Pose pose = getCameraPose(type, unit);

      // Get pose of part in robot's frame
      geometry_msgs::TransformStamped tfStamped;
      try {
        tfStamped = tfBuffer.lookupTransform("arm1_base_link", "logical_camera_" + unit + "_frame",
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
      ROS_INFO_THROTTLE(5, "Pose of object in robot's reference frame (w,x,y,z),(X,Y,Z): (%f,%f,%f,%f),(%f,%f,%f)", \
      goal_pose.pose.orientation.w, goal_pose.pose.orientation.x, goal_pose.pose.orientation.y, goal_pose.pose.orientation.z, \
      goal_pose.pose.position.x, goal_pose.pose.position.y, goal_pose.pose.position.z);



      // Instantiate variables for use with the kinematic system.
      double T_pose[4][4], T_des[4][4];
      double q_pose[6], q_des[8][6];

      // Where is the end effector given the joint angles.
      // joint_states.position[0] is the linear_arm_actuator_joint
      q_pose[0] = joint_states.position[1];
      q_pose[1] = joint_states.position[2];
      q_pose[2] = joint_states.position[3];
      q_pose[3] = joint_states.position[4];
      q_pose[4] = joint_states.position[5];
      q_pose[5] = joint_states.position[6];

      ROS_INFO_THROTTLE(10, "Current Joint States: [%f, %f, %f, %f, %f, %f, %f] Radians",\
      joint_states.position[0],\
      joint_states.position[1],\
      joint_states.position[2],\
      joint_states.position[3],\
      joint_states.position[4],\
      joint_states.position[5],\
      joint_states.position[6]);
      

      // Desired pose of the end effector wrt the base_link.
      T_des[0][3] = goal_pose.pose.position.x;
      T_des[1][3] = goal_pose.pose.position.y;
      T_des[2][3] = goal_pose.pose.position.z + 0.3; // above part
      T_des[3][3] = 1.0;
      
      // The orientation of the end effector so that the end effector is down.
      T_des[0][0] = 0.0; T_des[0][1] = -1.0; T_des[0][2] = 0.0;
      T_des[1][0] = 0.0; T_des[1][1] = 0.0; T_des[1][2] = 1.0;
      T_des[2][0] = -1.0; T_des[2][1] = 0.0; T_des[2][2] = 0.0;
      T_des[3][0] = 0.0; T_des[3][1] = 0.0; T_des[3][2] = 0.0;

      int num_sols = ur_kinematics::inverse((double *)&T_des, (double *)&q_des);

      // Declare a variable for generating and publishing a trajectory.
      trajectory_msgs::JointTrajectory joint_trajectory;

      // Fill out the joint trajectory header.
      // Each joint trajectory should have an non-monotonically increasing sequence number.
      joint_trajectory.header.seq = count++;
      joint_trajectory.header.stamp = ros::Time::now(); // When was this message created.
      joint_trajectory.header.frame_id = "/world"; // Frame in which this is specified.


      // Set the names of the joints being used. All must be present.
      joint_trajectory.joint_names.clear();
      joint_trajectory.joint_names.push_back("linear_arm_actuator_joint");
      joint_trajectory.joint_names.push_back("shoulder_pan_joint");
      joint_trajectory.joint_names.push_back("shoulder_lift_joint");
      joint_trajectory.joint_names.push_back("elbow_joint");
      joint_trajectory.joint_names.push_back("wrist_1_joint");
      joint_trajectory.joint_names.push_back("wrist_2_joint");
      joint_trajectory.joint_names.push_back("wrist_3_joint");

      // Set a start and end point.
      joint_trajectory.points.resize(2);
      // Set the start point to the current position of the joints from joint_states.
      joint_trajectory.points[0].positions.resize(joint_trajectory.joint_names.size());
      for (int indy = 0; indy < joint_trajectory.joint_names.size(); indy++) {
        for (int indz = 0; indz < joint_states.name.size(); indz++) {
          if (joint_trajectory.joint_names[indy] == joint_states.name[indz]) {
            joint_trajectory.points[0].positions[indy] = joint_states.position[indz];
            break;
          }
        }
      }

      // When to start (immediately upon receipt).
      joint_trajectory.points[0].time_from_start = ros::Duration(0.0);

      // Must select which of the num_sols solutions to use. Just start with the first.
      int q_des_indx = 0;

      // Set the end point for the movement
      joint_trajectory.points[1].positions.resize(joint_trajectory.joint_names.size());


      // Set the linear_arm_actuator_joint from joint_states as it is not part of the inverse kinematics solution.
      joint_trajectory.points[1].positions[0] = joint_states.position[1];
      for (int indy = 0; indy < 6; indy++) {
        joint_trajectory.points[1].positions[indy + 1] = q_des[q_des_indx][indy];
      }

      // How long to take for the movement.
      joint_trajectory.points[1].time_from_start = ros::Duration(1.0);
      
      follow_joint_trajectory.publish(joint_trajectory);




      // The actuators are commanded in an odd order, enter the joint positions in the correct positions
      /*
      ik_service::PoseIK ik_pose;
      //geometry_msgs::Pose part_pose;
      //part_pose.position.x = 0.5;
      //ik_pose.request.part_pose = part_pose;
      ik_pose.request.part_pose = goal_pose.pose;

      
      if (ik_client.call(ik_pose)) {
        ROS_INFO_THROTTLE(10,"Call to ik_service returned [%i] solutions", ik_pose.response.num_sols);
        for (int sol = 0; sol < ik_pose.response.num_sols; sol ++){
            ROS_INFO_THROTTLE(10, "Solution %i: [%f, %f, %f, %f, %f, %f] radians", sol+1, \
            ik_pose.response.joint_solutions[sol].joint_angles[0], \
            ik_pose.response.joint_solutions[sol].joint_angles[1], \
            ik_pose.response.joint_solutions[sol].joint_angles[2], \
            ik_pose.response.joint_solutions[sol].joint_angles[3], \
            ik_pose.response.joint_solutions[sol].joint_angles[4], \
            ik_pose.response.joint_solutions[sol].joint_angles[5]);
        }
      }
      else {
        ROS_ERROR("Failed to call service ik_service");
        return 1;
      }
      */

    }

    

    loop_rate.sleep();
  }
  return 0;
}