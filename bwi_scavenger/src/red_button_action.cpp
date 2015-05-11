#include <ros/ros.h>
#include <signal.h>
#include <iostream>
#include <vector>
#include <math.h>
#include <cstdlib>
#include <std_msgs/String.h>

//actions
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>

#include <bwi_scavenger/RedButtonAction.h>

#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>

using namespace std;

bool button_cloud_received = false;
bool plane_cloud_received = false;
bool moved_to_plane = false;

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

/* define what kind of point clouds we're using */
typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

// clouds //
PointCloudT::Ptr button_cloud (new PointCloudT);

vector<int> nn_indices (1);
vector<float> nn_dists (1);

float x, y, z;
float tf_x, tf_y, tf_z;

void button_cloud_callback (const sensor_msgs::PointCloud2ConstPtr& input) {
  pcl::fromROSMsg (*input, *button_cloud);
  button_cloud_received = true;
}

void plane_cloud_callback (const sensor_msgs::PointCloud2ConstPtr& input) {
  pcl::fromROSMsg (*input, *plane_cloud);
  plane_cloud_received = true;
}

void distanceToButtonPlane () {

  pcl::KdTree<PointT>::Ptr tree_kd (new pcl::KdTreeFLANN<PointT>);
  tree_kd->setInputCloud(plane_cloud);
  tree_kd->nearestKSearch(PointT(0, 0, 0), 1, nn_indices, nn_dists);

  x = cloud->points[nn_indices[0]].x;
  y = cloud->points[nn_indices[0]].y;
  z = cloud->points[nn_indices[0]].z;

  ROS_INFO("The closest point of (0, 0, 0) is: (%f, %f, %f)", cloud->points[nn_indices[0]].x, cloud->points[nn_indices[0]].y, cloud->points[nn_indices[0]].z); 
}

void transformToBase () {

  // x forward, y left, z up
  // optical frame
  // z forward, x right, y down

  tf::TransformListener listener;

  tf::Vector3 vector(z, x, y);
      
  tf::StampedTransform transform;

  try{
    listener.waitForTransform("base_footprint", "camera_depth_frame", ros::Time::now(), ros::Duration(3.0));
    listener.lookupTransform("base_footprint", "camera_depth_frame", ros::Time::now(), transform);
  }
  catch (tf::TransformException &ex) {
    ROS_ERROR("%s",ex.what());
    ROS_WARN("Base to depth camera transform unavailable %s", ex.what());
    ros::Duration(1.0).sleep();
    continue;
  }
      
  // Transform from depth frame to base_footprint
  tf::Vector3 transform_vector = transform * vector;

  tf_x = transform_vector[0];
  tf_y = transform_vector[1];
  tf_z = transform_vector[2];

}

void moveToPlane () {

  MoveBaseClient ac("move_base", true);

  while(!ac.waitForServer(ros::Duration(5.0))) {
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  move_base_msgs::MoveBaseGoal goal;

  //we'll send a goal to the robot to move 1 meter forward
  goal.target_pose.header.frame_id = "base_footprint";
  goal.target_pose.header.stamp = ros::Time::now();

  goal.target_pose.pose.position.x = tf_x;
  goal.target_pose.pose.position.y = tf_y;
  goal.target_pose.pose.orientation.w = 1.0;

  ROS_INFO("Sending goal");

  ac.sendGoal(goal);

  ac.waitForResult();

  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO("The base moved to the table");
  else
    ROS_INFO("The base failed to move for some reason");

  moved_to_plane = true;

}

void waitForCloud() {
	ros::Rate r(10);
	while (!button_cloud_received && !plane_cloud_received){
		r.sleep();
		ros::spinOnce();
	}
}

bool approach_red_button (bwi_scavenger::RedButtonAction::Request &req, 
  bwi_scavenger::RedButtonAction::Response &res){

  //Step 1: listen to the clouds
  waitForCloud();

  //Step 2: find closest point pf plane
  distanceToButtonPlane();
  
  //Step 3: transform to base
  transformToBase();

  //Step 4: move to plane
  moveToPlane();

  //Step 5: call push button node
  if(moved_to_plane = true;)
    system("rosrun mimic_motion push_button_demo");
  else 
    return false;

  return true;
}

int main(int argc, char **argv) {
  // Intialize ROS with this node name
  ros::init(argc, argv, "red_button_action");

  ros::NodeHandle nh;
  
  //create subscriber for button cloud
  ros::Subscriber sub_button = nh.subscribe("/red_button_server/button_cloud", 1, button_cloud_callback);
  ros::Subscriber sub_plane = nh.subscribe("/red_button_server/plane_cloud", 1, plane_cloud_callback);

  ros::ServiceServer service = nh.advertiseService("red_button_approach", approach_red_button);

  return 0;
}


