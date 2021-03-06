// rosrun bwi_scavenger red_button
// rosrun bwi_scavenger red_button_action
// rosservice call "/red_button_service" {}
// rosservice call "/red_button_approach" {}
// roslaunch bwi_launch segbot_v1.launch --screen
// rosrun segbot_bringup teleop_twist_keyboard 

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

// PCL specific includes
#include <pcl/conversions.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/console/parse.h>
#include <pcl/point_types.h>
#include <pcl/io/openni_grabber.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/common/time.h>
#include <pcl/common/common.h>

#include <pcl/kdtree/kdtree.h>
#include <pcl/kdtree/kdtree_flann.h>

using namespace std;

bool plane_cloud_received = false;
bool moved_to_plane = false;

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

/* define what kind of point clouds we're using */
typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

// clouds //
PointCloudT::Ptr plane_cloud (new PointCloudT);

vector<int> nn_indices (1);
vector<float> nn_dists (1);

//ros::Publisher set_pose;

float x, y, z;
float tf_x, tf_y, tf_z;

void plane_cloud_callback (const sensor_msgs::PointCloud2ConstPtr& input) {
    pcl::fromROSMsg (*input, *plane_cloud);
    plane_cloud_received = true;
}

void closetPointToButtonPlane () {

    pcl::KdTreeFLANN<PointT> kdtree;
    kdtree.setInputCloud(plane_cloud);
    kdtree.nearestKSearch(PointT(0, 0, 0), 1, nn_indices, nn_dists);

    x = plane_cloud->points[nn_indices[0]].x;
    y = plane_cloud->points[nn_indices[0]].y;
    z = plane_cloud->points[nn_indices[0]].z;

    ROS_INFO("The closest point of (0, 0, 0) is: (%f, %f, %f)", plane_cloud->points[nn_indices[0]].x, plane_cloud->points[nn_indices[0]].y, plane_cloud->points[nn_indices[0]].z); 
}

void transformToMap () {

    // x forward, y left, z up
    // optical frame
    // z forward, x right, y down

    tf::TransformListener listener;

    tf::Vector3 vector(z, x, y);

    tf::StampedTransform transform;

    //listener.waitForTransform("level_mux/map", "camera_depth_frame", ros::Time::now(), ros::Duration(3.0));
    //listener.lookupTransform("level_mux/map", "camera_depth_frame", ros::Time::now(), transform);
    listener.waitForTransform("level_mux/map", plane_cloud->header.frame_id, ros::Time(0), ros::Duration(3.0));
    listener.lookupTransform("level_mux/map", plane_cloud->header.frame_id, ros::Time(0), transform);

    // Transform from depth frame to base_footprint
    tf::Vector3 transform_vector = transform * vector;

    ROS_INFO("Transformed");

    tf_x = transform_vector[0];
    tf_y = transform_vector[1];
    tf_z = transform_vector[2];
    
    ROS_INFO("The transformed point is : (%f, %f, %f)", tf_x, tf_y, tf_z); 

}

void moveToPlane () {
    ROS_INFO("Moving to plane");
/*
    geometry_msgs::PoseStamped stampedPose; 

    stampedPose.header.frame_id = "level_mux/map";
    stampedPose.header.stamp = ros::Time(0);

    stampedPose.pose.position.x = tf_x;
    stampedPose.pose.position.y = tf_y;
    stampedPose.pose.orientation.w = 1.0;

    set_pose.publish(stampedPose);*/

    
    MoveBaseClient ac("move_base_interruptable", true);

    while (!ac.waitForServer(ros::Duration(5.0))) {
    ROS_INFO("Waiting for move_base action server to come up");
    }

    move_base_msgs::MoveBaseGoal goal;

    goal.target_pose.header.frame_id = "level_mux/map";
    goal.target_pose.header.stamp = ros::Time(0);

    goal.target_pose.pose.position.x = tf_x;
    goal.target_pose.pose.position.y = tf_y;
    goal.target_pose.pose.orientation.w = 1.0;

    ROS_INFO("Sending goal");

    ac.sendGoal(goal);

    ac.waitForResult();

    if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
      ROS_INFO("The base moved to the table");
      moved_to_plane = true;
    } else
      ROS_WARN("The base failed to move to table");
     
}

void waitForCloud() {
    ros::Rate r(10);
    while (!plane_cloud_received) {
        r.sleep();
        ros::spinOnce();
    }
}

bool approach_red_button (bwi_scavenger::RedButtonAction::Request &req, 
        bwi_scavenger::RedButtonAction::Response &res){

    //Step 1: listen to the clouds
    waitForCloud();

    //Step 2: find closest point of plane
    closetPointToButtonPlane();

    //Step 3: transform to base
    transformToMap();

    //Step 4: move to plane
    moveToPlane();

    //Step 5: call push button node
    if(moved_to_plane)
        ROS_INFO("Calling node to press button");
        //system("rosrun mimic_motion push_button_demo");
    else 
        return false;

    return true;
}

int main(int argc, char **argv) {
    // Intialize ROS with this node name
    ros::init(argc, argv, "red_button_action");

    ros::NodeHandle nh;

    //create subscriber for button cloud
    ros::Subscriber sub_plane = nh.subscribe("/red_button_server/plane_cloud", 1, plane_cloud_callback);

    ros::ServiceServer service = nh.advertiseService("red_button_approach", approach_red_button);

    //set_pose = nh.advertise<geometry_msgs::
    //    PoseStamped>("move_base_interruptable_simple/goal", 10);

    while (ros::ok()) {
        ros::spinOnce();
    }

    return 0;
}


