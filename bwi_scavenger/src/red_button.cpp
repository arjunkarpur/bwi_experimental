#include <signal.h>
#include <vector>
#include <string>
#include <sys/stat.h>
#include <ros/ros.h>
#include <ros/package.h>

#include <sensor_msgs/PointCloud2.h>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <tf/transform_listener.h>
#include <tf/tf.h>

#include <pcl_ros/impl/transforms.hpp>

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

#include <pcl/filters/crop_box.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>

#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

#include <pcl/kdtree/kdtree.h>

#include <bwi_scavenger/RedButton.h>

/* define what kind of point clouds we're using */
typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

// Mutex: //
boost::mutex cloud_mutex;

// flags //
bool red_button_detected;
bool new_cloud_available_flag = false;

// clouds //
PointCloudT::Ptr cloud (new PointCloudT);
PointCloudT::Ptr cloud_plane (new PointCloudT);
PointCloudT::Ptr cloud_blobs (new PointCloudT);
PointCloudT::Ptr empty_cloud (new PointCloudT);

// vectors //
std::vector<PointCloudT::Ptr > clusters;
std::vector<PointCloudT::Ptr > clusters_on_plane;

sensor_msgs::PointCloud2 button_cloud_ros;
sensor_msgs::PointCloud2 plane_cloud_ros;

ros::Publisher button_cloud_pub;
ros::Publisher plane_cloud_pub;
ros::Publisher button_pose_pub;

double computeAvgRedValue(PointCloudT::Ptr in){
    double total_red = 0;

    for (unsigned int i = 0; i < in->points.size(); i++){
        total_red += in->points.at(i).r;
    }

    total_red /= (in->points.size());
    return total_red;
}

int countRedVoxels(PointCloudT::Ptr in) {
    int total_red = 0;

    for (int i = 0; i < in->points.size(); i++) {
        unsigned int r, g, b;
        r = in->points[i].r;
        g = in->points[i].g;
        b = in->points[i].b;
        // Look for mostly red values points
        // These values are hand-picked using lab lighting samples
        if (r > 100 && (g + b) < 50) {
            total_red++;
        }
    }

    return total_red;
}


void computeClusters(PointCloudT::Ptr in, double tolerance){
    pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>);
    tree->setInputCloud (in);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<PointT> ec;
    ec.setClusterTolerance (tolerance); // 2cm
    ec.setMinClusterSize (50);
    ec.setMaxClusterSize (25000);
    ec.setSearchMethod (tree);
    ec.setInputCloud (in);
    ec.extract (cluster_indices);

    clusters.clear();

    int j = 0;
    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
    {
        PointCloudT::Ptr cloud_cluster (new PointCloudT);
        for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); pit++)
            cloud_cluster->points.push_back (in->points[*pit]); //*
        cloud_cluster->width = cloud_cluster->points.size ();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;

        clusters.push_back(cloud_cluster);
    }
}

void cloud_callback (const sensor_msgs::PointCloud2ConstPtr& input)
{

    ROS_INFO("Cloud callback called");

    cloud_mutex.lock ();

    //convert to PCL format
    pcl::fromROSMsg (*input, *cloud);

    //ROS_INFO("Converted input to PCL format");

    cloud_mutex.unlock ();

    if ( cloud_mutex.try_lock () )
    {
        //Step 1: z-filter and voxel filter

        // Create the filtering object
        pcl::PassThrough<PointT> pass;
        pass.setInputCloud (cloud);
        pass.setFilterFieldName ("z");
        pass.setFilterLimits (0.0, 1.15);
        pass.filter (*cloud);
        // Create the filtering object: downsample the dataset using a leaf size of 1cm

        pcl::VoxelGrid<PointT> vg;
        pcl::PointCloud<PointT>::Ptr cloud_filtered (new pcl::PointCloud<PointT>);
        vg.setInputCloud (cloud);
        vg.setLeafSize (0.005f, 0.005f, 0.005f);
        vg.filter (*cloud_filtered);


        // If no plane detected, do nothing
        if (cloud_filtered->points.size() == 0) {
            ROS_INFO("No plane detected");
            red_button_detected = false;
            cloud_mutex.unlock();
            return;

        }

        //ROS_INFO("After voxel grid filter: %i points",(int)cloud_filtered->points.size());

        //Step 2: plane fitting
        pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
        pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
        // Create the segmentation object
        pcl::SACSegmentation<PointT> seg;
        // Optional
        seg.setOptimizeCoefficients (true);
        // Mandatory
        seg.setModelType (pcl::SACMODEL_PLANE);
        seg.setMethodType (pcl::SAC_RANSAC);
        seg.setMaxIterations (1000);
        seg.setDistanceThreshold (0.01);

        // Create the filtering object
        pcl::ExtractIndices<PointT> extract;

        // Segment the largest planar component from the remaining cloud
        seg.setInputCloud (cloud_filtered);
        seg.segment (*inliers, *coefficients);

        // Extract the plane
        extract.setInputCloud (cloud_filtered);
        extract.setIndices (inliers);
        extract.setNegative (false);
        extract.filter (*cloud_plane);

        ROS_INFO("Cloud Plane"); 

        pcl::toROSMsg(*cloud_plane,plane_cloud_ros);
        plane_cloud_ros.header.frame_id = cloud->header.frame_id;
        plane_cloud_pub.publish(plane_cloud_ros);


        //extract everything else
        extract.setNegative (true);
        extract.filter (*cloud_blobs);

        //get the plane coefficients
        Eigen::Vector4f plane_coefficients;
        plane_coefficients(0)=coefficients->values[0];
        plane_coefficients(1)=coefficients->values[1];
        plane_coefficients(2)=coefficients->values[2];
        plane_coefficients(3)=coefficients->values[3];


        //Step 3: Eucledian Cluster Extraction
        computeClusters(cloud_blobs,0.04);
        clusters_on_plane.clear();

        // Find all clusters that are similar to target (button)
        for (unsigned int i = 0; i < clusters.size(); i++){
            Eigen::Vector4f centroid_i;
            pcl::compute3DCentroid(*clusters.at(i), centroid_i);
            pcl::PointXYZ center;
            center.x=centroid_i(0);center.y=centroid_i(1);center.z=centroid_i(2);

            double distance = pcl::pointToPlaneDistance(center, plane_coefficients);
            if (distance < 0.1 /*&& clusters.at(i).get()->points.size() >*/ ){
                clusters_on_plane.push_back(clusters.at(i));

            }
        }


        //Step 4: detect the button among the remaining clusters using red color
        int max_index = -1;
        ROS_INFO("Color detection"); 
        ROS_INFO("Clusters on plane: %i", clusters_on_plane.size());

        int max_num_red = 0;
        // Alternatively, find the blob with most red voxels
        for (unsigned int i = 0; i < clusters_on_plane.size(); i++) {
            int red_num_i = countRedVoxels(clusters_on_plane.at(i));
            ROS_INFO("Cluster %i:   %i red voxels", i, red_num_i);
            if (red_num_i > max_num_red) {
                max_num_red = red_num_i;
                max_index = i;
            }
        }

        ROS_INFO("Max Index: %i", max_index);
        if ((max_index >= 0) &&
                (clusters_on_plane.at(max_index)->points.size()) < 360 &&
                (clusters_on_plane.at(max_index)->points.size()) > 50) {

            pcl::toROSMsg(*clusters_on_plane.at(max_index),button_cloud_ros);
            button_cloud_ros.header.frame_id = cloud->header.frame_id;
            button_cloud_pub.publish(button_cloud_ros);
            red_button_detected = true;
            ROS_INFO("Red button found on plane");
            ros::Duration(5.0).sleep(); 


            /*
            Eigen::Vector4f centroid;
            pcl::compute3DCentroid(*clusters_on_plane.at(max_index), centroid);
            
            //transforms the pose into /map frame
            geometry_msgs::Pose pose_i;
            pose_i.position.x=centroid(0);
            pose_i.position.y=centroid(1);
            pose_i.position.z=centroid(2);
            pose_i.orientation = tf::createQuaternionMsgFromRollPitchYaw(0,0,-3.14/2);

            geometry_msgs::PoseStamped stampedPose;

              tf::TransformListener listener;

              stampedPose.header.frame_id = cloud->header.frame_id;
              stampedPose.header.stamp = ros::Time(0);
              stampedPose.pose = pose_i;

              geometry_msgs::PoseStamped stampOut;
              listener.waitForTransform(cloud->header.frame_id, "mico_api_origin", ros::Time(0), ros::Duration(3.0));
              listener.transformPose("mico_api_origin", stampedPose, stampOut);

              stampOut.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0,-3.14/2,0);
              button_pose_pub.publish(stampOut);*/

                    }
        } else {
            red_button_detected = false;
            pcl::toROSMsg(*empty_cloud,button_cloud_ros);
            button_cloud_ros.header.frame_id = cloud->header.frame_id;
            button_cloud_pub.publish(button_cloud_ros);
        }

        //unlock mutex
        cloud_mutex.unlock ();
    }
}

bool find_red_button (bwi_scavenger::RedButton::Request &req, 
        bwi_scavenger::RedButton::Response &res){

    ros::Rate r(10); 
    while ( !red_button_detected && ros::ok() ) {
        ROS_INFO("Ros Spin");
        ros::spinOnce();
    }

    res.found_button = true;
    ROS_INFO("Red button detected");

    return true;

}


int main (int argc, char** argv)
{
    // Initialize ROS
    ros::init (argc, argv, "red_button_server");
    ros::NodeHandle nh; 

    ROS_INFO("Red_button node started");

    // Create a ROS subscriber for the input point cloud
    //std::string camera_topic = "/camera/depth_registered/points";
    std::string camera_topic = "/nav_kinect/depth_registered/points";
    ros::Subscriber sub = nh.subscribe (camera_topic, 1, cloud_callback);

    button_cloud_pub = nh.advertise<sensor_msgs::PointCloud2>
        ("red_button_server/button_cloud", 10);
    plane_cloud_pub = nh.advertise<sensor_msgs::PointCloud2>
        ("red_button_server/plane_cloud", 10);
    button_pose_pub = nh.advertise<geometry_msgs::PoseStamped>
        ("red_button_server/button_pose", 10);

    ros::ServiceServer service = nh.advertiseService
        ("red_button_service", find_red_button);

    ros::Rate r(10);

    while (ros::ok()) {
        ros::spinOnce(); 
        r.sleep();
    }    return true;
};
