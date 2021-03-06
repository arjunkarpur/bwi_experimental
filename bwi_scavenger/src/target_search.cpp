#include <yaml-cpp/yaml.h>
#include <ros/ros.h>
#include <vector>
#include <fstream>
#include <string>
#include <stdio.h>

// msgs
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Twist.h> 

#include <std_msgs/String.h>

// path
#include <nav_msgs/Path.h>
#include <nav_msgs/GetPlan.h>

#include <tf/LinearMath/Matrix3x3.h>
#include <tf/LinearMath/Transform.h>
#include <tf/transform_datatypes.h>

#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

#include <bwi_scavenger/VisionTaskAction.h>
#include <bwi_scavenger/TargetSearch.h>
#include <actionlib/client/terminal_state.h>

#include <math.h>

const float PI = atan(1) * 4; 

// global variables

geometry_msgs::PoseWithCovarianceStamped curr_pos; 
bwi_scavenger::VisionTaskGoal *goal; 

ros::NodeHandle * nh; 

actionlib::SimpleActionClient <bwi_scavenger::VisionTaskAction> * ac; 

// callback function that saves robot's current position
void callbackCurrPos(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg) {
    
    // ROS_INFO("I heard something about my current position"); 
    curr_pos = *msg;
    
}


// compute the length of a path, nav_msgs::Path
// we used pixel as the distance unit, as we do not care the absolute values
// returns a float value
unsigned computePathLength(nav_msgs::Path * path) {
 
    float len = 0.0; 

    if (!path) {
        ROS_ERROR("Failed to compute path length as the path is empty"); 
        return 0.0; 
    }

    std::vector <geometry_msgs::PoseStamped> vec(path->poses);

    return (unsigned) vec.size(); 
}

// this function updates the belief distribution based on bayesian equation
void updateBelief(std::vector<float> *belief, bool detected, int next_goal_index) {
    
    float true_positive_rate, true_negative_rate; 

    ros::param::param <float> ("~true_positive_rate", true_positive_rate, 0.95); 
    ros::param::param <float> ("~true_negative_rate", true_negative_rate, 0.95); 
    
    std::vector <float> tmp_belief (*belief); 

    if (detected)
        tmp_belief[next_goal_index] = tmp_belief[next_goal_index] * true_positive_rate / 
            ((tmp_belief[next_goal_index] * true_positive_rate) + 
             ((1.0 - tmp_belief[next_goal_index]) * (1.0 - true_negative_rate))); 
    else
        tmp_belief[next_goal_index] = tmp_belief[next_goal_index] * (1.0 - true_positive_rate) / 
            ((tmp_belief[next_goal_index] * (1.0 - true_positive_rate)) + 
             ((1.0 - tmp_belief[next_goal_index]) * true_negative_rate)); 

    float normalizer = 0.0; 

    for (int i = 0; i < tmp_belief.size(); i++)
        normalizer += tmp_belief[i]; 

    for (int i = 0; i < tmp_belief.size(); i++)
        (*belief)[i]  = tmp_belief[i] / normalizer; 
        
}


// this function calls the robot platform and visual sensors to sense a specific
// scene. It return a boolean value identifies if the target is detected. 
bool observe() {

    ROS_INFO("%s: checking vision service availability", 
        ros::this_node::getName().c_str()); 
    ac->waitForServer(); // will wait for infinite time
    ROS_INFO("passed"); 

    // publish to /cmd_vel to stop the robot first
    ros::Publisher pub = nh->advertise <geometry_msgs::Twist> ("/cmd_vel", 100); 
    geometry_msgs::Twist vel; 
    vel.linear.x = 0;
    vel.linear.y = 0;
    vel.angular.z = 0;
    pub.publish(vel); 

    ROS_INFO("Look to the left..."); // look to the left
    vel.angular.z = 0.1;

    for (int i=0; i < 100; i++) {
        ros::spinOnce();
        pub.publish(vel); 
        ros::Duration(0.1).sleep();
    }

    ROS_INFO("Look to the right..."); // look to the right
    vel.angular.z = -0.1;

    for (int i=0; i < 200; i++) {
        ros::spinOnce();
        pub.publish(vel); 
        ros::Duration(0.1).sleep();
    }

    vel.angular.z = 0;
    for (int i=0; i < 10; i++) {
        ros::spinOnce();
        pub.publish(vel); 
        ros::Duration(0.1).sleep();
    }

    // here we assume the robot can never detects the target, so it's always
    // returning false; 
    bool ret = ac->getState() == actionlib::SimpleClientGoalState::SUCCEEDED;
    ROS_INFO("%s: observed? %d", ros::this_node::getName().c_str(), ret);
    return ret; 
}

bool  service_callback(bwi_scavenger::TargetSearch::Request &req, 
    bwi_scavenger::TargetSearch::Response &res) {

    ROS_INFO("%s: target search task received", ros::this_node::getName().c_str()); 

    goal->type = req.type;
    goal->path_to_template = req.path_to_template; 
    goal->color = req.color;

    ROS_INFO("%s: waiting for action service to start.",
        ros::this_node::getName().c_str()); 

    ac->waitForServer(); // will wait for infinite time

    ROS_INFO("%s: action server started, sending goal.",
        ros::this_node::getName().c_str()); 

    ac->sendGoal( *goal ); 
    
    std::string yaml_file_positions; 

    if (ros::param::get("~positions", yaml_file_positions))
        ROS_INFO("\nFile: %s", yaml_file_positions.c_str()); 

    std::ifstream fin(yaml_file_positions.c_str());
    YAML::Parser parser(fin);
    YAML::Node positions;
    parser.GetNextDocument(positions); 

    // belief distribution over all possible target positions
    std::vector <float> belief(positions.size(), 1.0/positions.size()); 

    // a vector that saves the distances to all possible positions
    std::vector <unsigned> distances(positions.size(), 0.0); 

    // fitness function: maximization to decide where to see
    std::vector <float> fitness(positions.size(), 0.0); 

    // service for computing a path from current position to a goal
    ros::ServiceClient client_compute_path = 
        nh->serviceClient <nav_msgs::GetPlan> ("move_base/NavfnROS/make_plan"); 
    nav_msgs::GetPlan srv; 

    // service for driving robot platform
    ros::Publisher pub_move_robot = nh->advertise <geometry_msgs::PoseStamped> 
        ("/move_base_interruptable_simple/goal", 100); 
    geometry_msgs::PoseStamped msg_goal; 

    // subscribe to the topic that reports the current position
    ros::Subscriber sub = nh->subscribe("amcl_pose", 100, callbackCurrPos); 
    ros::Duration(2.0).sleep(); 

    // get the parameter of tolerance to goal
    float tolerance; 
    if (!ros::param::get("~tolerance", tolerance)) 
        ROS_ERROR("tolerance value not set"); 

    float analyzing_cost;
    ros::param::param <float> ("~analyzing_scene_cost", analyzing_cost, 5.0); 

    bool detected = false;

    ros::Rate loop_rate(10); 

    std::stringstream ss;

    while (ros::ok()) {

        // to get the current position using the callback function
        ros::spinOnce(); 
        ROS_INFO("Compute which scene to analyze next"); 

        for (unsigned i = 0; i < positions.size(); i++) {

            // to determine the starting point
            srv.request.start.header.frame_id = "level_mux/map"; 
            srv.request.start.pose = curr_pos.pose.pose; 

            // to determine the goal point
            srv.request.goal.header.frame_id = "level_mux/map"; 
            positions[i][0] >> srv.request.goal.pose.position.x; 
            positions[i][1] >> srv.request.goal.pose.position.y; 

            // call service to compute a path to a possible positon
            client_compute_path.waitForExistence();
            if (!client_compute_path.call(srv))
                ROS_ERROR("Calling service for path computing failed"); 

            nav_msgs::Path path = srv.response.plan; 

            // compute and save the distance of that path
            distances[i] = computePathLength( & path); 

        }

        // fitness function, weighted belief probabilities
        int next_goal_index; 
        float tmp_max = -1.0; 
        float resolution;
        ros::param::param <float> ("/move_base/local_costmap/resolution", 
            resolution, 0.05); 

        for (unsigned i = 0; i < positions.size(); i++) {
            
            // exit of this program
            if (belief[i] > 0.8) {
                res.path_to_image = ac->getResult()->path_to_image; 
                return true; 
            }

            fitness[i] = belief[i] / ( (float) distances[i] * resolution + analyzing_cost); 
            
            // finds the largest fitness value and save its index to
            // next_goal_index
            if (fitness[i] >= tmp_max) {
                next_goal_index = i; 
                tmp_max = fitness[i];     
            }
        }

        // assumble a goal for analyzing the next scene of interest
        msg_goal.header.frame_id = "level_mux/map";
        positions[next_goal_index][0] >> msg_goal.pose.position.x; 
        positions[next_goal_index][1] >> msg_goal.pose.position.y; 

        float tmp_z, tmp_w;
        positions[next_goal_index][2] >> tmp_z; 
        positions[next_goal_index][3] >> tmp_w; 
        msg_goal.pose.orientation.z = tmp_z; 
        msg_goal.pose.orientation.w = tmp_w; 
        
        // we have assumbled a goal, we now publish it to the proper topic
        bool goal_achieved = false;

        ROS_INFO("Moving to the next scene for visual analyzation"); 
        pub_move_robot.publish(msg_goal); 
        int cnt = 0; 

        while (ros::ok() && !goal_achieved && ++cnt) {

            ros::spinOnce(); 

            float tmp_x = msg_goal.pose.position.x - curr_pos.pose.pose.position.x;
            float tmp_y = msg_goal.pose.position.y - curr_pos.pose.pose.position.y;

            float dis_to_goal = pow(tmp_x*tmp_x + tmp_y*tmp_y, 0.5); 

            char *dis_pt = new char[128], *tol_pt = new char[128]; 
            sprintf(dis_pt, "%f", dis_to_goal); 
            sprintf(tol_pt, "%f", tolerance); 
            ROS_INFO("target_search: dis_to_goal=%s", dis_pt); 
            ROS_INFO("target_search: tolerance=%s", tol_pt); 

            if (dis_to_goal < tolerance)  goal_achieved = true;

            // sometimes low-level motion planning gets aborted for unknown
            // reasons, so here we periodically re-send the goal
            if (cnt % 10 == 0) pub_move_robot.publish(msg_goal); 

            ros::Duration(2.0).sleep();

        }
        ROS_INFO("Arrived"); 

        detected = observe(); 

        // update belief based on observation (true or false)
        updateBelief( & belief, detected, next_goal_index); 
        
    }

}

// this is the entrance of the program
int main(int argc, char **argv) {

    ros::init(argc, argv, "target_search_server");
    nh = new ros::NodeHandle(); 

    ac = new actionlib::SimpleActionClient<bwi_scavenger::VisionTaskAction>
        ("scavenger_vision_server", true); 

    goal = new bwi_scavenger::VisionTaskGoal(); 

    ros::ServiceServer service = nh->advertiseService("target_search_service",
        service_callback); 
    ros::spin(); 

    return 0;        
}


