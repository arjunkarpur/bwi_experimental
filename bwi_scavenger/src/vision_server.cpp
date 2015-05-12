
#include "ros/ros.h"
#include "bwi_scavenger/ColorShirt.h"
#include "bwi_scavenger/ObjectDetection.h"
#include "bwi_scavenger/Whiteboard.h"
#include "bwi_scavenger/VisionTaskAction.h"
#include "bwi_scavenger/RedButton.h"
#include <actionlib/server/simple_action_server.h>


class VisionTaskAction {
    
protected:

    ros::NodeHandle nh_;

    actionlib::SimpleActionServer <bwi_scavenger::VisionTaskAction> as_;
    std::string action_name_; 

    bwi_scavenger::VisionTaskResult result_;
    bwi_scavenger::VisionTaskFeedback feedback_;

public:

    // constructor while initializing parameters
    VisionTaskAction(std::string name) : 
        as_(nh_, name, boost::bind( &VisionTaskAction::executeCB, this, _1), 
        false), action_name_(name) {

        as_.start();        
        ROS_INFO("%s: started", action_name_.c_str()); 
    }
    
    ~VisionTaskAction(void) {}

    void executeCB(const bwi_scavenger::VisionTaskGoalConstPtr &goal) {
        
        ros::Rate r(1);
        bool success = true; 

        ROS_INFO("%s: Executing, type %i of VisionTask", action_name_.c_str(),
            (int) goal->type); 

        ros::ServiceClient client_whiteboard = nh_.serviceClient 
            <bwi_scavenger::Whiteboard> ("whiteboard_service");
        bwi_scavenger::Whiteboard srv_whiteboard;

        ros::ServiceClient client_template = nh_.serviceClient
            <bwi_scavenger::ObjectDetection> ("object_detection_service");
        bwi_scavenger::ObjectDetection srv_template;

        ros::ServiceClient client_shirt = nh_.serviceClient
            <bwi_scavenger::ColorShirt> ("blue_shirt_service"); 
        bwi_scavenger::ColorShirt srv_shirt;

        ros::ServiceClient client_button = nh_.serviceClient
            <bwi_scavenger::RedButton> ("red_button_service");
        bwi_scavenger::RedButton srv_button;

        switch ( (int) goal->type) {
            
            case bwi_scavenger::VisionTaskGoal::WHITEBOARD: // whiteboard

                client_whiteboard.waitForExistence(); 
                client_whiteboard.call(srv_whiteboard); 
                result_.path_to_image = srv_whiteboard.response.path_to_image; 
                break;

            case bwi_scavenger::VisionTaskGoal::TEMPLATE: // template match
                
                client_template.waitForExistence(); 
                srv_template.request.path_to_template = goal->path_to_template;
                client_template.call(srv_template); 
                result_.path_to_image = srv_template.response.path_to_image; 
                break;

            case bwi_scavenger::VisionTaskGoal::COLORSHIRT: // color shirt

                client_shirt.waitForExistence(); 
                srv_shirt.request.color = goal->color; 
                client_shirt.call(srv_shirt); 
                result_.path_to_image = srv_shirt.response.path_to_image; 
                break;
            
            case bwi_scavenger::VisionTaskGoal::REDBUTTON:  // red button
                client_button.waitForExistence();
                client_button.call(srv_button);
                //TODO: return some kind of value to result_?
                break;

            default:

                ROS_ERROR("Error in calling scavenger hunt vision task"); 
        }

        ROS_INFO("%s: Succeeded", action_name_.c_str()); 
        as_.setSucceeded(result_); 
    }
}; 


int main(int argc, char ** argv) {
    
    ros::init(argc, argv, "scavenger_vision_server"); 

    VisionTaskAction visionTask(ros::this_node::getName());
    ros::spin();
    
    return true;    
}

