/**
*       manipulation.h
*
*       @date 19.10.2023
*       @author Joel Santos
*/

#pragma once 

#include "ros/ros.h"
#include <ros/package.h>
#include <swot_ur/ur_rtde.h>
#include <swot_msgs/SwotManipulation.h>                     
#include <swot_msgs/SwotManipulation2023.h>                 
#include <swot_msgs/SwotObjectMatching.h>
#include <swot_msgs/SwotFreeSpot.h>
#include <swot_msgs/SwotObjectPose.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/WrenchStamped.h>
#include <tf2_ros/transform_listener.h>
#include <algorithm>
#include <vector>
#include <memory>
#include <iostream>
#include <functional>
#include <geometry_msgs/TransformStamped.h>
#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/condition_node.h>
#include <behaviortree_cpp/action_node.h>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <utility>
#include <string>
#include <iterator>

// Alias for an array of 6 doubles
typedef boost::array<double, 6> array6d; 

// Alias for an array of 7 doubles
typedef boost::array<double, 7> array7d; 

// Alias for an array of 4 doubles
typedef boost::array<double, 4> array4d;

/**
*       @struct ConditionAction
*       @brief Represents a condition-action pair for handling RoboCup manipulation tasks.
*       @details This struct combines a condition function and an action function. The condition function
*           determines whether the specified condition is met, and the action function is executed
*           if the condition is true.
*/

struct ConditionAction {
    std::function<bool()> condition;        /**< The condition function. */
    std::function<void()> action;           /**< The action function. */
};

/**
*       @struct Tray
*       @brief Represents a tray used for RoboCup manipulation tasks.
*       @details This struct holds information about the top pose, load pose, save position, and tray object
*           associated with a specific tray.
*/

struct Tray {
    array6d topPose;                        /**< The top pose of the tray. */
    array6d loadPose;                       /**< The load pose of the tray. */
    std::string savePosition;               /**< The save position of the tray. */
    std::string& trayObject;                /**< The object placed on the tray. */

    Tray& operator=(const Tray& other)
    {
        //Perform member-wise assignment, excluding the reference member
        trayObject = other.trayObject;
        return *this;
    }
};

/**
*       @struct ManipulationHeight
*       @brief Custom struct used to represent the data obtained from the manipulation_height.csv file.
*       @details This struct combines a vector of string object and a vector of doubles. The string object (object_name) represents 
*           the name of the object which is to be picked, and the double object (height) represents the
*           height at which the object is to be picked.
*/

struct ManipulationHeight {
    std::vector<std::string> object_names;        
    std::vector<double> manipulation_heights;                  
};

/**
*       @struct Positions
*       @brief Custom Struct used to represent the data obtained from the positions.csv file.
*       @details This struct combines a vector of strings and positions of type array4d. The position_names
*           represent the posible positions the robot can take before obtaining the coordinate to which an object 
*           is to be picked or dropped, and the positions object represent the actual coordinates to which the robot will move. 
*/

struct Positions {
    std::vector<std::string> position_names;
    std::vector<array4d> positions;
};

/**
*       @struct WorkSpaceDimensionsFree
*       @brief Custom Struct used to represent the data obtained from the workspace_dimensions_free.csv file.
*       @details This struct combines a vector of strings and a vector which contains array elements of type double.
*           The workspace_names represent the workspace number to which the robot can navigate. The object workspace_dimensions
*           represent the dimensions every free workspace has.
*/

struct WorkSpaceDimensionsFree {
    std::vector<std::string> workspace_number;
    std::vector<std::array<double, 5> > workspace_dimensions;
};

/**
*       @struct WorkSpaceDimensionsMatching
*       @brief Custom Struct used to represent the data obtained from the workspace_dimensions_matching.csv file.
*       @details This struct combines a vector of strings and a vector which contains array elements of type double.
*           The workspace_names represent the workspace number to which the robot can navigate. The object workspace_dimensions
*           represent the dimensions every free workspace has.
*/

struct WorkSpaceDimensionsMatching {
    std::vector<std::string> workspace_number;
    std::vector<std::array<double, 5> > workspace_dimensions;
};

/**
*
*       @class Manipulation
*       @brief Main class to handle the RoboCup manipulation tasks
*       @details This class provides the functionality to handle manipulation tasks in the RoboCup environment.
*           It encapsulates various operations such as object grasping, object placement, collision detection, and robot control.
*           One can interact with this class to perform manipulation actions and control the robot's behavior.
*/

class Manipulation
{    
    private:
        std::string xml_file;                                                   /*! The path to the XML file containing manipulation configurations for the behavior tree to be structured. */
        std::string last_pos;                                                   /*! The last known position of the manipulator. */
        std::string grasping_area;                                              /*! The designated area for grasping objects. */
        const double wrench_limit;                                              /*! The maximum allowable wrench value for collision detection. */
        bool collision_detected;                                                /*! Flag indicating if a collision has been detected. */
        bool collision_activated;                                               /*! Flag indicating if collision detection is activated. */
        ros::NodeHandle nh_;                                                    /*! The ROS NodeHandle for communication. */
        ros::Subscriber sub_wrench;                                             /*! ROS subscriber for wrench data. */
        std::string tray;                                                       /*! The currently selected tray for object placement. */
        std::unique_ptr<URRTDE> rtde;                                           /*! The unique pointer to the URRTDE instance for robot control. */
        ros::ServiceServer service_server;                                      /*! ROS service server for handling manipulation requests. */
        ros::ServiceClient service_client_matching;                             /*! ROS service client for object matching. */
        ros::ServiceClient service_client_free;                                 /*! ROS service client for object freeing. */
        std::vector<swot_msgs::SwotManipulations::Request> req_array_;          /*! The request object for the SwotManipulation service. */
        swot_msgs::SwotManipulations::Response res_;                            /*! The response object for the SwotManipulation service. */
        double gripper_speed_;                                                  /*! The speed of the gripper for object manipulation. */
        double gripper_force_;                                                  /*! The force applied by the gripper for object manipulation. */
        double jnt_vel_;                                                        /*! The velocity of the robot's joints. */
        double jnt_acc_;                                                        /*! The acceleration of the robot's joints. */
        double left_left_thresh;                                                /*! The threshold value for the left-left condition. */
        double left_thresh;                                                     /*! The threshold value for the left condition. */
        double right_thresh;                                                    /*! The threshold value for the right condition. */
        double right_right_thresh;                                              /*! The threshold value for the right-right condition. */
        array6d target_position;                                                /*! The target position array for robot movement planning. */
        int count;                                                              /*! Count can either be 0, 1 or 2. */

        int ws_height;
        std::string ws_name;
        std::string ws_type;
        std::string obj_name;
        double obj_mani_height; 
        array4d ws_dim; 
        std::vector<std::string> task_track;                                             /*! Possible values for this vector are FOUND, NOTFOUND, FULLFILLED, NOTFULLFILLED, or UNKNOWN. */ 
        std::vector<std::pair<std::string, swot_msgs::SwotObjectPose>> pick_tracker;     /*! The initial string starts with a number which is the number of task in the current workspace, then 0 or 1 depending if the task was completed and then tha name of the object, for example, 20M20. */ 
        std::vector<std::pair<std::string, swot_msgs::SwotObjectPose>> place_tracker;    /*! The initial string starts with a number which is the number of task in the current workspace, then 0 or 1 depending if the task was completed and then tha name of the object, for example, 20M20. */
        int task_count;

        ManipulationHeight manipulation_height_object;
        Positions manipulation_poses;
        WorkSpaceDimensionsFree workspace_dimensions_free_object;
        WorkSpaceDimensionsMatching workspace_dimensions_matching_object;

    public:  
        // Constructor
        Manipulation();

        // Destructor
        ~Manipulation();
        
        // Member functions
        void initialize();
        void registerNodes(BT::BehaviorTreeFactory& factory, Manipulation& manipulation);
        bool callback_service_manipulation(swot_msgs::SwotManipulations::Request &req, swot_msgs::SwotManipulations::Response &res);
        void callback_wrench(const geometry_msgs::WrenchStamped &msg);
        void sendTargetPosition6d();
        void tray_top();
        
        // Setter functions 
        void set_last_pos(std::string last_pose);
        void set_grasping_area(std::string grasping_area);
        void set_collision_detected(bool collision);
        void set_collision_activated(bool collision);
        void set_tray(std::string tray);
        void set_response_status(std::string status);
        void setTargetPosition6d(); 
        void set_target(array6d);
        void set_count(int);
        void increment_count();

        void get_mani_height(); 
        void get_workspace_dimension_matching(); 
        void get_workspace_dimension_free(); 

        // Getter functions
        std::string get_last_pos() const;
        std::string get_grasping_area() const;
        bool get_collision_detected() const;
        bool get_collision_activated() const;
        ros::NodeHandle get_nh();
        std::string get_tray() const;
        ros::ServiceClient get_service_client_matching() const;
        ros::ServiceClient get_service_client_free() const; 
        const std::vector<swot_msgs::SwotManipulations::Request>& get_request_vector() const;
        const swot_msgs::SwotManipulations::Request& get_request(int index) const;
        swot_msgs::SwotManipulations::Response& get_response();
        double get_gripper_speed_() const;                          
        double get_gripper_force_() const;                         
        double get_jnt_vel_() const;                               
        double get_jnt_acc_() const;                               
        double get_left_left_thresh() const;                        
        double get_left_thresh() const;                             
        double get_right_thresh() const;                            
        double get_right_right_thresh() const;                     
        const std::unique_ptr<URRTDE>& getRTDE() const;
        int get_count() const;
        int& get_ws_height();
        std::string& get_ws_name();
        std::string& get_ws_type();
        std::string& get_obj_name();
        double& get_obj_mani_height();
        array4d& get_ws_dim();
        std::vector<std::string>& getTaskTrack();
        std::string object_in_gripper;
        std::vector<std::string> objects_in_trays;      
        std::vector<std::pair<std::string, swot_msgs::SwotObjectPose>>& getPickTracker();
        std::vector<std::pair<std::string, swot_msgs::SwotObjectPose>>& getPlaceTracker();
        int& get_task_count();

        ManipulationHeight get_manipulation_height_object();
        Positions get_manipulation_poses();
        WorkSpaceDimensionsFree get_workspace_dimensions_free_object();
        WorkSpaceDimensionsMatching get_workspace_dimensions_matching_object();
};
