/**
*       manipulation.h
*
*       @date 14.07.2023
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
#include <behaviortree_cpp_v3/bt_factory.h>
#include <behaviortree_cpp_v3/condition_node.h>
#include <behaviortree_cpp_v3/action_node.h>
#include <fstream>
#include <sstream>
#include <stdexcept>

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
        geometry_msgs::Pose grasping_point;                                     /*! The pose of the grasping points. */
        std::vector<swot_msgs::SwotObjectPose> poses;                           /*! The poses of the multiple grasping points. */
        std::string tray;                                                       /*! The currently selected tray for object placement. */
        std::unique_ptr<URRTDE> rtde;                                           /*! The unique pointer to the URRTDE instance for robot control. */
        ros::ServiceServer service_server;                                      /*! ROS service server for handling manipulation requests. */
        ros::ServiceClient service_client_matching;                             /*! ROS service client for object matching. */
        ros::ServiceClient service_client_free;                                 /*! ROS service client for object freeing. */
        std::vector<swot_msgs::SwotManipulation2023::Request> req_array_;       /*! The request object for the SwotManipulation service. */
        std::vector<swot_msgs::SwotManipulation2023::Response> res_array_;      /*! The response object for the SwotManipulation service. */
        double gripper_speed_;                                                  /*! The speed of the gripper for object manipulation. */
        double gripper_force_;                                                  /*! The force applied by the gripper for object manipulation. */
        double jnt_vel_;                                                        /*! The velocity of the robot's joints. */
        double jnt_acc_;                                                        /*! The acceleration of the robot's joints. */
        double left_left_thresh;                                                /*! The threshold value for the left-left condition. */
        double left_thresh;                                                     /*! The threshold value for the left condition. */
        double right_thresh;                                                    /*! The threshold value for the right condition. */
        double right_right_thresh;                                              /*! The threshold value for the right-right condition. */
        array6d target_position;                                                /*! The target position array for robot movement planning. */
        int count;

        int ws_height;
        std::string ws_name;
        std::string ws_type;
        std::string obj_name;
        double obj_mani_height; 
        array4d ws_dim; 
        std::string workspace_match_or_free;
        std::vector<std::string> task_track;                                 /*! Possible values for this vector are FOUND, NOTFOUND, FULLFILLED, NOTFULLFILLED, or UNKNOWN. */         

    public:  
        // Constructor
        Manipulation();

        // Destructor
        ~Manipulation();
        
        // Member functions
        void initialize();
        void registerNodes(BT::BehaviorTreeFactory& factory, Manipulation& manipulation);
        bool callback_service_manipulation(swot_msgs::SwotManipulation::Request &req, swot_msgs::SwotManipulation::Response &res);
        void callback_wrench(const geometry_msgs::WrenchStamped &msg);
        void sendTargetPosition6d();
        void tray_top();
        
        // Setter functions 
        void set_last_pos(std::string last_pose);
        void set_grasping_area(std::string grasping_area);
        void set_collision_detected(bool collision);
        void set_collision_activated(bool collision);
        void set_grasping_point(geometry_msgs::Pose grasping);
        void set_tray(std::string tray);
        void set_object_in_trays(std::string, int);
        void reset_object_in_trays(int);
        void set_response_status(const std::string& status, int index);
        void setTargetPosition6d(std::string target);
        void set_target(array6d);
        void set_count(int);
        void increment_count();

        void get_mani_height(std::string name_of_the_object);
        void get_worksapce_dimension_matching();
        void set_workspace_match_or_free(std::string);
        void set_grasping_point(int index, geometry_msgs::Pose grasping);

        // Getter functions
        std::string get_last_pos() const;
        std::string get_grasping_area() const;
        bool get_collision_detected() const;
        bool get_collision_activated() const;
        ros::NodeHandle get_nh();
        geometry_msgs::Pose get_grasping_point() const;
        std::string get_object_in_tray(int) const;
        std::string get_tray() const;
        ros::ServiceClient get_service_client_matching() const;
        ros::ServiceClient get_service_client_free() const; 
        const std::vector<swot_msgs::SwotManipulation2023::Request>& get_request_vector() const;
        const swot_msgs::SwotManipulation2023::Request& get_request(int index) const;
        const swot_msgs::SwotManipulation2023::Response& get_response(int index) const;
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

        int get_ws_height() const;
        std::string get_ws_name() const;
        std::string get_ws_type() const;
        std::string get_obj_name() const;
        double get_obj_mani_height() const;
        array4d get_ws_dim() const;
        std::string get_workspace_match_or_free() const;
        geometry_msgs::Pose get_grasping_point_of_index(int index) const;
        std::vector<std::string>& getTaskTrack();

        std::vector<std::string> objects_in_trays;      /*! The list of objects currently placed in trays. */
        array6d array_tray1_top = {0.027344752103090286, -1.3828709882548829, 0.7994797865497034,  -0.9756995004466553, -1.5633075873004358, -3.091755453740255};
        array6d array_tray2_top = {-0.2560957113849085, -1.5176499386182805, 0.9604175726519983, -1.0044456881335755, -1.5703113714801233, -3.387533966694967};
        array6d array_tray3_top = {-0.5458563009845179, -1.5639600318721314, 1.020019833241598,  -1.0378867548755188, -1.5619009176837366, -3.6705244223224085};
        array6d array_tray1_load = {0.064320102334023, -1.53433151290331, 1.48070460954775, -1.51407157376919, -1.59717017809023, -3.07259160677065};
        array6d array_tray2_load = {-0.255173508320944, -1.6467939815917, 1.58283216158022, -1.48665781438861, -1.55522424379458, -3.37798530260195};
        array6d array_tray3_load = {-0.530647579823629, -1.6887427769103, 1.65178472200503, -1.53478486955676, -1.56944162050356, -3.6110408941852};
        array6d array_scan_mid = {2.40435886383057, -1.83808960537099, 0.975212875996725, -0.674065129165985, -1.63826924959292, -3.8627772966968};

};
