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
#include <swot_msgs/SwotObjectMatching2023.h>
#include <swot_msgs/SwotFreeSpot.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/WrenchStamped.h>
#include <tf2_ros/transform_listener.h>
#include <algorithm>
#include <vector>
#include <memory>
#include <iostream>
#include <functional>
#include <behaviortree_cpp_v3/bt_factory.h>
#include <behaviortree_cpp_v3/condition_node.h>
#include <behaviortree_cpp_v3/action_node.h>
#include <geometry_msgs/TransformStamped.h


// Alias for an array of 6 doubles
typedef boost::array<double, 6> array6d; 

// Alias for an array of 7 doubles
typedef boost::array<double, 7> array7d; 

/**
*       @struct ConditionAction
*       @brief Represents a condition-action pair for handling RoboCup manipulation tasks.
*       @details This struct combines a condition function and an action function. The condition function
*           determines whether the specified condition is met, and the action function is executed
*           if the condition is true.
*/

struct ConditionAction;

/**
*       @struct Tray
*       @brief Represents a tray used for RoboCup manipulation tasks.
*       @details This struct holds information about the top pose, load pose, save position, and tray object
*           associated with a specific tray.
*/

struct Tray;

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
        std::string xml_file;                           /*! The path to the XML file containing manipulation configurations for the behavior tree to be structured. */
        std::string last_pos;                           /*! The last known position of the manipulator. */
        std::string grasping_area;                      /*! The designated area for grasping objects. */
        const double wrench_limit;                      /*! The maximum allowable wrench value for collision detection. */
        bool collision_detected;                        /*! Flag indicating if a collision has been detected. */
        bool collision_activated;                       /*! Flag indicating if collision detection is activated. */
        ros::NodeHandle nh_;                            /*! The ROS NodeHandle for communication. */
        ros::Subscriber sub_wrench;                     /*! ROS subscriber for wrench data. */
        geometry_msgs::Pose grasping_point;             /*! The pose of the grasping point. */
        std::string tray;                               /*! The currently selected tray for object placement. */
        std::unique_ptr<URRTDE> rtde;                   /*! The unique pointer to the URRTDE instance for robot control. */
        ros::ServiceServer service_server;              /*! ROS service server for handling manipulation requests. */
        ros::ServiceClient service_client_matching;     /*! ROS service client for object matching. */
        ros::ServiceClient service_client_free;         /*! ROS service client for object freeing. */
        swot_msgs::SwotManipulation::Request req_;      /*! The request object for the SwotManipulation service. */
        swot_msgs::SwotManipulation::Response res_;     /*! The response object for the SwotManipulation service. */
        double gripper_speed_;                          /*! The speed of the gripper for object manipulation. */
        double gripper_force_;                          /*! The force applied by the gripper for object manipulation. */
        double jnt_vel_;                                /*! The velocity of the robot's joints. */
        double jnt_acc_;                                /*! The acceleration of the robot's joints. */
        double left_left_thresh;                        /*! The threshold value for the left-left condition. */
        double left_thresh;                             /*! The threshold value for the left condition. */
        double right_thresh;                            /*! The threshold value for the right condition. */
        double right_right_thresh;                      /*! The threshold value for the right-right condition. */
        array6d target_position;                        /*! The target position array for robot movement planning. */

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
        void set_response_status(const std::string& status);
        void setTargetPosition6d(std::string target);

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
        swot_msgs::SwotManipulation::Request get_request() const;
        swot_msgs::SwotManipulation::Response get_response() const;
        double get_gripper_speed_() const;                          
        double get_gripper_force_() const;                         
        double get_jnt_vel_() const;                               
        double get_jnt_acc_() const;                               
        double get_left_left_thresh() const;                        
        double get_left_thresh() const;                             
        double get_right_thresh() const;                            
        double get_right_right_thresh() const;                     
        const std::unique_ptr<URRTDE>& MyClass::getRTDE() const;

        std::vector<std::string> objects_in_trays;      /*! The list of objects currently placed in trays. */

        // Array positions which can also be found in the csv file. 
        // This arrays will be substituted by the data.csv file where all this positions are also found.
        array6d array_scan_mid;
        array6d array_scan_left;
        array6d array_scan_left_yolo;
        array6d array_scan_right_yolo;
        array6d array_scan_right;
        array6d array_pick_mid;
        array6d array_pick_left_left;
        array6d array_pick_left;
        array6d array_pick_right;
        array6d array_pick_right_right;
        array6d array_rotate1;
        array6d array_rotate2;
        array6d array_rotate3;
        array6d array_tray1_top;
        array6d array_tray2_top;
        array6d array_tray3_top;
        array6d array_tray1_load;
        array6d array_tray2_load;
        array6d array_tray3_load;
        array6d array_drive;
        array6d free_backup_1;
        array6d free_backup_2;
        array6d free_SH_1;
};