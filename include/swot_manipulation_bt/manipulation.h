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
#include <geometry_msgs/TransformStamped.h>
#include <behaviortree_cpp_v3/bt_factory.h>
#include <behaviortree_cpp_v3/condition_node.h>
#include <behaviortree_cpp_v3/action_node.h>
#include <fstream>

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
        int count;

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
        void set_target(array6d);
        void set_count(int);

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
        const std::unique_ptr<URRTDE>& getRTDE() const;
        int get_count() const;

        std::vector<std::string> objects_in_trays;      /*! The list of objects currently placed in trays. */

        // Array positions which can also be found in the csv file. 
        // This arrays will be substituted by the data.csv file where all this positions are also found.
        // Array definitions
        array6d array_scan_mid = {2.40435886383057, -1.83808960537099, 0.975212875996725, -0.674065129165985, -1.63826924959292, -3.8627772966968};
        array6d array_scan_left = {3.681095838546753, -1.2161747378161927, 0.6712544600116175, -1.0322970908931275, -1.663314167653219, -2.6715996901141565};
        array6d array_scan_left_yolo = {3.31208157539368, -1.67535986522817, 1.43538600603213, -1.27126656592403, -1.62322599092592, -3.0253372828113};
        array6d array_scan_right_yolo = {2.03880262374878, -1.67188944439077, 1.43529826799502, -1.32640195012603, -1.58402139345278, -4.25680452982058};
        array6d array_scan_right = {1.9794570207595825, -1.405427412395813, 0.7290337721454065, -0.7683202785304566, -1.4733336607562464, -4.272872988377706};
        array6d array_pick_mid = {2.50433206558228, -1.73584236721181, 2.36165410677065, -2.17094959835195, -1.55130368867983, -3.78159839311709};
        array6d array_pick_left_left = {3.57079100608826, -1.25637282550845, 1.81794149080385, -2.13930000881338, -1.54926091829409, -2.72351652780642};
        array6d array_pick_left = {3.0599627494812, -1.64873184780263, 2.17455464998354, -2.03081049541616, -1.54161435762514, -3.26244932809939};
        array6d array_pick_right = {2.05809712409973, -1.44568693757568, 2.1275957266437, -2.25774492839956, -1.57374316850771, -4.27933890024294};
        array6d array_pick_right_right = {1.8982390165329, -1.10214848936115, 1.67234355608095, -2.18172802547597, -1.57016212144961, -4.43200451532473};
        array6d array_rotate1 = {2.3692173957824707, -2.3164030514159144, 1.3390710989581507, -0.9904833000949402, -2.1601603666888636, -2.726298157368795};
        array6d array_rotate2 = {0.9031553864479065, -2.4277192554869593, 1.0507047812091272, -0.964714304809906, -1.9267485777484339, -2.7257021109210413};
        array6d array_rotate3 = {-0.290535275136129, -1.32841757059608, 0.46738320985903, -0.702364699249603, -1.57764035860171, -3.45398217836489};
        array6d array_tray1_top = {0.064236424863339, -1.42052191615615, 0.902454201375143, -1.04965449989352, -1.59723025957216, -3.07310563722719};
        array6d array_tray2_top = {-0.255208794270651, -1.54578061894093, 1.05577546754946, -1.06061519802127, -1.55526000658144, -3.37846404710879};
        array6d array_tray3_top = {-0.53069526353945, -1.57270397762441, 1.04742652574648, -1.04646314800296, -1.56945592561831, -3.61158138910402};
        array6d array_tray1_load = {0.064320102334023, -1.53433151290331, 1.48070460954775, -1.51407157376919, -1.59717017809023, -3.07259160677065};
        array6d array_tray2_load = {-0.255173508320944, -1.6467939815917, 1.58283216158022, -1.48665781438861, -1.55522424379458, -3.37798530260195};
        array6d array_tray3_load = {-0.530647579823629, -1.6887427769103, 1.65178472200503, -1.53478486955676, -1.56944162050356, -3.6110408941852};
        array6d array_drive = {3.18401956558228, -2.55628885845327, 1.20438319841494, -0.691585080032684, -1.76227599779238, -3.09013063112368};
        array6d free_backup_1 = {3.5078, -1.3333, 1.7648, -2.033566, -1.58985, -4.33499};
        array6d free_backup_2 = {2.1859, -1.2849, 2.01598, -2.326377, -1.567803, -2.50999};
        array6d free_SH_1 = {3.1510367393493652, -1.4416437161019822, 1.5042608420001429, -2.213513513604635, -1.6092117468463343, -3.0877655188189905};
};
