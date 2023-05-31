#ifndef MANIPULATION_H
#define MANIPULATION_H

#include "ros/ros.h"
#include <ros/package.h>
#include <swot_ur/ur_rtde.h>
#include <swot_msgs/SwotManipulation.h>
#include <swot_msgs/SwotObjectMatching2023.h>
#include <swot_msgs/SwotFreeSpot.h>
#include <swot_msgs/SwotWeightedBBox.h>
#include <swot_msgs/SwotBBoxCheck.h>
#include <darknet_ros_msgs/BoundingBox.h>
#include <darknet_ros_msgs/BoundingBoxes.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/WrenchStamped.h>
#include <tf2_ros/transform_listener.h>
#include <algorithm>
#include <vector>
#include <memory>
#include <behaviortree_cpp_v3/loggers/bt_zmq_publisher.h>
#include <behaviortree_cpp_v3/bt_factory.h>
#include <behaviortree_cpp_v3/condition_node.h>
#include <behaviortree_cpp_v3/action_node.h>

typedef boost::array<double, 6> array6d;
typedef boost::array<long int, 6> array6i;
typedef boost::array<double, 7> array7d;

class Manipulation
{    
    private:

        std::string xml_file;
        std::string last_pos;    
        std::string grasping_area; 
        const double wrench_limit;
        bool collision_detected;
        bool collision_activated;
        ros::NodeHandle nh_;
        ros::Subscriber sub_wrench;
        geometry_msgs::Pose grasping_point;
        static inline bool initialized = false;
        std::string tray;
        
    public:
        std::array<array6d, 3> scan_pose = {array_scan_left_yolo, array_scan_right_yolo, array_scan_mid};
        std::unique_ptr<URRTDE> rtde;
        ros::ServiceServer service_server;
        ros::ServiceClient service_client_matching;
        ros::ServiceClient service_client_free;
        swot_msgs::SwotManipulation::Request req_;
        swot_msgs::SwotManipulation::Response res_;
        double gripper_speed_;
        double gripper_force_;
        double jnt_vel_;
        double jnt_acc_;
        int move_duration;
        double blend_;
        Manipulation() ;
        bool callback_service_manipulation(swot_msgs::SwotManipulation::Request &req, swot_msgs::SwotManipulation::Response &res)
        virtual ~Manipulation()
        void callback_wrench(const geometry_msgs::WrenchStamped &msg);
        
        void set_last_pos(std::string last_pos);
        std::string get_last_pos() const;
        
        void set_grasping_area(std::string grasping_area);
        std::string get_grasping_area() const;

        void set_collision(bool collision);
        bool get_collision() const;

        void set_collision_activated(bool collision);
        bool get_collision_activated() const;
        
        void set_tray(std::string tray);
        std::string get_tray() const;
    
        void set_grasping_point(geometry_msgs::Pose grasping);
        geometry_msgs::Pose get_grasping_point() const;
    
        swot_msgs::SwotManipulation::Request get_request() const;
        swot_msgs::SwotManipulation::Response get_response() const;
        ros::NodeHandle get_nh();
        
        array6d array_scan_mid;
        array6d array_scan_left;
        array6d array_scan_left_yolo;
        array6d array_scan_right_yolo;
        array6d array_scan_left_pc;
        array6d array_scan_right_pc;
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
};

#endif
