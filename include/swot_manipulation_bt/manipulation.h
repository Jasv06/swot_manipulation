#ifndef MANIPULATION_H
#define MANIPULATION_H

#include "ros/ros.h"
#include <ros/package.h>
#include <swot_msgs/SwotManipulation.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/WrenchStamped.h>
#include <tf2_ros/transform_listener.h>
#include <algorithm>
#include <vector>
#include <memory>
#include <functional>
#include <behaviortree_cpp_v3/bt_factory.h>
#include <geometry_msgs/TransformStamped.h>

typedef boost::array<double, 6> array6d;
typedef boost::array<double, 7> array7d;

struct ConditionAction;
struct Tray;

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
        bool initialized;
        std::string tray;
        std::vector<std::string> objects_in_trays;
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
        double left_left_thresh;
        double left_thresh;
        double right_thresh;
        double right_right_thresh;
        std::vector<std::string> objects_in_trays;

    public:
        Manipulation();
        bool callback_service_manipulation(swot_msgs::SwotManipulation::Request &req, swot_msgs::SwotManipulation::Response &res);
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
        void tray_top();
        void registerNodes(BT::BehaviorTreeFactory& factory, Manipulation& manipulation);
};

#endif
