/**
*       pick.cpp
*
*       @date 14.07.2023
*       @author Joel Santos
*/

#include "swot_manipulation_bt/pick_classes.h"

MoveToScan::MoveToScan(const std::string& name, Manipulation& manipulation) : BT::SyncActionNode(name, {}), manipulation_(manipulation) {}
MoveToScan::~MoveToScan() override = default;      
BT::NodeStatus MoveToScan::tick() override
{
    manipulation_.set_collision(false);
    ROS_INFO("move to scan");
    (manipulation_.rtde)->joint_target(manipulation_.array_scan_mid, manipulation_.jnt_vel_, manipulation_.jnt_acc_);
    return BT::NodeStatus::SUCCESS; 
}  


ScanWorkSpace::ScanWorkSpace(const std::string& name, Manipulation& manipulation) : BT::SyncActionNode(name, {}), manipulation_(manipulation), count(0) {}
ScanWorkSpace::~ScanWorkSpace() override = default;      
BT::NodeStatus ScanWorkSpace::tick() override
{
    swot_msgs::SwotObjectMatching2023 srv_match;
    srv_match.request.object = manipulation_.get_request().object;
    ROS_INFO("scan workspace");
    if(count == 0)
    {
        (manipulation_.rtde)->joint_target(manipulation_.array_scan_left_yolo, manipulation_.jnt_vel_, manipulation_.jnt_acc_);
    }
    else if(count == 1)
    {
        (manipulation_.rtde)->joint_target(manipulation_.array_scan_right_yolo, manipulation_.jnt_vel_, manipulation_.jnt_acc_);
    }
    else
    {
        (manipulation_.rtde)->joint_target(manipulation_.array_scan_mid, manipulation_.jnt_vel_, manipulation_.jnt_acc_);
    }

    if(ros::service::waitForService("ObjectMatchingServer", ros::Duration(3.0)) == false)
    {
        count++;
        return BT::NodeStatus::FAILURE;   
    }
    ros::Duration(1).sleep();
    if(!(manipulation_.service_client_matching).call(srv_match))
    {
        count++;
        ROS_WARN("Couldn't find ROS Service \"SwotObjectMatching\"");
        return BT::NodeStatus::FAILURE;
    }
    ros::Duration(1).sleep();
    if (srv_match.response.posture == "STANDING" || srv_match.response.posture == "FAILED")
    {
        count++;
        return BT::NodeStatus::FAILURE;
    }
    manipulation_.set_grasping_point(srv_match.response.pose);
    return BT::NodeStatus::SUCCESS;
}


MoveUp::MoveUp(const std::string& name, Manipulation& manipulation) : BT::SyncActionNode(name, {}), manipulation_(manipulation) 
{
    areaTargets = {
        {"left_left", manipulation_.array_pick_left_left},
        {"left", manipulation_.array_pick_left},
        {"mid", manipulation_.array_pick_mid},
        {"right", manipulation_.array_pick_right},
        {"right_right", manipulation_.array_pick_right_right}
    };
}
MoveUp::~MoveUp() override = default;      
BT::NodeStatus MoveUp::tick() override
{
    ROS_INFO("move up ");           
    array7d target = {manipulation_.get_grasping_point().position.x, manipulation_.get_grasping_point().position.y, manipulation_.get_grasping_point().position.z + 0.07,
    manipulation_.get_grasping_point().orientation.x, manipulation_.get_grasping_point().orientation.y, manipulation_.get_grasping_point().orientation.z,
    manipulation_.get_grasping_point().orientation.w};
    (manipulation_.rtde)->cart_target(1, target, manipulation_.jnt_vel_, manipulation_.jnt_acc_);           

    std::string graspingArea = manipulation_.get_grasping_area();
    array6d defaultTarget = manipulation_.array_pick_mid;

    auto find = std::find_if(areaTargets.begin(), areaTargets.end(), [&](const auto& pair){return pair.first == graspingArea;});
    if(find != areaTargets.end())
    {
        (manipulation_.rtde)->joint_target(find->second, manipulation_.jnt_vel_, manipulation_.jnt_acc_);
    }
    else
    {
        (manipulation_.rtde)->joint_target(defaultTarget, manipulation_.jnt_vel_, manipulation_.jnt_acc_);
    }
    return BT::NodeStatus::SUCCESS;
} 


DropObjectInTray::DropObjectInTray(const std::string& name, Manipulation& manipulation) : BT::SyncActionNode(name, {}), manipulation_(manipulation) 
{
    trays = {
        {manipulation_.array_tray1_top, manipulation_.array_tray1_load, "SAVE_1", objects_in_trays[0]},
        {manipulation_.array_tray2_top, manipulation_.array_tray2_load, "SAVE_2", objects_in_trays[1]},
        {manipulation_.array_tray3_top, manipulation_.array_tray3_load, "SAVE_3", objects_in_trays[2]}
    };
}
DropObjectInTray::~DropObjectInTray() override = default;     
BT::NodeStatus DropObjectInTray::tick() override
{
    ROS_INFO("drop object in tray");
    (manipulation_.rtde)->joint_target(manipulation_.array_rotate1, manipulation_.jnt_vel_, manipulation_.jnt_acc_);
    (manipulation_.rtde)->joint_target(manipulation_.array_rotate2, manipulation_.jnt_vel_, manipulation_.jnt_acc_);

    for (const auto& tray : trays) {
        if (tray.trayObject.empty() && manipulation_.get_request().save == tray.savePosition) {
            (manipulation_.rtde)->joint_target(tray.topPose, manipulation_.jnt_vel_, manipulation_.jnt_acc_);
            ros::Duration(1).sleep();                    
            manipulation_.set_collision(false);
            array6d free_axis = {1,1,1,0,0,0};
            array6d wrench = {0,0,-20,0,0,0};
            (manipulation_.rtde)->force_target(true, free_axis, wrench, 1.0);
            ROS_INFO("Force Mode activated");

            ros::Duration(0.5).sleep();
            manipulation_.set_collision_activated(true);

            long int timer = 0;
            while ( (manipulation_.get_collision() == false) && (timer<100) )
            {
                ros::Duration(0.1).sleep();
                timer++;
            }
            
            (manipulation_.rtde)->force_target(false, free_axis , wrench, 1.0);
            ROS_INFO("Force Mode deactivated");
            manipulation_.set_collision_activated(false);
            manipulation_.set_collision(false);

            ros::Duration(0.1).sleep();

            geometry_msgs::TransformStampedConstPtr pcp_pose_;
            pcp_pose_ = ros::topic::waitForMessage<geometry_msgs::TransformStamped>("/tcp_pose", ros::Duration(2.0));
            array7d target = {pcp_pose_->transform.translation.x, pcp_pose_->transform.translation.y, pcp_pose_->transform.translation.z + 0.006, 
                            pcp_pose_->transform.rotation.x, pcp_pose_->transform.rotation.y, pcp_pose_->transform.rotation.z, 
                            pcp_pose_->transform.rotation.w};
            (manipulation_.rtde)->cart_target(1, target, manipulation_.jnt_vel_*0.2, manipulation_.jnt_acc_*0.2);
            manipulation_.set_tray(tray.savePosition);
            tray.trayObject = manipulation_.get_request().object;
            break;
        }
    }
    (manipulation_.rtde)->gripper_open(manipulation_.gripper_speed_, manipulation_.gripper_force_);
    manipulation_.set_last_pos("tray");
    manipulation_.tray_top();
    return BT::NodeStatus::SUCCESS;
}