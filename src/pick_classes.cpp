/**
*       pick_classes.cpp
*
*       @date 19.09.2023
*       @author Joel Santos
*/

#include "swot_manipulation/pick_classes.h"

/**
 *      @brief Constructor of the MoveToScan class used to initialize the corresponding member variables.
 *      @param name The behavior tree node name.
 *      @param manipulation The Manipulation class object to access the neccesary data.
 */

MoveToScan::MoveToScan(const std::string& name, Manipulation& manipulation) : BT::SyncActionNode(name, {}), manipulation_(manipulation) 
{

}

/**
 * 	    @brief Destructor of class MoveToScan.
 */

MoveToScan::~MoveToScan() = default;      

/**
 *      @brief Executes the tick operation of the node MoveToScan.
 *      @return The execution status of the node which in this case can be SUCCESS or FAILURE.
 */

BT::NodeStatus MoveToScan::tick() 
{
    ROS_INFO("move to scan");
    if(manipulation_.get_request_vector()[manipulation_.get_task_count()].tasks[manipulation_.get_task_count()].mode != "PICK")
    {
        return BT::NodeStatus::FAILURE;
    }
    manipulation_.set_collision_detected(false);
    std::string scan;
    if(manipulation_.get_count() == 0)
    {
        scan = "array_scan_left_yolo_" + get_workspace_dimensions_matching_object().workspace_dimensions[manipulation_.index(req_array_[0].tasks[get_task_count()].task)];
        manipulation_.sendTargetPosition6d(scan);
    }
    if(manipulation_.get_count() == 1)
    {
        manipulation_.sendTargetPosition6d("array_scan_right_yolo_");
    }

    return BT::NodeStatus::SUCCESS; 
}  

/**
 *      @brief Constructor of the ScanWorkSpace class used to initialize the corresponding member variables.
 *      @param name The behavior tree node name.
 *      @param manipulation The Manipulation class object.
 */

ScanWorkSpace::ScanWorkSpace(const std::string& name, Manipulation& manipulation) : BT::SyncActionNode(name, {}), manipulation_(manipulation) 
{
    
}

/**
 * 	    @brief Destructor of class ScanWorkSpace.
 */

ScanWorkSpace::~ScanWorkSpace()  = default;      

/**
 *      @brief Executes the tick operation of the node ScanWorkSpace.
 *      @return The execution status of the node which in this case can be either SUCCESS or FAILURE.
 */

BT::NodeStatus ScanWorkSpace::tick() 
{
    ROS_INFO("scan workspace");
    swot_msgs::SwotObjectMatching srv_match;
    for(auto i = 0; i < manipulation_.get_request_vector().size(); i++)
    {
        if(manipulation_.getTaskTrack()[i] == "UNKNOWN" || manipulation_.getTaskTrack()[i] == "NOTFOUND" || manipulation_.getTaskTrack()[i] == "NOTFULFILLED")
        {
            srv_match.request.objects[i] = manipulation_.get_request(i).tasks[i].object;
        }
    }
    
    for(auto i = 0; i < 4; i++)
    {
        srv_match.request.ws_dimensions[i] = manipulation_.get_ws_dim()[i];
    }    
    if(ros::service::waitForService("ObjectMatchingServer", ros::Duration(3.0)) == false)
    {
        std::cout << "Service not available" << std::endl;
        return BT::NodeStatus::FAILURE;   
    }
    if(!(manipulation_.get_service_client_matching()).call(srv_match))
    {
        ROS_WARN("Couldn't find ROS Service \"SwotObjectMatching\"");
        return BT::NodeStatus::FAILURE;
    }
    for(auto i = 0; i < manipulation_.getPickTracker().size(); i++)
    {
        manipulation_.getPickTracker()[i].second = srv_match.response.poses[i];
    }
    for(auto i = 0; i < manipulation_.getPickTracker().size() ; i++)
    {
        if(srv_match.response.poses[i].error_code == 0)
        {
            manipulation_.getTaskTrack()[i] = "FOUND";
        }
        else
        {
            manipulation_.getTaskTrack()[i] = "NOTFOUND";
        }
    }
    return BT::NodeStatus::SUCCESS;
}

/**
 *      @brief Constructor of the MoveUp class used to initialize the corresponding member variables.
 *      @param name The behavior tree node name.
 *      @param manipulation The Manipulation class object.
 */

MoveUp::MoveUp(const std::string& name, Manipulation& manipulation) : BT::SyncActionNode(name, {}), manipulation_(manipulation) 
{
    conditionActions = {
    { [&]() { return manipulation_.get_grasping_area() == "left_left";},
      [&]() { manipulation_.setTargetPosition6d("array_pick_left_left"); manipulation_.sendTargetPosition6d();}
    },
    { [&]() { return manipulation_.get_grasping_area() == "left";},
      [&]() { manipulation_.setTargetPosition6d("array_pick_left"); manipulation_.sendTargetPosition6d();}
    },
    { [&]() { return manipulation_.get_grasping_area() == "mid";},
      [&]() { manipulation_.setTargetPosition6d("array_pick_mid"); manipulation_.sendTargetPosition6d();}
    },
    { [&]() { return manipulation_.get_grasping_area() == "right";},
      [&]() { manipulation_.setTargetPosition6d("array_pick_right"); manipulation_.sendTargetPosition6d();}
    },
    { [&]() { return manipulation_.get_grasping_area() == "right_right";},
      [&]() { manipulation_.setTargetPosition6d("array_pick_right_right"); manipulation_.sendTargetPosition6d();}
    }
    };
}

/**
 * 	@brief Destructor of class MoveUp.
 */

MoveUp::~MoveUp()  = default;      

/**
 *      @brief Executes the tick operation of the node MoveUp.
 *      @return The execution status of the node.
 */

BT::NodeStatus MoveUp::tick() 
{
    ROS_INFO("move up ");  
    geometry_msgs::Pose grasping_point;
    for(auto i = 0; i < manipulation_.getPickTracker().size(); i++)
    {
        if(manipulation_.getPickTracker()[i].first[1] == '1' && manipulation_.getPickTracker()[i].first.substr(2) == manipulation_.object_in_gripper)
        {
            grasping_point = manipulation_.getPickTracker()[i].second.pose;
        }
    }         
    array7d target = {grasping_point.position.x, grasping_point.position.y, grasping_point.position.z + 0.07,
    grasping_point.orientation.x, grasping_point.orientation.y, grasping_point.orientation.z, grasping_point.orientation.w};
    (manipulation_.getRTDE())->cart_target(1, target, manipulation_.get_jnt_vel_(), manipulation_.get_jnt_acc_());           

    for (const auto& conditionAction : conditionActions) 
    {
        if (conditionAction.condition()) 
        {
            conditionAction.action();
            return BT::NodeStatus::SUCCESS;
        }
    }
    std::cout << "No matching action found." << std::endl;
    return BT::NodeStatus::FAILURE;
} 

/**
 *      @brief Constructor of the DropObjectInTray class used to initialize the corresponding member variables.
 *      @param name The behavior tree node name.
 *      @param manipulation The Manipulation class object.
 */

DropObjectInTray::DropObjectInTray(const std::string& name, Manipulation& manipulation) : BT::SyncActionNode(name, {}), manipulation_(manipulation) 
{
    trays = {
        {manipulation_.array_tray1_top, manipulation_.array_tray1_load, "SAVE_1", manipulation_.objects_in_trays[0]},
        {manipulation_.array_tray2_top, manipulation_.array_tray2_load, "SAVE_2", manipulation_.objects_in_trays[1]},
        {manipulation_.array_tray3_top, manipulation_.array_tray3_load, "SAVE_3", manipulation_.objects_in_trays[2]}
    };
}

/**
 * 	@brief Destructor of class DropObjectInTray.
 */

DropObjectInTray::~DropObjectInTray()  = default;     

/**
 *      @brief Executes the tick operation of the node DropObjectInTray.
 *      @return The execution status of the node.
 */

BT::NodeStatus DropObjectInTray::tick() 
{
    ROS_INFO("drop object in tray");
    manipulation_.setTargetPosition6d("array_rotate1"); manipulation_.sendTargetPosition6d();
    manipulation_.setTargetPosition6d("array_rotate2"); manipulation_.sendTargetPosition6d();

    for (const auto& tray : trays) {
        if (tray.trayObject.empty() && manipulation_.get_request_vector()[manipulation_.get_task_count()].tasks[manipulation_.get_task_count()].save == tray.savePosition) {
            (manipulation_.getRTDE())->joint_target(tray.topPose, manipulation_.get_jnt_vel_(), manipulation_.get_jnt_acc_());
            ros::Duration(1).sleep();                    
            manipulation_.set_collision_detected(false);
            array6d free_axis = {1,1,1,0,0,0};
            array6d wrench = {0,0,-20,0,0,0};
            (manipulation_.getRTDE())->force_target(true, free_axis, wrench, 1.0);
            ROS_INFO("Force Mode activated");

            ros::Duration(0.5).sleep();
            manipulation_.set_collision_activated(true);

            long int timer = 0;
            while ( (manipulation_.get_collision_detected() == false) && (timer<100) )
            {
                ros::Duration(0.1).sleep();
                timer++;
            }
            
            (manipulation_.getRTDE())->force_target(false, free_axis , wrench, 1.0);
            ROS_INFO("Force Mode deactivated");
            manipulation_.set_collision_activated(false);
            manipulation_.set_collision_detected(false);

            ros::Duration(0.1).sleep();

            geometry_msgs::TransformStampedConstPtr pcp_pose_;
            pcp_pose_ = ros::topic::waitForMessage<geometry_msgs::TransformStamped>("/tcp_pose", ros::Duration(2.0));
            array7d target = {pcp_pose_->transform.translation.x, pcp_pose_->transform.translation.y, pcp_pose_->transform.translation.z + 0.006, 
                            pcp_pose_->transform.rotation.x, pcp_pose_->transform.rotation.y, pcp_pose_->transform.rotation.z, 
                            pcp_pose_->transform.rotation.w};
            (manipulation_.getRTDE())->cart_target(1, target, manipulation_.get_jnt_vel_()*0.2, manipulation_.get_jnt_acc_()*0.2);
            manipulation_.set_tray(tray.savePosition);
            tray.trayObject = manipulation_.get_request_vector()[manipulation_.get_task_count()].tasks[manipulation_.get_task_count()].object;
            break;
        }
    }
    (manipulation_.getRTDE())->gripper_open(manipulation_.get_gripper_speed_(), manipulation_.get_gripper_force_());
    manipulation_.set_last_pos("tray");
    manipulation_.tray_top();
    return BT::NodeStatus::SUCCESS;
}
