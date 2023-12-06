/**
*       shared_classes.cpp
*
*       @date 14.07.2023
*       @author Joel Santos
*/

#include "swot_manipulation/shared_classes.h"

/**
 *      @brief Constructor of the GetGraspAndMoveGrasp class used to initialize the corresponding member variables.
 *      @param name The behavior tree node name.
 *      @param manipulation The Manipulation class object to access the neccesary data.
 */

GetGraspAndMoveGrasp::GetGraspAndMoveGrasp(const std::string& name, Manipulation& manipulation) : BT::SyncActionNode(name, {}), manipulation_(manipulation) 
{
    
}

/**
 * 	    @brief Destructor of class GetGraspAndMoveGrasp.
 */

GetGraspAndMoveGrasp::~GetGraspAndMoveGrasp()  = default; 

/**
 *      @brief Executes the tick operation of the node GetGraspAndMoveGrasp.
 *      @return The execution status of the node which in this case can be SUCCESS or FAILURE.
 */

BT::NodeStatus GetGraspAndMoveGrasp::tick() 
{
    ROS_INFO("get grasp and move grasp");
    geometry_msgs::Pose grasping_point;
    for(auto i = 0; i < manipulation_.get_request_vector().size(); i++)
    {
        if(manipulation_.getPickTracker()[i].first[0] == manipulation_.get_task_count())
        {
            grasping_point = manipulation_.getPickTracker()[i].second.pose;
        }
        if(manipulation_.getPlaceTracker()[i].first[0] == manipulation_.get_task_count())
        {
            grasping_point = manipulation_.getPlaceTracker()[i].second.pose;
        }
    }

    conditionActions = {
    { [&]() { return grasping_point.position.y >= manipulation_.get_left_left_thresh();},
      [&]() { manipulation_.set_grasping_area("left_left"); manipulation_.sendTargetPosition6d("array_pick_left_left");}
    },
    { [&]() { return grasping_point.position.y < manipulation_.get_left_left_thresh() && grasping_point.position.y >= manipulation_.get_left_thresh();},
      [&]() { manipulation_.set_grasping_area("left"); manipulation_.sendTargetPosition6d("array_pick_left");}
    },
    { [&]() { return grasping_point.position.y < manipulation_.get_left_thresh() && grasping_point.position.y >= manipulation_.get_right_thresh();},
      [&]() { manipulation_.set_grasping_area("mid"); manipulation_.sendTargetPosition6d("array_pick_mid");}
    },
    { [&]() { return grasping_point.position.y < manipulation_.get_right_thresh() && grasping_point.position.y >= manipulation_.get_right_right_thresh();},
      [&]() { manipulation_.set_grasping_area("right"); manipulation_.sendTargetPosition6d("array_pick_right");}
    },
    { [&]() { return grasping_point.position.y < manipulation_.get_right_right_thresh();},
      [&]() { manipulation_.set_grasping_area("right_right"); manipulation_.sendTargetPosition6d("array_pick_right_right");}
    }
    }; 

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
 *      @brief Constructor of the PickPlaceObject class used to initialize the corresponding member variables.
 *      @param name The behavior tree node name.
 *      @param manipulation The Manipulation class object to access the neccesary data.
 */

PickPlaceObject::PickPlaceObject(const std::string& name, Manipulation& manipulation) : BT::SyncActionNode(name, {}) , manipulation_(manipulation)
{

}

/**
 * 	    @brief Destructor of class PickPlaceObject.
 */

PickPlaceObject::~PickPlaceObject()  = default;      

/**
 *      @brief Executes the tick operation of the node PickPlaceObject.
 *      @return The execution status of the node which in this case can be SUCCESS or FAILURE.
 */

BT::NodeStatus PickPlaceObject::tick() 
{
    ROS_INFO("pick object");
    
    geometry_msgs::Pose grasping_point;
    for(auto i = 0; i < manipulation_.get_request_vector().size(); i++)
    {
        if(manipulation_.getPickTracker()[i].first[0] == manipulation_.get_task_count())
        {
            grasping_point = manipulation_.getPickTracker()[i].second.pose;
            manipulation_.get_mani_height(manipulation_.getPickTracker()[i].first.substr(2));
        }
        if(manipulation_.getPlaceTracker()[i].first[0] == manipulation_.get_task_count())
        {
            grasping_point = manipulation_.getPlaceTracker()[i].second.pose;
            manipulation_.get_mani_height(manipulation_.getPlaceTracker()[i].first.substr(2));
        }
    }
    array7d target = {grasping_point.position.x, grasping_point.position.y, grasping_point.position.z + 0.07,
    grasping_point.orientation.x, grasping_point.orientation.y, grasping_point.orientation.z, grasping_point.orientation.w};
    (manipulation_.getRTDE())->cart_target(1, target, manipulation_.get_jnt_vel_() * 0.7, manipulation_.get_jnt_acc_() * 0.5);

    ROS_INFO("Move to grasping_point");

    array6d free_axis = {1,1,1,0,0,0};
    array6d wrench = {0,0,-20,0,0,0};
    ros::Duration(1.0).sleep();
    (manipulation_.getRTDE())->force_target(true, free_axis, wrench, 1.0);
    ROS_INFO("Force Mode activated");
    manipulation_.set_collision_activated(true);
    long int timer = 0;
    while ( (manipulation_.get_collision_detected() == false) && (timer<100) )
    {
        ros::Duration(0.1).sleep();
        timer++;
    }
    manipulation_.set_collision_activated(false);
    manipulation_.set_collision_detected(false);
    (manipulation_.getRTDE())->force_target(false, free_axis , wrench, 1.0);
    ROS_INFO("Force Mode deactivated");

    ros::Duration(0.5).sleep();
    geometry_msgs::TransformStampedConstPtr pcp_pose_;
    pcp_pose_ = ros::topic::waitForMessage<geometry_msgs::TransformStamped>("/tcp_pose", ros::Duration(2.0));

    array7d target_2 = {pcp_pose_->transform.translation.x, pcp_pose_->transform.translation.y, pcp_pose_->transform.translation.z + manipulation_.get_obj_mani_height(), 
                pcp_pose_->transform.rotation.x, pcp_pose_->transform.rotation.y, pcp_pose_->transform.rotation.z, 
                pcp_pose_->transform.rotation.w};
    (manipulation_.getRTDE())->cart_target(1, target_2, manipulation_.get_jnt_vel_()*0.2, manipulation_.get_jnt_acc_()*0.2);
        
    ros::Duration(0.5).sleep();
    if(manipulation_.get_request_vector()[manipulation_.get_task_count()].tasks[manipulation_.get_task_count()].mode == "PICK")
    {
        (manipulation_.getRTDE())->gripper_close(manipulation_.get_gripper_speed_(), manipulation_.get_gripper_force_());
        ros::Duration(0.5).sleep();
        if(manipulation_.getRTDE()->get_gripper_position_per() < 2)
        {
            (manipulation_.getRTDE())->gripper_open(manipulation_.get_gripper_speed_(), manipulation_.get_gripper_force_());
            array7d target = {grasping_point.position.x, grasping_point.position.y, grasping_point.position.z + 0.07,
            grasping_point.orientation.x, grasping_point.orientation.y, grasping_point.orientation.z, grasping_point.orientation.w};
            (manipulation_.getRTDE())->cart_target(1, target, manipulation_.get_jnt_vel_(), manipulation_.get_jnt_acc_());
            return BT::NodeStatus::FAILURE;
        }
    }
    if(manipulation_.get_request_vector()[manipulation_.get_task_count()].tasks[manipulation_.get_task_count()].mode == "PLACE")
    {
        (manipulation_.getRTDE())->gripper_open(manipulation_.get_gripper_speed_(), manipulation_.get_gripper_force_());
    }           
    return BT::NodeStatus::SUCCESS;
}

/**
 *      @brief Constructor of the MoveHomePos class used to initialize the corresponding member variables.
 *      @param name The behavior tree node name.
 *      @param manipulation The Manipulation class object to access the neccesary data.
 */

MoveHomePos::MoveHomePos(const std::string& name, Manipulation& manipulation) : BT::SyncActionNode(name, {}), manipulation_(manipulation) 
{

}

/**
 * 	    @brief Destructor of class MoveHomePos.
 */

MoveHomePos::~MoveHomePos()  = default;    

/**
 *      @brief Executes the tick operation of the node MoveHomePos.
 *      @return The execution status of the node which in this case can be SUCCESS or FAILURE.
 */

BT::NodeStatus MoveHomePos::tick() 
{
    ROS_INFO("move home pos");
    if(manipulation_.get_last_pos() == "tray")
    {
        manipulation_.sendTargetPosition6d("array_rotate2");
        manipulation_.sendTargetPosition6d("array_rotate1");
        manipulation_.sendTargetPosition6d("array_scan_mid");
        manipulation_.set_response_status("FINISHED");
        return BT::NodeStatus::SUCCESS; 
    }
    else
    {  
        manipulation_.sendTargetPosition6d("array_scan_mid");
        manipulation_.set_response_status("FINISHED");
        return BT::NodeStatus::SUCCESS;
    }
}

/**
 *      @brief Constructor of the MoveToDrivePose class used to initialize the corresponding member variables.
 *      @param name The behavior tree node name.
 *      @param manipulation The Manipulation class object to access the neccesary data.
 */

MoveToDrivePose::MoveToDrivePose(const std::string& name, Manipulation& manipulation) : BT::SyncActionNode(name, {}) , manipulation_(manipulation)
{

}

/**
 * 	    @brief Destructor of class MoveToDrivePose.
 */

MoveToDrivePose::~MoveToDrivePose()  = default;  

/**
 *      @brief Executes the tick operation of the node MoveToDrivePose.
 *      @return The execution status of the node which in this case can be SUCCESS or FAILURE.
 */

BT::NodeStatus MoveToDrivePose::tick() 
{
    ROS_INFO("move to drive pos");
    if(manipulation_.get_request_vector()[manipulation_.get_task_count()].tasks[manipulation_.get_task_count()].mode != "DRIVE")
    {
        return BT::NodeStatus::FAILURE;
    }
    manipulation_.sendTargetPosition6d("array_drive");
    manipulation_.set_response_status("FINISHED");
    return BT::NodeStatus::SUCCESS;
}
