/**
*       place_classes.cpp
*
*       @date 14.07.2023
*       @author Joel Santos
*/

#include "swot_manipulation_bt/place_classes.h"

/**
 *      @brief Constructor of the CheckObjRequired class used to initialize the corresponding member variables.
 *      @param name The behavior tree node name.
 *      @param manipulation The Manipulation class object to access the neccesary data.
 */

CheckObjRequired::CheckObjRequired(const std::string& name, Manipulation& manipulation) : BT::SyncActionNode(name, {}), manipulation_(manipulation) 
{

}

/**
 * 	    @brief Destructor of class CheckObjRequired.
 */

CheckObjRequired::~CheckObjRequired() override = default;      

/**
 *      @brief Executes the tick operation of the node CheckObjRequired.
 *      @return The execution status of the node which in this case can be SUCCESS or FAILURE.
 */

BT::NodeStatus CheckObjRequired::tick() override
{
    manipulation_.set_collision(false);
    if (manipulation_.get_request().save == "SAVE_1")
    {
        ROS_INFO("Object from tray 1 required");
        objects_in_trays[0] = "";
    }
    else if (manipulation_.get_request().save == "SAVE_2")
    {
        ROS_INFO("Object from tray 2 required");
        objects_in_trays[1] = "";
    }
    else
    {
        ROS_INFO("Object from tray 3 required");
        objects_in_trays[2] = "";
    }
    return BT::NodeStatus::SUCCESS;
}     

/**
 *      @brief Constructor of the CheckWSFree class used to initialize the corresponding member variables.
 *      @param name The behavior tree node name.
 *      @param manipulation The Manipulation class object to access the neccesary data.
 */

CheckWSFree::CheckWSFree(const std::string& name, Manipulation& manipulation) : BT::SyncActionNode(name, {}), manipulation_(manipulation) 
{

}

/**
 * 	    @brief Destructor of class CheckWSFree.
 */

CheckWSFree::~CheckWSFree() override = default;      

/**
 *      @brief Executes the tick operation of the node CheckWSFree.
 *      @return The execution status of the node which in this case can be SUCCESS or FAILURE.
 */

BT::NodeStatus CheckWSFree::tick() override
{
    ROS_INFO("check ws free");
    if(manipulation_.req_.task == "RED_CONTAINER" || manipulation_.req_.task == "BLUE_CONTAINER")
    {

    }
    else
    {
    swot_msgs::SwotFreeSpot srv_free;
    if (ros::service::waitForService("FreeSpotServer", ros::Duration(5.0)))
    {
        std::cout << "FreeSpotServer" << std::endl;
        (manipulation_.getRTDE())->joint_target(manipulation_.array_scan_left, manipulation_.get_jnt_vel_(), manipulation_.get_jnt_acc_());
        ros::Duration(2).sleep();
            if (!(manipulation_.get_service_client_free()).call(srv_free))
            {
                (manipulation_.getRTDE())->joint_target(manipulation_.array_scan_right, manipulation_.get_jnt_vel_(), manipulation_.get_jnt_acc_());
                ros::Duration(2).sleep();
                if (!(manipulation_.get_service_client_free()).call(srv_free))
                {
                    (manipulation_.getRTDE())->joint_target(manipulation_.array_scan_mid, manipulation_.get_jnt_vel_(), manipulation_.get_jnt_acc_());                   
                    ros::Duration(2).sleep();
                    if(!(manipulation_.get_service_client_free()).call(srv_free))
                    {
                        return BT::NodeStatus::FAILURE;
                    }
                }
            }
    }
    else
    {
        ROS_WARN("Couldn't find ROS Service \"SwotFreeSpot\"");
        return BT::NodeStatus::FAILURE;
    }
    manipulation_.set_grasping_point(srv_free.response.pose);
    std::cout << manipulation_.get_grasping_point() << std::endl;
    }
    return BT::NodeStatus::SUCCESS;
}

/**
 *      @brief Constructor of the MoveToTray class used to initialize the corresponding member variables.
 *      @param name The behavior tree node name.
 *      @param manipulation The Manipulation class object to access the neccesary data.
 */

MoveToTray::MoveToTray(const std::string& name, Manipulation& manipulation) : BT::SyncActionNode(name, {}), manipulation_(manipulation) 
{

}

/**
 * 	    @brief Destructor of class MoveToTray.
 */

MoveToTray::~MoveToTray() override = default;   

/**
 *      @brief Executes the tick operation of the node MoveToTray.
 *      @return The execution status of the node which in this case can be SUCCESS or FAILURE.
 */

BT::NodeStatus MoveToTray::tick() override
{
    ROS_INFO("move to tray");
    (manipulation_.getRTDE())->joint_target(manipulation_.array_rotate1, manipulation_.get_jnt_vel_(), manipulation_.get_jnt_acc_());
    (manipulation_.getRTDE())->joint_target(manipulation_.array_rotate2, manipulation_.get_jnt_vel_(), manipulation_.get_jnt_acc_());
    manipulation_.tray_top();       
    return BT::NodeStatus::SUCCESS;   
}

/**
 *      @brief Constructor of the PickFromTray class used to initialize the corresponding member variables.
 *      @param name The behavior tree node name.
 *      @param manipulation The Manipulation class object to access the neccesary data.
 */

PickFromTray::PickFromTray(const std::string& name, Manipulation& manipulation) : BT::SyncActionNode(name, {}), manipulation_(manipulation) 
{

}

/**
 * 	    @brief Destructor of class PickFromTray.
 */

PickFromTray::~PickFromTray() override = default; 

/**
 *      @brief Executes the tick operation of the node PickFromTray.
 *      @return The execution status of the node which in this case can be SUCCESS or FAILURE.
 */

BT::NodeStatus PickFromTray::tick() override
{
    ROS_INFO("pick from tray");
    if (manipulation_.get_tray() == "SAVE_1")
    {
        (manipulation_.getRTDE())->joint_target(manipulation_.array_tray1_load, manipulation_.get_jnt_vel_(), manipulation_.get_jnt_acc_());  
    }
    else if (manipulation_.get_tray() == "SAVE_2")
    {
        (manipulation_.getRTDE())->joint_target(manipulation_.array_tray2_load, manipulation_.get_jnt_vel_(), manipulation_.get_jnt_acc_());        
    }
    else
    {
        (manipulation_.getRTDE())->joint_target(manipulation_.array_tray3_load, manipulation_.get_jnt_vel_(), manipulation_.get_jnt_acc_());
    }
    ros::Duration(0.5).sleep();

    manipulation_.set_collision(false);
    array6d free_axis = {1,1,1,0,0,0};
    array6d wrench = {0,0,-20,0,0,0};
    (manipulation_.getRTDE())->force_target(true, free_axis, wrench, 1.0);
    ROS_INFO("Force Mode activated");

    ros::Duration(0.5).sleep();
    manipulation_.set_collision_activated(true);

    long int  timer = 0;
    while ( (manipulation_.get_collision() == false) && (timer<100) )
    {
        ros::Duration(0.1).sleep();
        timer++;
    }

    (manipulation_.getRTDE())->force_target(false, free_axis , wrench, 1.0);
    ROS_INFO("Force Mode deactivated");
    manipulation_.set_collision_activated(false);
    manipulation_.set_collision(false);
    ros::Duration(0.1).sleep();
    geometry_msgs::TransformStampedConstPtr pcp_pose_;
    pcp_pose_ = ros::topic::waitForMessage<geometry_msgs::TransformStamped>("/tcp_pose", ros::Duration(2.0));

    array7d target = {pcp_pose_->transform.translation.x, pcp_pose_->transform.translation.y, pcp_pose_->transform.translation.z + 0.006, 
                    pcp_pose_->transform.rotation.x, pcp_pose_->transform.rotation.y, pcp_pose_->transform.rotation.z, 
                    pcp_pose_->transform.rotation.w};
    (manipulation_.getRTDE())->cart_target(1, target, manipulation_.get_jnt_vel_()*0.2, manipulation_.get_jnt_acc_()*0.2);

    (manipulation_.getRTDE())->gripper_close(manipulation_.get_gripper_speed_(), manipulation_.get_gripper_force_());
    manipulation_.tray_top();          
    return BT::NodeStatus::SUCCESS;
}

/**
 *      @brief Constructor of the MoveToDropPos class used to initialize the corresponding member variables.
 *      @param name The behavior tree node name.
 *      @param manipulation The Manipulation class object to access the neccesary data.
 */

MoveToDropPos::MoveToDropPos(const std::string& name, Manipulation& manipulation) : BT::SyncActionNode(name, {}), manipulation_(manipulation) 
{

}

/**
 * 	    @brief Destructor of class MoveToDropPos.
 */

MoveToDropPos::~MoveToDropPos() override = default;

/**
 *      @brief Executes the tick operation of the node MoveToDropPos.
 *      @return The execution status of the node which in this case can be SUCCESS or FAILURE.
 */

BT::NodeStatus MoveToDropPos::tick() override
{
    ROS_INFO("move to drop pos");
    (manipulation_.getRTDE())->joint_target(manipulation_.array_rotate2, manipulation_.get_jnt_vel_(), manipulation_.get_jnt_acc_());
    (manipulation_.getRTDE())->joint_target(manipulation_.array_rotate1, manipulation_.get_jnt_vel_(), manipulation_.get_jnt_acc_());
    (manipulation_.getRTDE())->joint_target(manipulation_.array_scan_mid, manipulation_.get_jnt_vel_(), manipulation_.get_jnt_acc_());
    manipulation_.set_last_pos("mid");
    return BT::NodeStatus::SUCCESS;
}