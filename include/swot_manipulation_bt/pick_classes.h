/**
*       pick_classes.h
*
*       @date 14.07.2023
*       @author Joel Santos
*/

#ifndef PICK_H
#define PICK_H

class MoveToScan : public BT::SyncActionNode
{
    private:
        Manipulation& manipulation_;

    public:
        MoveToScan(const std::string& name, Manipulation& manipulation);
        ~MoveToScan() override;      
        virtual BT::NodeStatus tick() override;            
}; 

class ScanWorkSpace : public BT::SyncActionNode
{
    private:
        Manipulation& manipulation_;
        int count;

    public:
        ScanWorkSpace(const std::string& name, Manipulation& manipulation);
        ~ScanWorkSpace() override;      
        virtual BT::NodeStatus tick() override;
};

class GetGraspAndMoveGrasp : public BT::SyncActionNode
{
    private:
        Manipulation& manipulation_;

    public:
        GetGraspAndMoveGrasp(const std::string& name, Manipulation& manipulation) : BT::SyncActionNode(name, {}), manipulation_(manipulation) {}
        ~GetGraspAndMoveGrasp() override = default; 
        std::vector<ConditionAction> conditionActions = {
        { [&]() { return manipulation_.get_grasping_point().position.y >= manipulation_.left_left_thresh;},
          [&]() {
              (manipulation_.rtde)->joint_target(manipulation_.array_pick_left_left, manipulation_.jnt_vel_, manipulation_.jnt_acc_);
              manipulation_.set_grasping_area("left_left");
          }
        },
        { [&]() { return manipulation_.get_grasping_point().position.y < manipulation_.left_left_thresh && manipulation_.get_grasping_point().position.y >= manipulation_.left_thresh;},
          [&]() {
              (manipulation_.rtde)->joint_target(manipulation_.array_pick_left, manipulation_.jnt_vel_, manipulation_.jnt_acc_);
              manipulation_.set_grasping_area("left");
          }
        },
        { [&]() { return manipulation_.get_grasping_point().position.y < manipulation_.left_thresh && manipulation_.get_grasping_point().position.y >= manipulation_.right_thresh;},
          [&]() {
              (manipulation_.rtde)->joint_target(manipulation_.array_pick_mid, manipulation_.jnt_vel_, manipulation_.jnt_acc_);
              manipulation_.set_grasping_area("mid");
          }
        },
        { [&]() { return manipulation_.get_grasping_point().position.y < manipulation_.right_thresh && manipulation_.get_grasping_point().position.y >= manipulation_.right_right_thresh;},
          [&]() {
              (manipulation_.rtde)->joint_target(manipulation_.array_pick_right, manipulation_.jnt_vel_, manipulation_.jnt_acc_);
              manipulation_.set_grasping_area("right");
          }
        },
        { [&]() { return manipulation_.get_grasping_point().position.y < manipulation_.right_right_thresh;},
          [&]() {
              (manipulation_.rtde)->joint_target(manipulation_.array_pick_right_right, manipulation_.jnt_vel_, manipulation_.jnt_acc_);
              manipulation_.set_grasping_area("right_right");
          }
        }
        };     
        virtual BT::NodeStatus tick() override
        {
            ROS_INFO("get grasp and move grasp");

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
};

class PickObject : public BT::SyncActionNode
{
    private:
        Manipulation& manipulation_;

    public:
        PickObject(const std::string& name, Manipulation& manipulation) : BT::SyncActionNode(name, {}) , manipulation_(manipulation){}
        ~PickObject() override = default;      
        virtual BT::NodeStatus tick() override
        {
            ROS_INFO("pick object");
            
            array7d target = {manipulation_.get_grasping_point().position.x, manipulation_.get_grasping_point().position.y, manipulation_.get_grasping_point().position.z + 0.07,
            manipulation_.get_grasping_point().orientation.x, manipulation_.get_grasping_point().orientation.y, manipulation_.get_grasping_point().orientation.z,
            manipulation_.get_grasping_point().orientation.w};
            (manipulation_.rtde)->cart_target(1, target, manipulation_.jnt_vel_ * 0.7, manipulation_.jnt_acc_ * 0.5);

            ROS_INFO("Move to grasping_point");

            array6d free_axis = {1,1,1,0,0,0};
            array6d wrench = {0,0,-20,0,0,0};
            ros::Duration(1.0).sleep();
            (manipulation_.rtde)->force_target(true, free_axis, wrench, 1.0);
            ROS_INFO("Force Mode activated");
            manipulation_.set_collision_activated(true);
            long int timer = 0;
            while ( (manipulation_.get_collision() == false) && (timer<100) )
            {
                ros::Duration(0.1).sleep();
                timer++;
            }
            manipulation_.set_collision_activated(false);
            manipulation_.set_collision(false);
            (manipulation_.rtde)->force_target(false, free_axis , wrench, 1.0);
            ROS_INFO("Force Mode deactivated");

            ros::Duration(0.5).sleep();
            geometry_msgs::TransformStampedConstPtr pcp_pose_;
            pcp_pose_ = ros::topic::waitForMessage<geometry_msgs::TransformStamped>("/tcp_pose", ros::Duration(2.0));

            array7d target_2 = {pcp_pose_->transform.translation.x, pcp_pose_->transform.translation.y, pcp_pose_->transform.translation.z + 0.002, 
                     pcp_pose_->transform.rotation.x, pcp_pose_->transform.rotation.y, pcp_pose_->transform.rotation.z, 
                     pcp_pose_->transform.rotation.w};
            (manipulation_.rtde)->cart_target(1, target_2, manipulation_.jnt_vel_*0.2, manipulation_.jnt_acc_*0.2);
                
            ros::Duration(0.5).sleep();
            if(manipulation_.get_request().mode == "PICK")
            {
                (manipulation_.rtde)->gripper_close(manipulation_.gripper_speed_, manipulation_.gripper_force_);
            }
            if(manipulation_.get_request().mode == "PLACE")
            {
                (manipulation_.rtde)->gripper_open(manipulation_.gripper_speed_, manipulation_.gripper_force_);
            }           
            return BT::NodeStatus::SUCCESS;
        }
};

class MoveUp : public BT::SyncActionNode
{
    private:
        Manipulation& manipulation_;

    public:
        MoveUp(const std::string& name, Manipulation& manipulation) : BT::SyncActionNode(name, {}), manipulation_(manipulation) {}
        ~MoveUp() override = default;      
        std::vector<std::pair<std::string, array6d>> areaTargets = {
                {"left_left", manipulation_.array_pick_left_left},
                {"left", manipulation_.array_pick_left},
                {"mid", manipulation_.array_pick_mid},
                {"right", manipulation_.array_pick_right},
                {"right_right", manipulation_.array_pick_right_right}
        };
        virtual BT::NodeStatus tick() override
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
};

class DropObjectInTray : public BT::SyncActionNode
{
    private:
        Manipulation& manipulation_;

    public:
        DropObjectInTray(const std::string& name, Manipulation& manipulation) : BT::SyncActionNode(name, {}), manipulation_(manipulation) {}
        ~DropObjectInTray() override = default;     
        std::vector<Tray> trays = {
            {manipulation_.array_tray1_top, manipulation_.array_tray1_load, "SAVE_1", objects_in_trays[0]},
            {manipulation_.array_tray2_top, manipulation_.array_tray2_load, "SAVE_2", objects_in_trays[1]},
            {manipulation_.array_tray3_top, manipulation_.array_tray3_load, "SAVE_3", objects_in_trays[2]}
        };
        virtual BT::NodeStatus tick() override
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
};

class MoveHomePos : public BT::SyncActionNode
{
    private:
        Manipulation& manipulation_;

    public:
        MoveHomePos(const std::string& name, Manipulation& manipulation) : BT::SyncActionNode(name, {}), manipulation_(manipulation) {}
        ~MoveHomePos() override = default;      
        virtual BT::NodeStatus tick() override
        {
            ROS_INFO("move home pos");
            if(manipulation_.get_last_pos() == "tray")
            {
                (manipulation_.rtde)->joint_target(manipulation_.array_rotate2, manipulation_.jnt_vel_, manipulation_.jnt_acc_);
                (manipulation_.rtde)->joint_target(manipulation_.array_rotate1, manipulation_.jnt_vel_, manipulation_.jnt_acc_);
                (manipulation_.rtde)->joint_target(manipulation_.array_scan_mid, manipulation_.jnt_vel_, manipulation_.jnt_acc_);
                manipulation_.res_.status = "FINISHED";
                return BT::NodeStatus::SUCCESS; 
            }
            else
            {  
                (manipulation_.rtde)->joint_target(manipulation_.array_scan_mid, manipulation_.jnt_vel_, manipulation_.jnt_acc_);
                manipulation_.res_.status = "FINISHED";
                return BT::NodeStatus::SUCCESS;
            }
        }
};


#endif 