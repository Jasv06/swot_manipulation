/**
*       shared_classes.h
*
*       @date 14.07.2023
*       @author Joel Santos
*/

#ifndef SHARED_POSES_H
#define SHARED_POSES_H

#include <behaviortree_cpp_v3/bt_factory.h>
#include <behaviortree_cpp_v3/action_node.h>

/**
 *      @class GetGraspAndMoveGrasp
 *      @brief A synchronized action node for getting the grasping and moving point.
 *      @details This class handles the logic for obtaining the grasping point and moving the robot 
 *          arm to the desired position.
 */

class GetGraspAndMoveGrasp : public BT::SyncActionNode
{
    private:
        Manipulation& manipulation_;
        std::vector<ConditionAction> conditionActions;

    public:
        GetGraspAndMoveGrasp(const std::string& name, Manipulation& manipulation);
        ~GetGraspAndMoveGrasp() override; 
        virtual BT::NodeStatus tick() override;
};

/**
 *      @class PickPlaceObject
 *      @brief A synchronized action node for picking up or placing an object.
 *      @details This class handles the logic for picking up an object or placing it in the grasping point.
 */

class PickPlaceObject : public BT::SyncActionNode
{
    private:
        Manipulation& manipulation_;

    public:
        PickPlaceObject(const std::string& name, Manipulation& manipulation);
        ~PickPlaceObject() override;      
        virtual BT::NodeStatus tick() override;
    
};

/**
 *      @class MoveHomePos
 *      @brief A synchronized action node for moving to the home position.
 *      @details This class handles the logic for moving the robot to the home position depending on the 
 *          last position of the robot.
 */

class MoveHomePos : public BT::SyncActionNode
{
    private:
        Manipulation& manipulation_;

    public:
        MoveHomePos(const std::string& name, Manipulation& manipulation);
        ~MoveHomePos() override;      
        BT::NodeStatus tick() override;
};

/**
 *      @class MoveToDrivePose
 *      @brief A synchronized action node for moving to the drive pose.
 *      @details This class handles the logic for moving the robot to the drive pose which is one the
 *          main tasks to be perfomed.
 */

class MoveToDrivePose : public BT::SyncActionNode
{
    private:
        Manipulation& manipulation_;

    public:
        MoveToDrivePose(const std::string& name, Manipulation& manipulation);
        ~MoveToDrivePose() override;      
        BT::NodeStatus tick() override;
};

#endif