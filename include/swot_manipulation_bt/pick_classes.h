/**
*       pick_classes.h
*
*       @date 14.07.2023
*       @author Joel Santos
*/

#pragma once

#include "swot_manipulation_bt/manipulation.h"

/**
 *      @class MoveToScan
 *      @brief Action node for moving to the scanning position.
 *      @details This action node is responsible for moving the robot to the scanning position.
 */

class MoveToScan : public BT::SyncActionNode
{
    private:
        Manipulation& manipulation_;

    public:
        MoveToScan(const std::string& name, Manipulation& manipulation);
        ~MoveToScan() override;      
        BT::NodeStatus tick() override;            
}; 

/**
 *      @class ScanWorkSpace
 *      @brief Action node for scanning the workspace.
 *      @details This action node is responsible for scanning the workspace for objects.
 */

class ScanWorkSpace : public BT::SyncActionNode
{
    private:
        Manipulation& manipulation_;

    public:
        ScanWorkSpace(const std::string& name, Manipulation& manipulation);
        ~ScanWorkSpace() override;      
        BT::NodeStatus tick() override;
};

/**
 *      @class MoveUp
 *      @brief Action node for moving up after picking an object.
 *      @details This action node is responsible for moving the robot up to a certain position
 *          after picking an object from a workspace.
 */

class MoveUp : public BT::SyncActionNode
{
    private:
        Manipulation& manipulation_;
        std::vector<ConditionAction> conditionActions;

    public:
        MoveUp(const std::string& name, Manipulation& manipulation);
        ~MoveUp() override;      
        BT::NodeStatus tick() override;      
};

/**
 *      @class DropObjectInTray
 *      @brief Action node for dropping an object in a tray.
 *      @details This action node is responsible for dropping an object into a designated tray
 *          based on the requirements of the task to be accomplished. For example, if the object
 *          is required in tray 2 the availability of this tray is check and not only placed.
 */

class DropObjectInTray : public BT::SyncActionNode
{
    private:
        Manipulation& manipulation_;
        std::vector<Tray> trays;

    public:
        DropObjectInTray(const std::string& name, Manipulation& manipulation);
        ~DropObjectInTray() override;     
        BT::NodeStatus tick() override;
}; 
