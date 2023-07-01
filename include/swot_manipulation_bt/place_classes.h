/**
*       place_classes.h
*
*       @date 14.07.2023
*       @author Joel Santos
*/

#ifndef PLACE_H
#define PLACE_H

/**
 *      @class CheckObjRequired
 *      @brief Synchronous action node for checking which object is required for placing.
 *      @details This synchronous action node checks which object is required for the placing operation.
 */

class CheckObjRequired : public BT::SyncActionNode
{
    private:
        Manipulation& manipulation_;

    public:
        CheckObjRequired(const std::string& name, Manipulation& manipulation);
        ~CheckObjRequired() override;      
        BT::NodeStatus tick() override;   
};

/**
 *      @class CheckWSFree
 *      @brief Synchronous action node for checking if the workspace is free for placing.
 *      @details This synchronous action node checks if the workspace is free for the placing operation.
 */

class CheckWSFree : public BT::SyncActionNode
{
    private:
        Manipulation& manipulation_;

    public:
        CheckWSFree(const std::string& name, Manipulation& manipulation);
        ~CheckWSFree() override;      
        BT::NodeStatus tick() override;
};

/**
 *      @class MoveToTray
 *      @brief Synchronous action node for moving to the designated tray.
 *      @details This synchronous action node moves the robot to the designated tray where the object
 *          required is located.
 */

class MoveToTray : public BT::SyncActionNode
{
    private:
        Manipulation& manipulation_;

    public:
        MoveToTray(const std::string& name, Manipulation& manipulation);
        ~MoveToTray() override;      
        BT::NodeStatus tick() override;
};

/**
 *      @class PickFromTray
 *      @brief Synchronous action node for picking an object from the tray.
 *      @details This synchronous action node picks an object from the tray before placing.
 */

class PickFromTray : public BT::SyncActionNode
{
    private:
        Manipulation& manipulation_;

    public:
        PickFromTray(const std::string& name, Manipulation& manipulation);
        ~PickFromTray() override;      
        BT::NodeStatus tick() override;
};

/**
 *      @class MoveToDropPos
 *      @brief Synchronous action node for moving to the drop position.
 *      @details This synchronous action node moves the robot to the designated drop position for placing the object.
 */

class MoveToDropPos : public BT::SyncActionNode
{
    private:
        Manipulation& manipulation_;

    public:
        MoveToDropPos(const std::string& name, Manipulation& manipulation);
        ~MoveToDropPos() override;      
        BT::NodeStatus tick() override;
};

#endif