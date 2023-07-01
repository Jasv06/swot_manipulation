/**
*       @file condition_classes.h
*       @date 14.07.2023
*       @author Joel Santos
*       @brief The condition_classes header file is no longer needed in newer versions.
*       @deprecated This header file is deprecated and no longer needed in newer versions such as the branch bt_v4 located in this repository.
*           As an alternative to this header preconditions and postconditions are used. These conditions were introduced in the newer versions 
*           of the behavior tree cpp framework.
*       @note In case of any issues are encountered with this branch do not hesistate in contacting the author at joel.santosvalle@gmail.com
*/

#ifndef CONDITION_CLASSES_H
#define CONDITINO_CLASSES_H

/**
 *      @class NotDrive
 *      @brief Condition node for checking if moving to the driving is not required.
 *      @details This class is a condition node derived from the ConditionNode base class. It checks
 *          if driving is not required in the current context. It performs a conditional task within the
 *          manipulation system by accessing the Manipulation instance.
 */

class NotDrive : public BT::ConditionNode
{
    private:
        Manipulation& manipulation_;

    public:
        NotDrive(const std::string& name, Manipulation& manipulation);
        ~NotDrive() override;
        virtual BT::NodeStatus tick() override;
};

/**
 *      @class NotPick
 *      @brief Condition node for checking if the picking task is not needed.
 *      @details This class is a condition node derived from the ConditionNode base class. It verifies
 *          if picking is not needed in the current context. It performs a conditional task within the
 *          manipulation system by accessing the Manipulation instance.
 */

class NotPick : public BT::ConditionNode
{
    private:
        Manipulation& manipulation_;

    public:
        NotPick(const std::string& name, Manipulation& manipulation);
        ~NotPick() override;
        virtual BT::NodeStatus tick() override;
};

/**
 *      @class NotPlace
 *      @brief Condition node for checking if placing is unnecessary.
 *      @details This class is a condition node derived from the ConditionNode base class. It examines
 *          if placing is unnecessary in the current context. It performs a conditional task within the
 *          manipulation system by accessing the Manipulation instance.
 */

class NotPlace : public BT::ConditionNode
{
    private:
        Manipulation& manipulation_;

    public: 
        NotPlace(const std::string& name, Manipulation& manipulation);
        ~NotPlace() override;
        virtual BT::NodeStatus tick() override;
};

class NotPP : public BT::ConditionNode
{
    private:
        Manipulation& manipulation_;

    public: 
        NotPP(const std::string& name, Manipulation& manipulation);
        ~NotPP() override;
        virtual BT::NodeStatus tick() override;
};

class NotSH : public BT::ConditionNode
{
    private:
        Manipulation& manipulation_;

    public: 
        NotSH(const std::string& name, Manipulation& manipulation);
        ~NotSH() override;
        virtual BT::NodeStatus tick() override;
};

class NotTT : public BT::ConditionNode
{
    private:
        Manipulation& manipulation_;

    public: 
        NotTT(const std::string& name, Manipulation& manipulation);
        ~NotTT() override;
        virtual BT::NodeStatus tick() override;
};

class NotWS : public BT::ConditionNode
{
    private:
        Manipulation& manipulation_;

    public: 
        NotWS(const std::string& name, Manipulation& manipulation);
        ~NotWS() override;
        virtual BT::NodeStatus tick() override;
};

#endif