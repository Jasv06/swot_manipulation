/**
*       place.cpp
*
*       @date 14.07.2023
*       @author Joel Santos
*/

NotDrive::NotDrive(const std::string& name, Manipulation& manipulation) : BT::ConditionNode(name, {}), manipulation_(manipulation) 
{

}

NotDrive::~NotDrive() override = default;

BT::NodeStatus NotDrive::tick() override
{
    ROS_INFO("drive achieved");
    if(manipulation_.get_request().mode == "DRIVE") {return BT::NodeStatus::FAILURE;}
    else {return BT::NodeStatus::SUCCESS;}
}

NotPick::NotPick(const std::string& name, Manipulation& manipulation) : BT::ConditionNode(name, {}), manipulation_(manipulation) 
{

}

NotPick::~NotPick() override = default;

BT::NodeStatus NotPick::tick() override
{
    ROS_INFO("pick achieved");
    if(manipulation_.get_request().mode == "PICK") {return BT::NodeStatus::FAILURE;}
    else {return BT::NodeStatus::SUCCESS;}
}

NotPlace(const std::string& name, Manipulation& manipulation) : BT::ConditionNode(name, {}), manipulation_(manipulation) {}
~NotPlace() override = default;

BT::NodeStatus tick() override
{
    ROS_INFO("place achieved");
    if(manipulation_.get_request().mode == "PLACE") {return BT::NodeStatus::FAILURE;}
    else {return BT::NodeStatus::SUCCESS;}
} 