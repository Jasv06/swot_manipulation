/**
*       manipulation.cpp
*
*       @date 14.07.2023
*       @author Joel Santos
*/

#include "swot_manipulation_bt/manipulation.h"
#include "swot_manipulation_bt/condition_classes.h"
#include "swot_manipulation_bt/pick_classes.h"
#include "swot_manipulation_bt/place_classes.h"
#include "swot_manipulation_bt/shared_classes.h"

/**
*       @struct ConditionAction
*       @brief Represents a condition-action pair for handling RoboCup manipulation tasks.
*       @details This struct combines a condition function and an action function. The condition function
*           determines whether the specified condition is met, and the action function is executed
*           if the condition is true.
*/

struct ConditionAction {
    std::function<bool()> condition;        /**< The condition function. */
    std::function<void()> action;           /**< The action function. */
};

/**
*       @struct Tray
*       @brief Represents a tray used for RoboCup manipulation tasks.
*       @details This struct holds information about the top pose, load pose, save position, and tray object
*           associated with a specific tray.
*/

struct Tray {
    array6d topPose;                        /**< The top pose of the tray. */
    array6d loadPose;                       /**< The load pose of the tray. */
    std::string savePosition;               /**< The save position of the tray. */
    std::string& trayObject;                /**< The object placed on the tray. */
};

/**
 *      @brief Constructor of class Manipulation used to initialize the corresponding member variables.
 */

Manipulation::Manipulation() : last_pos("drive"), grasping_area("mid"), wrench_limit(10.5), collision_detected(false), collision_activated(false), gripper_speed_(1.0), gripper_force_(60.0), jnt_vel_(1), jnt_acc_(1), move_duration(5),left_left_thresh(0.2), left_thresh(0.1), right_thresh(-0.1), right_right_thresh(-0.2)
{

}

/**
 *      @brief Destructor of class Manipulation.
 */

Manipulation::~Manipulation()
{

}

// Member functions -------------------------------------------

void Manipulation::initialize()
{
    rtde = std::make_unique<URRTDE>(nh_);
    service_server = nh_.advertiseService("SwotManipulationBT", &Manipulation::callback_service_manipulation, this);
    service_client_matching = nh_.serviceClient<swot_msgs::SwotObjectMatching2023>("ObjectMatchingServer");
    service_client_free = nh_.serviceClient<swot_msgs::SwotFreeSpot>("FreeSpotServer");
    sub_wrench = nh_.subscribe("wrench", 1000, &Manipulation::callback_wrench, this);
    ROS_INFO("ROS service started"); 
}

void Manipulation::registerNodes(BT::BehaviorTreeFactory& factory, Manipulation& manipulation)
{
    BT::NodeBuilder builder_1 = [&](const std::string& name, const BT::NodeConfiguration& config) {return std::make_unique<NotPick>(name,  std::ref(manipulation));};
    factory.registerBuilder<NotPick>("NotPick", builder_1);

    BT::NodeBuilder builder_2 = [&](const std::string& name, const BT::NodeConfiguration& config) {return std::make_unique<NotDrive>(name,  std::ref(manipulation));};
    factory.registerBuilder<NotDrive>("NotDrive", builder_2);

    BT::NodeBuilder builder_3 = [&](const std::string& name, const BT::NodeConfiguration& config) {return std::make_unique<NotPlace>(name,  std::ref(manipulation));};
    factory.registerBuilder<NotPlace>("NotPlace", builder_3);        
        
    BT::NodeBuilder builder_4 = [&](const std::string& name, const BT::NodeConfiguration& config) {return std::make_unique<NotPP>(name,  std::ref(manipulation));};
    factory.registerBuilder<NotPP>("NotPP", builder_4);
        
    BT::NodeBuilder builder_5 = [&](const std::string& name, const BT::NodeConfiguration& config) {return std::make_unique<NotWS>(name,  std::ref(manipulation));};
    factory.registerBuilder<NotWS>("NotWS", builder_5);

    BT::NodeBuilder builder_6 = [&](const std::string& name, const BT::NodeConfiguration& config) {return std::make_unique<NotSH>(name,  std::ref(manipulation));};
    factory.registerBuilder<NotSH>("NotSH", builder_6);

    BT::NodeBuilder builder_7 = [&](const std::string& name, const BT::NodeConfiguration& config) {return std::make_unique<NotTT>(name,  std::ref(manipulation));};
    factory.registerBuilder<NotTT>("NotTT", builder_7);

    BT::NodeBuilder builder_8 = [&](const std::string& name, const BT::NodeConfiguration& config) {return std::make_unique<CheckObjRequired>(name,  std::ref(manipulation));};
    factory.registerBuilder<CheckObjRequired>("CheckObjRequired", builder_8);

    BT::NodeBuilder builder_9 = [&](const std::string& name, const BT::NodeConfiguration& config) {return std::make_unique<CheckWSFree>(name, std::ref(manipulation));};
    factory.registerBuilder<CheckWSFree>("CheckWSFree", builder_9);

    BT::NodeBuilder builder_10 = [&](const std::string& name, const BT::NodeConfiguration& config) {return std::make_unique<DropObjectInTray>(name,  std::ref(manipulation));};
    factory.registerBuilder<DropObjectInTray>("DropObjectInTray", builder_10);

    BT::NodeBuilder builder_11 = [&](const std::string& name, const BT::NodeConfiguration& config) {return std::make_unique<GetGraspAndMoveGrasp>(name,  std::ref(manipulation));};
    factory.registerBuilder<GetGraspAndMoveGrasp>("GetGraspAndMoveGrasp", builder_11);

    BT::NodeBuilder builder_12 = [&](const std::string& name, const BT::NodeConfiguration& config) {return std::make_unique<MoveHomePos>(name,  std::ref(manipulation));};
    factory.registerBuilder<MoveHomePos>("MoveHomePos", builder_12);

    BT::NodeBuilder builder_13 = [&](const std::string& name, const BT::NodeConfiguration& config) {return std::make_unique<MoveToDrivePose>(name,  std::ref(manipulation));};
    factory.registerBuilder<MoveToDrivePose>("MoveToDrivePose", builder_13);

    BT::NodeBuilder builder_14 = [&](const std::string& name, const BT::NodeConfiguration& config) {return std::make_unique<MoveToDropPos>(name,  std::ref(manipulation));};
    factory.registerBuilder<MoveToDropPos>("MoveToDropPos", builder_14); 
        
    BT::NodeBuilder builder_15 = [&](const std::string& name, const BT::NodeConfiguration& config) {return std::make_unique<MoveToScan>(name,  std::ref(manipulation));};
    factory.registerBuilder<MoveToScan>("MoveToScan", builder_15);

    BT::NodeBuilder builder_16 = [&](const std::string& name, const BT::NodeConfiguration& config) {return std::make_unique<MoveToTray>(name,  std::ref(manipulation));};
    factory.registerBuilder<MoveToTray>("MoveToTray", builder_16);

    BT::NodeBuilder builder_17 = [&](const std::string& name, const BT::NodeConfiguration& config) {return std::make_unique<MoveUp>(name,  std::ref(manipulation));};       
    factory.registerBuilder<MoveUp>("MoveUp", builder_17);

    BT::NodeBuilder builder_18 = [&](const std::string& name, const BT::NodeConfiguration& config) {return std::make_unique<PickFromTray>(name,  std::ref(manipulation));};
    factory.registerBuilder<PickFromTray>("PickFromTray", builder_18);

    BT::NodeBuilder builder_19 = [&](const std::string& name, const BT::NodeConfiguration& config) {return std::make_unique<PickObject>(name,  std::ref(manipulation));};
    factory.registerBuilder<PickObject>("PickObject", builder_19); 

    BT::NodeBuilder builder_20 = [&](const std::string& name, const BT::NodeConfiguration& config) {return std::make_unique<ScanWorkSpace>(name,  std::ref(manipulation));};       
    factory.registerBuilder<ScanWorkSpace>("ScanWorkSpace", builder_20);
}

bool Manipulation::callback_service_manipulation(swot_msgs::SwotManipulation::Request &req, swot_msgs::SwotManipulation::Response &res)
{
    this->req_ = req;
    this->res_ = res;

    std::cout << get_request().mode << std::endl;
    std::cout << get_request().object << std::endl;
    std::cout << get_request().save << std::endl;
    std::cout << get_request().task << std::endl;
    rtde->gripper_open(gripper_speed_, gripper_force_);
    BT::BehaviorTreeFactory factory;
    registerNodes(factory, *this);   
    nh_.param<std::string>("file", xml_file,"/home/irobot/catkin_ws/src/swot_manipulation_bt/xml_structure/swot_manipulation.xml");
    auto tree = factory.createTreeFromFile(xml_file);
    tree.tickRoot();
    return true;
}

void Manipulation::callback_wrench(const geometry_msgs::WrenchStamped &msg)
{
    if(collision_activated)
    {
        if (collision_detected == false)
        {
            if (std::max({std::abs(msg.wrench.force.x), std::abs(msg.wrench.force.y), std::abs(msg.wrench.force.z)}) > wrench_limit)
            {
                ROS_INFO("wrench_limit achieved");
                set_collision(true);
                set_collision_activated(false);
            }
        }
    }
}

void Manipulation::sendTargetPosition()
{

}
 
void Manipulation::tray_top()
{
    if (get_tray() == "SAVE_1")
    {
        rtde->joint_target(array_tray1_top, jnt_vel_, jnt_acc_);    
    }
    else if (get_tray() == "SAVE_2")
    {
        rtde->joint_target(array_tray2_top, jnt_vel_, jnt_acc_);          
    }
    else
    {
        rtde->joint_target(array_tray3_top, jnt_vel_, jnt_acc_); 
    }
}

// Setter functions -------------------------------------------

void Manipulation::set_last_pos(std::string last_pose)
{
    this->last_pos = last_pose;
}

void Manipulation::set_grasping_area(std::string grasping_area)
{
    this->grasping_area = grasping_area;
}

void Manipulation::set_collision_detected(bool collision)
{
    this->collision_detected = collision;
}

void Manipulation::set_collision_activated(bool collision)
{
    this->collision_activated = collision;
}

void Manipulation::set_grasping_point(geometry_msgs::Pose grasping)
{
    this->grasping_point = grasping;
}

void Manipulation::set_tray(std::string tray)
{
    this->tray = tray;
}

void Manipulation::set_object_in_trays(std::string object, int tray_number)
{
    this->objects_in_trays[tray_number] = object;
}

void Manipulation::rest_object_in_trays(int tray_number)
{
    this->objects_in_trays[tray_number] = "";
}

void Manipulation::set_response_status(const std::string& status)
{
    this->res_.status = status;
}

void Manipulation::setTargetPosition(std::string target)
{
    
}

// Getter functions -------------------------------------------

std::string Manipulation::get_last_pos() const
{

}

std::string Manipulation::get_grasping_area() const
{

}

bool Manipulation::get_collision() const
{

}

bool Manipulation::get_collision_activated() const
{

}

ros::NodeHandle Manipulation::get_nh()
{

}

geometry_msgs::Pose Manipulation::get_grasping_point() const
{

}

std::string Manipulation::get_object_in_tray(int) const
{

}

std::string Manipulation::get_tray() const
{

}

ros::ServiceClient Manipulation::get_service_client_matching() const
{

}

ros::ServiceClient Manipulation::get_service_client_free() const
{

}

swot_msgs::SwotManipulation::Request Manipulation::get_request() const
{

}

swot_msgs::SwotManipulation::Response Manipulation::get_response() const
{

}

double Manipulation::get_gripper_speed_() const
{

}

double Manipulation::get_gripper_force_() const
{

}   

double Manipulation::get_jnt_vel_() const
{

}

double Manipulation::get_jnt_acc_() const
{

}   

double Manipulation::get_left_left_thresh() const
{

}   

double Manipulation::get_left_thresh() const
{

}   

double Manipulation::get_right_thresh() const
{

}   

double Manipulation::get_right_right_thresh() const
{

}   

array6d Manipulation::get_target_position() const
{

}