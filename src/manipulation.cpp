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
    target_position = {2.40435886383057, -1.83808960537099, 0.975212875996725, -0.674065129165985, -1.63826924959292, -3.8627772966968};
}

/**
 *      @brief Destructor of class Manipulation.
 */

Manipulation::~Manipulation()
{

}

// Member functions -------------------------------------------

/**
 *      @brief Initialize the Manipulation object by setting up ROS components.
 */

void Manipulation::initialize()
{
    rtde = std::make_unique<URRTDE>(nh_);
    service_server = nh_.advertiseService("SwotManipulationBT", &Manipulation::callback_service_manipulation, this);
    service_client_matching = nh_.serviceClient<swot_msgs::SwotObjectMatching2023>("ObjectMatchingServer");
    service_client_free = nh_.serviceClient<swot_msgs::SwotFreeSpot>("FreeSpotServer");
    sub_wrench = nh_.subscribe("wrench", 1000, &Manipulation::callback_wrench, this);
    ROS_INFO("ROS service started"); 
}

/**
 *      @brief Register the custom nodes for the behavior tree.
 *      @param factory The behavior tree factory object.
 *      @param manipulation The Manipulation class object.
 */

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

/**
 *       @brief Callback function for the Manipulation service.
 *
 *      This function is called when the Manipulation service is requested. It sets the request and response objects,
 *      prints some request information, opens the gripper, registers nodes in the behavior tree factory, creates a behavior
 *      tree from an XML file, and executes the behavior tree.
 *
 *      @param req The Manipulation service request.
 *      @param res The Manipulation service response.
 *      @return True if the service is successfully executed.
 *
*/

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

/**
 *      @brief Callback function for the wrench topic.
 *
 *      This function is called when a message is received on the wrench topic. It checks if collision detection is activated,
 *      and if the maximum force threshold is exceeded, it sets the collision flag and deactivates collision detection.
 *
 *      @param msg The wrench message received on the wrench topic.
 */

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

/**
 *      @brief Sends the target position (6D) to the robot.
 *          This function sends the target position (6D) to the robot using the RTDE interface.
 */

void Manipulation::sendTargetPosition6d()
{
    rtde->joint_target(target_position, jnt_vel_, jnt_acc_);
}
 
/**
 *      @brief Sets the top position of the tray.
 *
 *      This function sets the top position of the tray based on the current tray selection. It uses the RTDE interface to
 *      send the joint target.
 */

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

/**
 *      @brief Sets the last position.
 *      @param last_pose The last position to set.
 */

void Manipulation::set_last_pos(std::string last_pose)
{
    this->last_pos = last_pose;
}

/**
 *      @brief Sets the grasping area.
 *      @param grasping_area The grasping area to set.
 */

void Manipulation::set_grasping_area(std::string grasping_area)
{
    this->grasping_area = grasping_area;
}

/**
 *      @brief Sets the collision flag.
 *      @param collision The collision flag value to set.
 */

void Manipulation::set_collision_detected(bool collision)
{
    this->collision_detected = collision;
}

/**
 *      @brief Sets the collision activation flag.
 *      @param collision The collision activation flag value to set.
 */

void Manipulation::set_collision_activated(bool collision)
{
    this->collision_activated = collision;
}

/**
 *      @brief Sets the grasping point.
 *      @param grasping The grasping point to set.
 */

void Manipulation::set_grasping_point(geometry_msgs::Pose grasping)
{
    this->grasping_point = grasping;
}

/**
 *      @brief Sets the current tray.
 *      @param tray The tray to set.
 */

void Manipulation::set_tray(std::string tray)
{
    this->tray = tray;
}

/**
 *      @brief Sets the object placed in the specified tray.
 *      @param object The object to set.
 *      @param tray_number The tray number where the object is placed.
 */

void Manipulation::set_object_in_trays(std::string object, int tray_number)
{
    this->objects_in_trays[tray_number] = object;
}

/**
 *      @brief Resets the object placed in the specified tray.
 *      @param tray_number The tray number to reset.
 */

void Manipulation::rest_object_in_trays(int tray_number)
{
    this->objects_in_trays[tray_number] = "";
}

/**
 *      @brief Sets the response status.
 *      @param status The response status to set.
 */

void Manipulation::set_response_status(const std::string& status)
{
    this->res_.status = status;
}

/**
 *      @brief Sets the target position (6D).
 *      @param target The target position (6D) to set.
 */

void Manipulation::setTargetPosition6d(std::string target)
{
    std::string csvFilePath = "../csv/target_positions.csv";  // Path to the CSV file

    std::ifstream csvFile(csvFilePath);
    if (!csvFile.is_open()) {
        std::cerr << "Failed to open CSV file: " << csvFilePath << std::endl;
        return;
    }

    std::string line;
    while (std::getline(csvFile, line)) {
        std::istringstream iss(line);
        std::string columnName;
        std::getline(iss, columnName, ',');

        if (columnName == target) {

            // Extract values from each column and assign them to the targetValues array
            std::string columnValue;
            for (int i = 0; i < target_position.size(); ++i) {
                std::getline(iss, columnValue, ',');
                target_position[i] = std::stod(columnValue);
            }

            break;  // Found the target, so no need to continue searching
        }
    }

    csvFile.close();
}

// Getter functions -------------------------------------------

/**
 *      @brief Gets the last position.
 *      @return The last position.
 */

std::string Manipulation::get_last_pos() const
{
    return this->last_pos;
}

/**
 *      @brief Gets the grasping area.
 *      @return The grasping area.
 */

std::string Manipulation::get_grasping_area() const
{
    return this->grasping_area;
}

/**
 *      @brief Gets the collision flag.
 *      @return The collision flag.
 */

bool Manipulation::get_collision_detected() const
{
    return this->collision_detected;
}

/**
 *      @brief Gets the collision activation flag.
 *      @return The collision activation flag.
 */

bool Manipulation::get_collision_activated() const
{
    return this->collision_activated;
}

/**
 *      @brief Gets the ROS node handle.
 *      @return The ROS node handle.
 */

ros::NodeHandle Manipulation::get_nh()
{
    return this->nh_;
}

/**
 *      @brief Gets the grasping point.
 *      @return The grasping point.
 */

geometry_msgs::Pose Manipulation::get_grasping_point() const
{
    return this->grasping_point;
}

/**
 *      @brief Gets the object placed in the specified tray.
 *      @param tray_number The tray number.
 *      @return The object placed in the tray.
 */
 
std::string Manipulation::get_object_in_tray(int tray_number) const
{
    return this->objects_in_trays[tray_number];
}

/**
 *      @brief Gets the current tray.
 *      @return The current tray.
 */

std::string Manipulation::get_tray() const
{
    return this->tray;
}

/**
 *      @brief Gets the matching service client.
 *      @return The matching service client.
 */

ros::ServiceClient Manipulation::get_service_client_matching() const
{
    return this->service_client_matching;
}

/**
 *      @brief Gets the free service client.
 *      @return The free service client.
 */

ros::ServiceClient Manipulation::get_service_client_free() const
{
    return this->service_client_free;
}

/**
 *      @brief Gets the Manipulation service request.
 *      @return The Manipulation service request.
 */

swot_msgs::SwotManipulation::Request Manipulation::get_request() const
{
    return this->req_;
}

/**
 *      @brief Gets the Manipulation service response.
 *      @return The Manipulation service response.
 */

swot_msgs::SwotManipulation::Response Manipulation::get_response() const
{
    return this->res_;
}

/**
 *      @brief Gets the gripper speed.
 *      @return The gripper speed.
 */

double Manipulation::get_gripper_speed_() const
{
    return this->gripper_speed_;
}

/**
 *      @brief Gets the gripper force.
 *      @return The gripper force.
 */

double Manipulation::get_gripper_force_() const
{
    return this->gripper_force_;
}

/**
 *      @brief Gets the joint velocity.
 *      @return The joint velocity.
 */

double Manipulation::get_jnt_vel_() const
{
    return this->jnt_vel_;
}

/**
 *      @brief Gets the joint acceleration.
 *      @return The joint acceleration.
 */

double Manipulation::get_jnt_acc_() const
{
    return this->jnt_acc_;
}

/**
 *      @brief Gets the left-left threshold.
 *      @return The left-left threshold.
 */

double Manipulation::get_left_left_thresh() const
{
    return this->left_left_thresh;
}

/**
 *      @brief Gets the left threshold.
 *      @return The left threshold.
 */

double Manipulation::get_left_thresh() const
{
    return this->left_thresh;
}

/**
 *      @brief Gets the right threshold.
 *      @return The right threshold.
 */

double Manipulation::get_right_thresh() const
{
    return this->right_thresh;
}

/**
 *      @brief Gets the right-right threshold.
 *      @return The right-right threshold.
 */
 
double Manipulation::get_right_right_thresh() const
{
    return this->right_right_thresh;
}

/**
 *      @brief Gets the RTDE interface.
 *      @return The RTDE interface.
 */

std::unique_ptr<RTDEControlInterface> Manipulation::get_rtde() const
{
    return this->rtde;
}