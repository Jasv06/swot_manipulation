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
 *      @brief Constructor of class Manipulation used to initialize the corresponding member variables.
 */

Manipulation::Manipulation() : last_pos("drive"), grasping_area("mid"), wrench_limit(10.5), collision_detected(false), collision_activated(false), gripper_speed_(1.0), gripper_force_(60.0), jnt_vel_(1), jnt_acc_(1), left_left_thresh(0.2), left_thresh(0.1), right_thresh(-0.1), right_right_thresh(-0.2), count(0)
{
    target_position = {2.40435886383057, -1.83808960537099, 0.975212875996725, -0.674065129165985, -1.63826924959292, -3.8627772966968};
    ws_dim = {0.15, 0.50, -0.45, 0.45};
    ws_height = 10;
    ws_name = "WS01";
    ws_type = "WS";
    obj_mani_height = 0.002;
    obj_name = "M20";
    getTaskTrack().rezise(get_request_vector().size());
    for(auto i = 0; i < getTaskTrack().size(); i++)
    {
        getTaskTrack()[i] = "UNKNOWN";
    }
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
    service_client_matching = nh_.serviceClient<swot_msgs::SwotObjectMatching>("ObjectMatchingServer");
    service_client_free = nh_.serviceClient<swot_msgs::SwotFreeSpot>("FreeSpotServer");
    sub_wrench = nh_.subscribe("wrench", 1000, &Manipulation::callback_wrench, this);
    ROS_INFO("ROS service started"); 
    std::cout << "Current gripper position: " << rtde.get_gripper_position_per() << std::endl;
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

    BT::NodeBuilder builder_19 = [&](const std::string& name, const BT::NodeConfiguration& config) {return std::make_unique<PickPlaceObject>(name,  std::ref(manipulation));};
    factory.registerBuilder<PickPlaceObject>("PickPlaceObject", builder_19); 

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
    this->req_array_ = req;
    this->res_array_ = res;

    
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
                set_collision_detected(true);
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
        setTargetPosition6d("array_tray1_top"); sendTargetPosition6d();
    }
    else if (get_tray() == "SAVE_2")
    {
        setTargetPosition6d("array_tray2_top"); sendTargetPosition6d();
    }
    else
    {
        setTargetPosition6d("array_tray3_top"); sendTargetPosition6d();
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
 *      @param tray_number The tray number where the object is placed. It can be 0,1 or 2.
 */

void Manipulation::set_object_in_trays(std::string object, int tray_number)
{
    this->objects_in_trays[tray_number] = object;
}

/**
 *      @brief Resets the object placed in the specified tray.
 *      @param tray_number The tray number to reset.
 */

void Manipulation::reset_object_in_trays(int tray_number)
{
    this->objects_in_trays[tray_number] = "";
}

/**
 *      @brief Sets the response status.
 *      @param status The response status to set.
 */

void Manipulation::set_response_status(const std::string& status, int index)
{
    // Check if index is within bounds
    if (index < res_array_.size()) {
        res_array_[index] = newResponse;
    } else {
        throw std::out_of_range("Index out of bounds");
    }
}

/**
 *      @brief Sets the target position (6D).
 *      @param target The target position (6D) to set.
 */

void Manipulation::setTargetPosition6d(std::string target)
{
    std::string csvFilePath = "../csv_files/target_positions.csv";  // Path to the CSV file

    std::ifstream csvFile(csvFilePath);
    if (!csvFile.is_open()) {
        std::cerr << "Failed to open CSV file: " << csvFilePath << std::endl;
        return;
    }

    std::string line;
    while (std::getline(csvFile, line)) {
        std::istringstream linestream(line);
        std::string col1, col2, col3, col4, col5, col6, col7;

        if (std::getline(linestream, col1, ',') &&
            std::getline(linestream, col2, ',') &&
            std::getline(linestream, col3, ',') &&
            std::getline(linestream, col4, ',') &&
            std::getline(linestream, col5, ',') &&
            std::getline(linestream, col6, ',') &&
            std::getline(linestream, col7)) {
            if (col1 == target) {
                target_position[0] = col2;
                target_position[1] = col3;
                target_position[2] = col4;
                target_position[3] = col5;
                target_position[4] = col6;
                target_position[5] = col7;
                csvFile.close(); // Close the file before returning
                return;
            }
        }
    }
     // If we reach here, it means searchValue was not found in the CSV file.
    std::cerr << "Value not found in the CSV file." << std::endl;
    csvFile.close();
}
                  
void Manipulation::set_target(array6d target)
{
    this->target_position = target;
}

void Manipulation::set_count(int number)
{
    this->count = number;
}

void Manipulation::increment_count()
{
    this->count++;
}

void Manipulation::get_mani_height(const std::string& name_of_the_object)
{
    std::string csvFilePath = "../csv_files/manipulation_height.csv";  // Path to the CSV file

    std::ifstream csvFile(csvFilePath);
    if (!csvFile.is_open()) {
        std::cerr << "Failed to open CSV file: " << csvFilePath << std::endl;
        return;
    }

    std::string line;
    while (std::getline(csvFile, line)) {
        std::istringstream linestream(line);
        std::string col1, col2;

        if (std::getline(linestream, col1, ',') &&
            std::getline(linestream, col2)) {
            if (col1 == get_request().object) {
                obj_mani_height = std::stod(col2);
                csvFile.close(); // Close the file before returning
                return;
            }
        }
    }

    // If we reach here, it means searchValue was not found in the CSV file.
    std::cerr << "Value not found in the CSV file." << std::endl;
    csvFile.close(); // Close the file before returning

}

void Manipulation::get_worksapce_dimension_matching()
{
    std::string csvFilePath;
    if(this->workspace_match_or_free == "MATCHING")
    {
        csvFilePath = "../csv_files/workspace_dimensions_matching.csv";  // Path to the CSV file
    }
    else
    {
        csvFilePath = "../csv_files/workspace_dimensions_free.csv";  // Path to the CSV file
    }

    std::ifstream csvFile(csvFilePath);
    if (!csvFile.is_open()) {
        std::cerr << "Failed to open CSV file: " << csvFilePath << std::endl;
        return;
    }

    std::string line;
    while (std::getline(csvFile, line)) {
        std::istringstream linestream(line);
        std::string col1, col2, col3, col4, col5, col6;

        if (std::getline(linestream, col1, ',') &&
            std::getline(linestream, col2, ',') &&
            std::getline(linestream, col3, ',') &&
            std::getline(linestream, col4, ',') &&
            std::getline(linestream, col5, ',') &&
            std::getline(linestream, col6)) {
            if (col1 == get_request().task) {
                ws_dim[0] = std::stod(col2);
                ws_dim[1] = std::stod(col3);
                ws_dim[2] = std::stod(col4);
                ws_dim[3] = std::stod(col5);
                ws_height = std::stod(col6);
            }
        }
    }
    csvFile.close();
}   

void Manipulation::set_workspace_match_or_free(std::string type)
{
    workspace_match_or_free = type;
}

void Manipulation::set_grasping_point(int index, geometry_msgs::Pose grasping)
{
    try
    {
        geometry_msgs::Pose value = poses.at(index);
        poses[index] = grasping;
    }
    catch(const std::out_of_range& e)
    {
        std::cerr << "Out of range error: " << e.what() << std::endl;
    }
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

const std::vector<swot_msgs::SwotManipulation2023::Request>& Manipulation::get_request_vector() const {
    return this->req_array_; 
}

/**
 *      @brief Gets the Manipulation service request.
 *      @return The Manipulation service request.
 */

const swot_msgs::SwotManipulation2023::Request& Manipulation::get_request(int index) const
{
    // Check if index is within bounds
    if (index < req_array_.size()) {
        return req_array_[index];
    } else {
        throw std::out_of_range("Index out of bounds");
    }
}

/**
 *      @brief Gets the Manipulation service response.
 *      @return The Manipulation service response.
 */

swot_msgs::SwotManipulation2023::Response& Manipulation::get_response(int index)
{
    // Check if index is within bounds
    if (index < res_array_.size()) {
        return res_array_[index];
    } else {
        throw std::out_of_range("Index out of bounds");
    }
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

const std::unique_ptr<URRTDE>& Manipulation::getRTDE() const
{
    return rtde;
}

int Manipulation::get_count() const
{
    return this->count;
}

int Manipulation::get_ws_height() const
{
    return this->ws_height;
}

std::string Manipulation::get_ws_name() const
{
    return this->ws_name;
}

std::string Manipulation::get_ws_type() const
{
    return this->ws_type;
}

std::string Manipulation::get_obj_name() const
{
    return this->obj_name;
}

double Manipulation::get_obj_mani_height() const
{
    return this->obj_mani_height;
}

array4d Manipulation::get_ws_dim() const
{
    return this->ws_dim;
}

std::string Manipulation::get_workspace_match_or_free() const
{
    return this->workspace_match_or_free;
}

geometry_msgs::Pose Manipulation::get_grasping_point_of_index(int index) const
{
    try
    {
        geometry_msgs::Pose value = poses.at(index);
        return poses[index];
    }
    catch(const std::out_of_range& e)
    {
        std::cerr << "Out of range error: " << e.what() << std::endl;
        throw std::runtime_error("Out of range error"); 
    }
}

std::vector<std::string>& Manipulation::getTaskTrack(){
    return this->task_track;
}
