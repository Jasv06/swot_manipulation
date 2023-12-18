/**
*       manipulation.cpp
*
*       @date 19.09.2023
*       @author Joel Santos
*/

#include "swot_manipulation/manipulation.h"
#include "swot_manipulation/pick_classes.h"
#include "swot_manipulation/place_classes.h"
#include "swot_manipulation/shared_classes.h"


/**
 *      @brief Constructor of class Manipulation used to initialize the corresponding member variables.
 */

Manipulation::Manipulation() : last_pos("drive"), grasping_area("mid"), wrench_limit(10.5), collision_detected(false), collision_activated(false), gripper_speed_(1.0), gripper_force_(60.0), jnt_vel_(1), jnt_acc_(1), left_left_thresh(0.2), left_thresh(0.1), right_thresh(-0.1), right_right_thresh(-0.2), count(0)
{
    target_position = {2.7868363857269287, -1.927878042260641, 1.708729092274801, -1.4058648657849808, -1.6280115286456507, -3.468132559453146};
    ws_dim = {0.15, 0.50, -0.45, 0.45};
    ws_height = 10;
    ws_name = "WS01";
    ws_type = "WS";
    obj_mani_height = 0.002;
    obj_name = "M20";
    task_count = 0;
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
    setTargetPosition6d();
    get_mani_height(); 
    get_workspace_dimension_matching(); 
    get_workspace_dimension_free(); 
}

/**
 *      @brief Register the custom nodes for the behavior tree.
 *      @param factory The behavior tree factory object.
 *      @param manipulation The Manipulation class object.
 */

void Manipulation::registerNodes(BT::BehaviorTreeFactory& factory, Manipulation& manipulation)
{
    BT::NodeBuilder builder_1 = [&](const std::string& name, const BT::NodeConfiguration& config) {return std::make_unique<CheckObjRequired>(name,  std::ref(manipulation));};
    factory.registerBuilder<CheckObjRequired>("CheckObjRequired", builder_1);

    BT::NodeBuilder builder_2 = [&](const std::string& name, const BT::NodeConfiguration& config) {return std::make_unique<CheckWSFree>(name, std::ref(manipulation));};
    factory.registerBuilder<CheckWSFree>("CheckWSFree", builder_2);

    BT::NodeBuilder builder_3 = [&](const std::string& name, const BT::NodeConfiguration& config) {return std::make_unique<DropObjectInTray>(name,  std::ref(manipulation));};
    factory.registerBuilder<DropObjectInTray>("DropObjectInTray", builder_3);

    BT::NodeBuilder builder_4 = [&](const std::string& name, const BT::NodeConfiguration& config) {return std::make_unique<GetGraspAndMoveGrasp>(name,  std::ref(manipulation));};
    factory.registerBuilder<GetGraspAndMoveGrasp>("GetGraspAndMoveGrasp", builder_4);

    BT::NodeBuilder builder_5 = [&](const std::string& name, const BT::NodeConfiguration& config) {return std::make_unique<MoveHomePos>(name,  std::ref(manipulation));};
    factory.registerBuilder<MoveHomePos>("MoveHomePos", builder_5);

    BT::NodeBuilder builder_6 = [&](const std::string& name, const BT::NodeConfiguration& config) {return std::make_unique<MoveToDrivePose>(name,  std::ref(manipulation));};
    factory.registerBuilder<MoveToDrivePose>("MoveToDrivePose", builder_6);

    BT::NodeBuilder builder_7 = [&](const std::string& name, const BT::NodeConfiguration& config) {return std::make_unique<MoveToDropPos>(name,  std::ref(manipulation));};
    factory.registerBuilder<MoveToDropPos>("MoveToDropPos", builder_7); 
        
    BT::NodeBuilder builder_8 = [&](const std::string& name, const BT::NodeConfiguration& config) {return std::make_unique<MoveToScan>(name,  std::ref(manipulation));};
    factory.registerBuilder<MoveToScan>("MoveToScan", builder_8);

    BT::NodeBuilder builder_9 = [&](const std::string& name, const BT::NodeConfiguration& config) {return std::make_unique<MoveToTray>(name,  std::ref(manipulation));};
    factory.registerBuilder<MoveToTray>("MoveToTray", builder_9);

    BT::NodeBuilder builder_10 = [&](const std::string& name, const BT::NodeConfiguration& config) {return std::make_unique<MoveUp>(name,  std::ref(manipulation));};       
    factory.registerBuilder<MoveUp>("MoveUp", builder_10);

    BT::NodeBuilder builder_11 = [&](const std::string& name, const BT::NodeConfiguration& config) {return std::make_unique<PickFromTray>(name,  std::ref(manipulation));};
    factory.registerBuilder<PickFromTray>("PickFromTray", builder_11);

    BT::NodeBuilder builder_12 = [&](const std::string& name, const BT::NodeConfiguration& config) {return std::make_unique<PickPlaceObject>(name,  std::ref(manipulation));};
    factory.registerBuilder<PickPlaceObject>("PickPlaceObject", builder_12); 

    BT::NodeBuilder builder_13 = [&](const std::string& name, const BT::NodeConfiguration& config) {return std::make_unique<ScanWorkSpace>(name,  std::ref(manipulation));};       
    factory.registerBuilder<ScanWorkSpace>("ScanWorkSpace", builder_13);
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

bool Manipulation::callback_service_manipulation(swot_msgs::SwotManipulations::Request& req, swot_msgs::SwotManipulations::Response& res)
{
    swot_msgs::SwotObjectPose defaultpose;
    
    int size_of_req = sizeof(req.tasks)/sizeof(req.tasks[0]);

    std::cout << size_of_req << std::endl;

    for(size_t i = 0; i < size_of_req; i++)
    {
        swot_msgs::SwotManipulations::Request mani;

        for(size_t j = 0; i != j; j++)
        {
            mani.tasks[j] = req.tasks[j];
        }

        req_array_.push_back(mani);
    }

    /*The print statements below can be deleted*/
    std::cout << req_array_[0].tasks[get_task_count()].object << std::endl;
    std::cout << req_array_[1].tasks[get_task_count()].object << std::endl;

    getTaskTrack().resize(get_request_vector().size());
    for(auto i = 0; i < getTaskTrack().size(); i++)
    {
        getTaskTrack()[i] = "UNKNOWN";
    }

    for(auto i = 0; i < get_request_vector().size(); i++)
    {
        if(req_array_[i].tasks[get_task_count()].mode == "PICK")
        {
            pick_tracker.push_back(std::make_pair(std::to_string(i) + "0" + req_array_[i].tasks[get_task_count()].object, defaultpose));
        }
        if(req_array_[i].tasks[get_task_count()].mode == "PLACE")
        {
            place_tracker.push_back(std::make_pair(std::to_string(i) + "0" + req_array_[i].tasks[get_task_count()].object, defaultpose));
        }
    }
    rtde->gripper_open(gripper_speed_, gripper_force_);
    BT::BehaviorTreeFactory factory;
    registerNodes(factory, *this);   
    nh_.param<std::string>("file", xml_file,"/home/irobot/catkin_ws/src/swot_manipulation/xml_structure/swot_manipulation.xml");
    auto tree = factory.createTreeFromFile(xml_file);
    tree.tickOnce();
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

void Manipulation::sendTargetPosition6d(std::string target_point)
{
    for(int i = 0; i < 26; i++)
    {
        if(manipulation_poses.position_names[i] == target_point)
        {
            for(int j = 0; j < 6; j++)
            {
                target_position[j] = manipulation_poses.positions[i][j];
            }
            break;
        }
    }
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
        sendTargetPosition6d("array_tray1_top");
    }
    else if (get_tray() == "SAVE_2")
    {
        sendTargetPosition6d("array_tray2_top");
    }
    else
    {
        sendTargetPosition6d("array_tray3_top");
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
 *      @brief Sets the current tray.
 *      @param tray The tray to set.
 */

void Manipulation::set_tray(std::string tray)
{
    this->tray = tray;
}

/**
 *      @brief Sets the response status.
 *      @param status The response status to set.
 */

void Manipulation::set_response_status(std::string status)
{
    this->res_.status = status;
}

/**
 *      @brief Sets the target position (6D).
 *      @param target The target position (6D) to set.
 */

void Manipulation::setTargetPosition6d()
{
    std::string csvFilePath = "../csv_files/target_positions.csv";  // Path to the CSV file

    std::ifstream csvFile(csvFilePath);
    if (!csvFile.is_open()) {
        std::cerr << "Failed to open CSV file: " << csvFilePath << std::endl;
        return;
    }

    std::string line;
    // Read the CSV file line by line
    while (std::getline(csvFile, line)) {
        std::istringstream iss(line);
        std::string token;
        array6d rowData;
        
        // Read the first column (string) into position_names
        std::getline(iss, token, ',');
        manipulation_poses.position_names.push_back(token);

        // Read the remaining columns (values) into positions array
        for (int i = 0; i < 6; ++i) {
            std::getline(iss, token, ',');
            rowData[i] = std::stod(token);
        }

        manipulation_poses.positions.push_back(rowData);
    }
    std::cerr << "Value not found in the CSV file." << std::endl;
    csvFile.close();

    /*DELETE PRINT STATEMENTS BELOW*/

    for (int i = 0; i < 6; i++) {
        std::cout << manipulation_poses.position_names[i] << std::endl;
        for(int j = 0; j < 6; j++)
        {
            std::cout << manipulation_poses.positions[i][j] << std::endl;
        }
    }
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

void Manipulation::get_mani_height()
{
    std::string csvFilePath = "../csv_files/manipulation_height.csv";  // Path to the CSV file

    std::ifstream csvFile(csvFilePath);
    if (!csvFile.is_open()) {
        std::cerr << "Failed to open CSV file: " << csvFilePath << std::endl;
        return;
    }

    std::string line;
    // Read the CSV file line by line
    while (std::getline(csvFile, line)) {
        std::istringstream iss(line);
        std::string token;
        double data;
        
        // Read the first column (string) into object_names
        std::getline(iss, token, ',');
        manipulation_height_object.object_names.push_back(token);

        std::getline(iss, token, ',');
        data = std::stod(token);
        manipulation_height_object.manipulation_heights.push_back(data);
    }
    csvFile.close();

    /*DELETE PRINT STATEMENTS BELOW*/

    for (int i = 0; i < 6; i++) {
        std::cout << manipulation_height_object.object_names[i] << std::endl;
        std::cout << manipulation_height_object.manipulation_heights[i] << std::endl;
        
    }
    return;
}

void Manipulation::get_workspace_dimension_matching()
{
    std::string csvFilePath;
    csvFilePath = "../csv_files/workspace_dimensions_matching.csv";  // Path to the CSV file
    
    std::ifstream csvFile(csvFilePath);
    if (!csvFile.is_open()) {
        std::cerr << "Failed to open CSV file: " << csvFilePath << std::endl;
        return;
    }

    std::string line;
    // Read the CSV file line by line
    while (std::getline(csvFile, line)) {
        std::istringstream iss(line);
        std::string token;
        
        // Read the first column (string) into object_names
        std::getline(iss, token, ',');
        workspace_dimensions_matching_object.workspace_number.push_back(token);

        //Read the remaining columns into workspace dimensions
        std::array<double, 5> dimensions;
        for(int i = 0; i < 5; i++)
        {
            std::getline(iss, token, ',');
            dimensions = std::stod(token);
        }
        workspace_dimensions_matching_object.workspace_dimensions.push_back(dimensions);
    }
    csvFile.close();

    /*DELETE PRINT STATEMENTS BELOW*/

    for (int i = 0; i < 6; i++) {
        std::cout << workspace_dimensions_matching_object.workspace_number[i] << std::endl;
        for(int j = 0; j < 5; j++)
        {
            std::cout << workspace_dimensions_matching_object.workspace_dimensions[i][j] << std::endl;
        }  
    }
    return;
}   

void Manipulation::get_workspace_dimension_free()
{
    std::string csvFilePath;
    csvFilePath = "../csv_files/workspace_dimensions_free.csv";  // Path to the CSV file

    std::ifstream csvFile(csvFilePath);
    if (!csvFile.is_open()) {
        std::cerr << "Failed to open CSV file: " << csvFilePath << std::endl;
        return;
    }

    std::string line;
    // Read the CSV file line by line
    while (std::getline(csvFile, line)) {
        std::istringstream iss(line);
        std::string token;
        
        // Read the first column (string) into object_names
        std::getline(iss, token, ',');
        workspace_dimensions_free_object.workspace_number.push_back(token);

        //Read the remaining columns into workspace dimensions
        std::array<double, 5> dimensions;
        for(int i = 0; i < 5; i++)
        {
            std::getline(iss, token, ',');
            dimensions = std::stod(token);
        }
        workspace_dimensions_free_object.workspace_dimensions.push_back(dimensions);
    }
    csvFile.close();

    /*DELETE PRINT STATEMENTS BELOW*/

    for (int i = 0; i < 6; i++) {
        std::cout << workspace_dimensions_free_object.workspace_number[i] << std::endl;
        for(int j = 0; j < 5; j++)
        {
            std::cout << workspace_dimensions_free_object.workspace_dimensions[i][j] << std::endl;
        }  
    }
    return;
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

const std::vector<swot_msgs::SwotManipulations::Request>& Manipulation::get_request_vector() const {
    return this->req_array_; 
}

/**
 *      @brief Gets the Manipulation service request.
 *      @return The Manipulation service request.
 */

const swot_msgs::SwotManipulations::Request& Manipulation::get_request(int index) const
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

swot_msgs::SwotManipulations::Response& Manipulation::get_response()
{
    return res_;
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

int& Manipulation::get_ws_height()
{
    return this->ws_height;
}

std::string& Manipulation::get_ws_name()
{
    return this->ws_name;
}

std::string& Manipulation::get_ws_type()
{
    return this->ws_type;
}

std::string& Manipulation::get_obj_name()
{
    return this->obj_name;
}

double& Manipulation::get_obj_mani_height()
{
    return this->obj_mani_height;
}

array4d& Manipulation::get_ws_dim()
{
    return this->ws_dim;
}

std::vector<std::string>& Manipulation::getTaskTrack()
{
    return this->task_track;
}

std::vector<std::pair<std::string, swot_msgs::SwotObjectPose>>& Manipulation::getPickTracker()
{
    return this->pick_tracker;
}
std::vector<std::pair<std::string, swot_msgs::SwotObjectPose>>& Manipulation::getPlaceTracker()
{
    return this->place_tracker;
}

int& Manipulation::get_task_count()
{
    return this->task_count;
}

ManipulationHeight Manipulation::get_manipulation_height_object()
{
    return this->manipulation_height_object;
}

Positions Manipulation::get_manipulation_poses()
{
    return this->manipulation_poses;
}

WorkSpaceDimensionsFree Manipulation::get_workspace_dimensions_free_object()
{
    return this->workspace_dimensions_free_object;
}

WorkSpaceDimensionsMatching Manipulation::get_workspace_dimensions_matching_object()
{
    return this->workspace_dimensions_matching_object;
}

int Manipulation::index(std::string ws_name)
{
    for(int i = 1; i <= 16; i++)
    {
        if(workspace_dimensions_matching_object.workspace_dimensions[i][4] == ws_name)
        {
            return i;
        }
    }
    return 0;
}

int Manipulation::index_height(std::string obj_name)
{
    for(int i = 1; i <= 18; i++)
    {
        if(workspace_dimensions_matching_object.workspace_dimensions[i][4] == obj_name)
        {
            return i;
        }
    }
    return 0;
}
