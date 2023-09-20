/**
*       main.cpp
*
*       @date 14.07.2023
*       @author Joel Santos
*/

#include <iostream>
#include "swot_manipulation/manipulation.h"
#include "swot_manipulation/debug_mode.h"

int main(int argc, char **argv)
{
    // Initialize ROS
    ros::init(argc, argv, "swot_manipulation");

    // Remove current definition of DEBUG_MODE in case any other value has been assigned
    //#ifdef DEBUG_MODE   
    //#undef DEBUG_MODE
    //#endif

    // Create an instance of the Manipulation class
    // This instance calls the default constructor of the class
    Manipulation manipulation;

    // Extract the value of the variable debug which was passed as a argument via the launch file
    // The variable debug is defined in "swot_manipulation/debug_mode.h"
    //manipulation.get_nh().getParam("debug", debug);
    
    // Define DEBUG_MODE based on the debug parameter
    //#define DEBUG_MODE debug

    //#if !DEBUG_MODE
        // Initialize the manipulation class and its ROS services
        manipulation.initialize();
        // Create a MultiThreadedSpinner with 4 threads
        ros::MultiThreadedSpinner spinner(4);
        // Spin the spinner to process callbacks in multiple threads
        spinner.spin();
    //#else
        //std::cout << "Debug mode" << std::endl;
        // Initialize the manipulation class and its ROS services in debug mode
        //manipulation.initialize();
    //#endif

    return 0;
}

