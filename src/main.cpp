/**
*       main.cpp
*
*       @date 14.07.2023
*       @author Joel Santos
*/

#include <iostream>
#include "swot_manipulation_bt/manipulation.h"
#include "swot_manipulation_bt/debug_mode.h"

int main(int argc, char **argv)
{
    // Initialize ROS
    ros::init(argc, argv, "swot_manipulation_bt_joel");

    // Create an instance of the Manipulation class
    // This instance calls the default constructor of the class
    Manipulation manipulation;

    // Initialize the manipulation class and its ROS services
    manipulation.initialize();
    // Create a MultiThreadedSpinner with 4 threads
    ros::MultiThreadedSpinner spinner(4);
    // Spin the spinner to process callbacks in multiple threads
    spinner.spin();

    return 0;
}

