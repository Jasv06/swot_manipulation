
# Header Files
### Overview
Inside this folder you will find 6 different header files that provide the declaration for the necessary classes used. The header files that provide this definition are the following: 
- The **pick_classes.h** file which provides the definition of the classes which allow the robot to scan a workspace, pick an object from any workspace, and store an object in one of the storage spaces available.
- The **place_classes.h** file which provides the definition of the classes which allow the robot to pick an object from the storage compartments, scan for free spaces to place the object required and place it in an available spot.
- The **shared_classes.h** file which provides the definition of the classes which allow the robot to either move to drive position or home position.
- The **manipulation.h** file which provides the definition of the main class which is in charge of starting and doing the callback of the behavior tree everytime the service is called.
- The **condition_classes.h** file which provides the definition of the conditional classes which help the behavior tree make decisions.
- The **debug_mode.h** file which provides the definition of an extern variable which is used as an argument in the launch file to determine when do we use debug or normal mode.
