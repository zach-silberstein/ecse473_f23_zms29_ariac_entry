# Final Project

### This project creates a foundation for completing parts of the ARIAC (Agile Robotics for Industrial Automation Competition) 
The project begins the competition and if began successfully it will display the first product in the first shipment of the first order, a bin that contains the product, the pose of the object in the cameras reference frame and the reference frame of the robot base link. Additionally, the arm will be moved over the part, picked up, and then moved to a spot near the bin.  
By Zach Silberstein

#### Initial setup
To run this project the cwru_ariac_2019 package must be installed and can be found [here](https://github.com/cwru-eecs-373/cwru_ariac_2019).

Additional information about the competition and useful documentation can be found [here](https://bitbucket.org/osrf/ariac/wiki/2019/Home).


#### Usage: 
While untested due to my virtual machine crashing, a launch file is provided that launches the ik_service_node as well as the ecse_373_ariac.launch file. This can be used by running:  
    `roslaunch ariac_entry entry.launch`  
By default, the launch file will not launch the ariac_entry_node. To do so, either run:  
     `roslaunch ariac_entry entry.launch ariac_entry:= true`  
or use the preferred method by running the launch file and then running the node separately with the following commands:  
    `roslaunch ariac_entry entry.launch`  
     `rosrun ariac_entry ariac_entry_node`  
To run the project without the launch file, first start the simulator by running  
    `roslaunch ecse_373_ariac ecse_373_ariac.launch`  
or to run it in the background (preferred) use  
    `roslaunch ecse_373_ariac ecse_373_ariac.launch &`  
Once the simulator is open, be sure to click the play button at the bottom and then wait a few seconds (typically until the simtime reaches 5 seconds) before continuing.  
Next start the ik_service node with  
     `rosrun ik_service ik_service_node`  
To launch the node of this project run  
    `rosrun ariac_entry ariac_entry_node`  
When it is desired to end the simulation run  
    `killall gzserver gzclient roslaunch`  

#### Future Improvements:
The node supplied here provides a successful framework which can be further built upon to successfully complete the competition. Specifically, the node is able move the arm to a desired location and is able to successful pick up the desired part. Some future improvements are listed below in the order I would implement them.  
1. Determine correct trey to place part
2. Move arm to trey
3. Determine desired pose of the part in the trey
4. Transform this pose into the frame of the robot
5. Place part onto trey
6. Add functionality to move the linear actuator near the correct part bin once it has been determined
7. Add loops to run code over each part of each order of each shipment
8. Switch to controlling arm through the action server so more information about its state can be determined
