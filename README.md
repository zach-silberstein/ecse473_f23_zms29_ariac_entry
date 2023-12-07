# Final Project

### This project creates a foundation for completing parts of the ARIAC (Agile Robotics for Industrial Automation Competition) 
The project begins the competition and if began sucessfully it will display the first product in the first shipment of the first order, a bin that contians the product, the pose of the object in the cameras reference frame and the reference frame of the robot base link. Additionally, the arm will be moved over the part, picked up, and then moved to a spot near the bin.
By Zach Silberstein

#### Initial setup
To run this project the cwru_ariac_2019 package must be installed and can be found [here](https://github.com/cwru-eecs-373/cwru_ariac_2019).

Additional information about the competition and useful documentation can be found [here](https://bitbucket.org/osrf/ariac/wiki/2019/Home).


#### Usage: 
While untested due to my virtual machine crashing, a launch file is provided that launches the ik_service_node as well as the ecse_373_ariac.launch file. This can be used by running:  
    `roslaunch ariac_entry entry.launch`  
By defualt, the launch file will not launch the ariac_entry_node. To do so, either run:  
     `roslaunch ariac_entry entry.launch ariac_entry:= true`  
or use the perfered method by running the launch file and then running the node separately with the following commands: 
    `roslaunch ariac_entry entry.launch`  
     `rosrun ariac_entry ariac_entry_node`
To run the project without the launch file, first start the simulator by running  
    `roslaunch ecse_373_ariac ecse_373_ariac.launch`  
or to run it in the background (perfered) use  
    `roslaunch ecse_373_ariac ecse_373_ariac.launch &`  
Once the simulator is open, be sure to click the play button at the bottom and then wait a few seconds (typically until the simtime reaches 5 seconds) before continuing.  
Next start the ik_service node with
     `rosrun ik_service ik_service_node`
To launch the node of this project run  
    `rosrun ariac_entry ariac_entry_node`  
When it is desired to end the simulation run  
    `killall gzserver gzclient roslaunch`  
