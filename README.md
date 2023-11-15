# LAB5

### This project creates a foundation for completing parts of the ARIAC (Agile Robotics for Industrial Automation Competition) 
The project begins the competition and if began sucessfully it will display the first product in the first shipment of the first order, a bin that contians the product, the pose of the object in the cameras reference frame and the reference frame of the robot base link.

#### Initial setup
To run this project the cwru_ariac_2019 package must be installed and can be found [here](https://github.com/cwru-eecs-373/cwru_ariac_2019).

Additional information about the competition and useful documentation can be found [here](https://bitbucket.org/osrf/ariac/wiki/2019/Home).


#### Usage: 
To run the project, first start the simulator by running  
`roslaunch ecse_373_ariac ecse_373_ariac.launch`  
or to run it in the background (perfered) use  
`roslaunch ecse_373_ariac ecse_373_ariac.launch &`  
Once the simulator is open, be sure to click the play button at the bottom and then wait a few seconds (typically until the simtime reaches 5 seconds) before continuing.  
To launch the node of this project run  
`rosrun ariac_entry ariac_entry_node`  
When it is desired to end the simulation run  
`killall gzserver gzclient roslaunch`  