# LAB5

### This project creates a foundation for completing parts of the ARIAC (Agile Robotics for Industrial Automation Competition) 
The project begins the competition and if began sucessfully it will display the first product in the first shipment of the first order, a bin that contians the product, the pose of the object in the cameras reference frame and the reference frame of the robot base link.

#### Initial setup


#### Usage: 
To run the project, run the launcer.launch file by:  
`roslaunch robot_no_crash launcher.launch`   

The launch file has the following args:
- robot_ns
- rqt_gui

> robot_ns (string, default = "robot0")  
Specifies the name space to be launced in  
  
> rqt_gui (boolean, default = "true")  
If true, then the rqt_gui is launched  
To control the robot using the gui:  
Add a Robot Steering Plugin and change the topic to **"robot0/des_vel"**
