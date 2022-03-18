# Nordbo LRS6 Force-Torque Sensor ROS Node
Welcome to the ROS node for the Nordbo Robotics LRS6. It is assumed that the reader is familiar with a ROS and ROS nodes. 

The node has been tested on Ubuntu 20.04 LTS with ROS Noetic (But should work in any case with Melodic). 


## SetUp

Set the ethernet port to a static address to 192.168.0.99, Netmask 255.255.255.0, Gateway 192.168.0.1


## Using the node
The node is started using a roslaunch script. In a terminal, run the following: 

```
roslaunch nordbo_lrs6 nordbo_lrs6.launch
```
or
```
rosrun nordbo_lrs6 ftsensoreth.py 
```

Remember to set the correct IP adresse of the force torque sensor in the launch file. The launch file is placed in the "launch" directory. 

### Topics:
Once the node is running, wrench measurements are published to topic (default name): 
/nordbo//wrench_data

### Services: 
At time of writing, 4 services exists: 

/nordbo/start_sensor
/nordbo/stop_sensor
/nordbo/tare_sensor


**/nordbo/start_sensor**  AND 
**/nordbo/stop_sensor**
Are used to start and stop publishing to the wrench topic. 

**/nordbo/tare_sensor**
Is used to tare the sensor - eg. set current wrench equal to 0. Should be called starting the task where wrench data is used (ie. before the robot starts polishing etc.)


Code originated from NordBo Robotics Repository : https://gitlab.com/nordbo-robotics-public/nrs-6/-/tree/SimpleLogger
