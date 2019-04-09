# commander_ros_service

Commander ROS client, Assignment 2
Akhil Kandhari
axk751@case.edu
Client sends out a frequency and amplitude for wave generation to a service which gives feedback when inputs have been received and starts publishing the velocity commands for any subscribers.

## Example usage

In one terminal type "rosrun commander_ros_service commander_ros_client"
In another terminal trype "rosrun commander_ros_service commander_ros_service_pub"

In the client terminal a frequency and amplitude will be requested, once entered, the service will give feedback of the values being received and starts publishing velocity command based on inputs.


NOTE: to see client-service relation for debugging In another terminal trype "rosrun commander_ros_service commander_ros_service"
