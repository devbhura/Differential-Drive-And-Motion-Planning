# Arm Move

Make the Interbotix px100 arm move in real life, gazebo or rviz, using moveit to plan and execute the path.

To launch from the command line, run `roslaunch arm_move arm.launch use_fake:=True use_moveit_rviz:=True` to launch in rviz. 

To launch gazebo, run `roslaunch arm_move arm.launch use_gazebo:=True`

To run on a real robot, plug in the real robot and run `roslaunch arm_move arm.launch use_actual:=True`

Three services are provided by the node:
Reset: Takes in a parameter that determines if the waypoints are cleared or not. It spawns the environment with the box and object to be picked up

Follow: Takes in number of iteration and loops through following the successful waypoints loaded to the waypoints 

Step: Takes the x,y,z coordinates of the desired end effector location and returns whether you can successfully go there or not. It executes the path if it is successful. 

The demo in gazebo and real life are shown below:
![ezgif com-gif-maker (3)](https://user-images.githubusercontent.com/55405657/140261289-c2da3b1b-ed27-4b9c-9178-bf43340aed3a.gif)

![ezgif com-gif-maker (4)](https://user-images.githubusercontent.com/55405657/140261279-9d8f54c9-4652-4c2f-b821-5a8e52c9ec77.gif)


TESTING:

To test, run `catkin_make run_tests` from your workspace. It should succeed as it takes the return value from step service and tests if it is a success or not. For one of the 2 tests, it drives the robot arm into the ground by setting negative z coordinate to check if the return value is failure and for the second test, it checks if for a positive attainable value (0.1,0.1,0.1) does the service return a success or failure. 
