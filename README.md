The Node 'gather_data' functions to drive to a location, take the image from the camera into a png named 'image.png', 
and then drive to a thrid locatoin when the right pad on the xbox controller for the robot is pressed.

Guide to running code: (always remeber to colcon build and source on the first run and after any code change)

- run 'systemctl start --user sunshine' on robot terminal

- moonlight into robot on laptop (make sure on same wifi as robot)

- run 'robot_free_stretch_processes.py' on terminal

- run 'colcon build' and 'source install/setup.bash' on terminal in a workspace

- position robot in it's start position

- run launch file : 'ros2 launch rp_household_tasks rp_household_tasks.launch.py' 

- turn on the xbox controller:
    press home button until the top two sections surrounding it are blue
    press the right game pad to activate program
    wait a few seconds for robot to boot up navigation

- after the program is over you will find the image in 'image.png' in the path 
  given in the code line 88 where the path location for the image is specified

------------------------------

TIP: remeber to download the supplementry code ['rp_household_tasks_msgs'](https://github.com/masettizan/rp_household_tasks_msgs) 
as it holds the actions structure

The Node 'turn_to_frame' functions to have the robots camera turn to a given frame, which can be set as a parameter
or changed with writing an action client to request the change to a new frame. One can also request the robot to stop
or start it's motion through a request to the codes action client.

Guide to running code: (always remeber to colcon build and source on the first run and after any code change)

- same beginning steps as how to run gather_data node

- create action client code, and example of how that code would look and interact with the 'turn_to_frame' 
  code can be seen in 'example_client.py'

- add file to 'setup.py' in the same form as 'example_client.py' is written in the file

- run launch file : 'ros2 launch rp_household_tasks turn_to_camera.launch.py' 

- open another terminal

- run 'source install/setup.bash' on terminal in a workspace

- run your file with the action client to send requests to the action server,
  an example of how the command line to run the node would looks is 'ros2 run rp_household_tasks client'

- the code will run until it is run-stopped or it is request to stop from the action client

