This node functions to drive to a location, take the image from the camera into a png named 'image.png', 
and then drive to a thrid locatoin when the right pad on the xbox controller for the robot is pressed.

Guide to running code: (always remeber to colcon build and source on the first run and after any code change)

- position robot in it's start position

- run launch file : ros2 launch rp_household_tasks rp_household_tasks.launch.py 

- turn on the xbox controller:
    press home button until the top two sections surrounding it are blue
    press the right game pad to activate program
    wait a few seconds for robot to boot up navigation

- after the program is over you will find the image in 'image.png' in the path 
  given in the code line 88 where the path location for the image is specified

