# ROSCODE
has code for ros and other coded projects for the autonomus car 
 WOOOOHOOOOO!!!

### Booting Up Simulation
##### With rocker
1. Navigate to folder containing sim_ws folder
2. Use rocker to dock into workspace
`rocker --nvidia --x11 --user --volume sim_ws -- f1tenth_gym_ros`
3. Navigate into the simulation workspace
`cd sim_ws`
4. Source necessary components
`source /opt/ros/foxy/setup.bash`
`source install/local_setup.bash`
5. Launch the simulation
`ros2 launch f1tenth_gym_ros gym_bridge_launch.py`
6. (Optional) To run the built-in controller for manual movement of the car, boot up a separate terminal and follow steps 1-4 on that terminal
7. (Optional) Run the keyboard
`ros2 run teleop_twist_keyboard teleop_twist_keyboard`

### Utilizing Packages from Labs in Simulation
1. Run the simulation per usual in one terminal
2. Boot up a separate terminal, and navigate to the lab workspace containing the desired package
e.g., `cd lab3_ws/`
3. Source necessary libraries to initialize the workspace
`source /opt/ros/foxy/setup.bash`
`source install/local_setup.bash`
4.  Launch the package
e.g., `ros2 launch lab3_pkg lab3_launch.py`