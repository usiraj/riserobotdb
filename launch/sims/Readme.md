These launch files are for running robots in simulated environment in gazebo.
These launch files accept following arguments:
- robot_name : Name of Robot to identify instance of robot.
- gazebo_world : Environment to be loaded in gazebo.
- rviz_config : rviz configuration for rviz.
- gazebo_gui : If set to true, gazebo gui is shown otherwise the gui is hidden.

### Launch files in folder
The launch files load robots with different sensor configurations. Run these launch files directly or include them into your launch file.
- p3at_basic_sim.launch : Pioneer 3 AT robot with no extra sensors attached.
- p3at_lrf_sim.launch : Pioneer 3 AT robot with Sick LMS 200 fixed on top.