This folder contains common launch files that are used by other launch files in this package. You can use them in your launch files.
- paramload.launch : This launch file loads robot's urdf/sdf which is in xacro format into parameter server after parsing the xacro.
	- robot_model : Xacro file containing robot's description.
	- robot_name : Name of robot. It is used as namespace for robot's frame. Robot tf_prefix which is (robot_name)_tfprefix contains this name.
	- param_name : The parameter name in which to load the robot's description.
	- ignore_joints : Joints to ignore by default joint state publisher.
- loadrobot.launch : This launch file robot's description in urdf format and also runs joint state publisher and robot state publisher. This file is used for loading robot description of real robots without running rviz.
	Robot description is loaded into /(rootns)/(robot_name)_urdf
    - model_urdf : model file of robot.
    - rootns : Top level hierarchy for robots. By default = /Robots
    - robot_name : Name of robot.
    - ignore_joints : Joints to ignore by joint state publisher.
    Joint & Robot state publishers are run with namespace /(rootns)/(robot_name)/(node name)
- robotwithrviz.launch : This launch file is similar to loadrobot.launch. It in addition to that launch file starts rviz which is run with name /(rootns)/rviz. Arguments in addition to loadrobot.launch are :
	- rviz_config : RVIZ configuration file.
- loadandsimulate.launch : This launch file runs gazebo simulation and rviz for vizualziation.
	- model_urdf : URDF xacro of robot.
	- model_sdf : SDF xacro of robot.
	- robot_name : Robot's Name.
	- gazebo_world : Simulation environment to run in gazebo.
	- rviz_config : RVIZ configuration file.
	- rootns : root namespace. i.e. the topmost level of namespace hierarchy.
	- gazebo_gui : Whether to run gazebo gui or not.
	- ignore_joints : Joints to ignore by default joint state publisher.
	- use_sim_time : Whether to use simulation time.
	- paused : Whether to start simulation in paused state.
	- headless : Whether to run gazebo in headless state.
	- debug : Whehter to run gazebo in debug mode.