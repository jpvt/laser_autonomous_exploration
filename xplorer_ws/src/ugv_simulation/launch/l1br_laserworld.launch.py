import os

from ament_index_python.packages import get_package_share_path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import ExecuteProcess
from launch.conditions import IfCondition , UnlessCondition
from launch.substitutions import LaunchConfiguration, Command
from ament_index_python.packages import get_package_prefix
from launch_ros.parameter_descriptions import ParameterValue
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit

from launch_ros.actions import Node


def generate_launch_description():
    
    # Laser World Launch
    ugv_simulation_path = get_package_share_path("ugv_simulation")
    world_gazebo_path = ugv_simulation_path / "worlds/laser.world"
    install_dir = get_package_prefix("ugv_simulation")
    world_models_path = os.path.join(ugv_simulation_path, 'worlds/models')

    if 'GAZEBO_MODEL_PATH' in os.environ:
        os.environ['GAZEBO_MODEL_PATH'] = os.environ['GAZEBO_MODEL_PATH'] + \
            ':' + install_dir + '/share' + ':' + world_models_path
    else:
        os.environ['GAZEBO_MODEL_PATH'] = install_dir + \
            "/share" + ':' + world_models_path

    if 'GAZEBO_PLUGIN_PATH' in os.environ:
        os.environ['GAZEBO_PLUGIN_PATH'] = os.environ['GAZEBO_PLUGIN_PATH'] + \
            ':' + install_dir + '/lib'
    else:
        os.environ['GAZEBO_PLUGIN_PATH'] = install_dir + '/lib'

    print("GAZEBO MODELS PATH=="+str(os.environ["GAZEBO_MODEL_PATH"]))
    print("GAZEBO PLUGINS PATH=="+str(os.environ["GAZEBO_PLUGIN_PATH"]))
    
    world_gazebo_arg = DeclareLaunchArgument(name="world", default_value=str(world_gazebo_path), description="starts world for simulation")
    
    gazebo_launch = ExecuteProcess(
        cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_factory.so',  '-s', 'libgazebo_ros_init.so', LaunchConfiguration('world')],
        output='screen'
    )

    # L1BR Launch
    ugv_robots_description_path = get_package_share_path("ugv_robots_descriptions")
    model_l1br_path = ugv_robots_description_path / "robots_description/l1br/urdf/l1br.urdf.xacro"
    rviz_config_path = ugv_robots_description_path / "rviz/default.rviz"

    gui_arg = DeclareLaunchArgument(name="gui_joint", default_value="false", choices=["true", "false"], description="Flag to enable joint_state_publisher_gui")
    model_arg = DeclareLaunchArgument(name="model", default_value=str(model_l1br_path), description="Absolute path to robot urdf file")
    rviz_config_arg = DeclareLaunchArgument(name="rvizconfig", default_value=str(rviz_config_path), description="Absolute path to rviz config file")
    rviz_launch_arg = DeclareLaunchArgument(name="rviz", default_value="true", choices=["true", "false"], description="Launch rviz with basic visualization of robot")
    use_sim_time_arg = DeclareLaunchArgument(name="use_sim_time", default_value="true", choices=["true", "false"], description='Flag to enable use_sim_time')
    
    robot_description = ParameterValue(
   	    Command(["xacro ", LaunchConfiguration("model")]) ,
      	value_type=str ,
    )

    twist_mux_params = os.path.join(get_package_share_path("ugv_simulation"),'config','twist_mux.yaml')
    twist_mux = Node(
            package="twist_mux",
            executable="twist_mux",
            parameters=[twist_mux_params, {'use_sim_time': True}],
            remappings=[('/cmd_vel_out','/diff_cont/cmd_vel_unstamped')]
        )

    robot_state_publisher_node = Node(
   	 	package= "robot_state_publisher" ,
   	 	executable= "robot_state_publisher" ,
   	 	name= "robot_state_publisher" ,
   	 	parameters= [{"robot_description" : robot_description}],
    )

    joint_state_publisher_node = Node(
   		package="joint_state_publisher",
   		executable="joint_state_publisher",       
   		condition=UnlessCondition(LaunchConfiguration("gui_joint"))
    )

    joint_state_publisher_gui_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",    
        condition=IfCondition(LaunchConfiguration("gui_joint"))
    )

    gazebo = Node(
        package="gazebo_ros" ,
        executable="spawn_entity.py" ,
        name="spawn_l1br" ,  
        output="screen" ,
        arguments= ["-topic", "/robot_description", "-entity", "l1br", "-z", "0.03", "-x", "2", "-y", "2", "-timeout", "360"] ,
    )
    
    robot_localization_node = Node(
		package="robot_localization" ,
		executable="ekf_node" ,
		name="ekf_filter_node" ,
		output="screen" ,	
		parameters=[str(get_package_share_path("ugv_simulation") / "config/ekf_l1br.yaml"), {"use_sim_time" : LaunchConfiguration("use_sim_time")}] 
	)
    
    rviz = Node(
    	package="rviz2",
    	executable="rviz2",
    	name="rviz2",     
    	output="screen",
    	arguments= ["-d", LaunchConfiguration("rvizconfig")],
    )

    diff_drive_spawner = Node(
        package="controller_manager",
        executable="spawner",       
        arguments=["diff_cont"],
    )

    joint_broad_spawner = Node(
        package="controller_manager",
        executable="spawner",     
        arguments=["joint_broad"],
    )

    # Code for delaying a node (I haven't tested how effective it is)
    # 
    # First add the below lines to imports
    # from launch.actions import RegisterEventHandler
    # from launch.event_handlers import OnProcessExit
    #
    # Then add the following below the current diff_drive_spawner
    delayed_diff_drive_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=gazebo,
            on_exit=[diff_drive_spawner],
        )
    )

    delayed_joint_broad_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=gazebo,
            on_exit=[joint_broad_spawner],
        )
    )
    #
    # Replace the diff_drive_spawner in the final return with delayed_diff_drive_spawner


    return LaunchDescription([
        # Laser World
        world_gazebo_arg ,
        gazebo_launch ,

        # L1BR
        model_arg ,
        twist_mux,
        rviz_config_arg ,
        rviz_launch_arg ,
        use_sim_time_arg ,
        gui_arg, 
        rviz ,
        gazebo ,
        robot_localization_node ,
        #diff_drive_spawner ,
        delayed_diff_drive_spawner,
        #joint_broad_spawner ,
        delayed_joint_broad_spawner,
        robot_state_publisher_node ,
        joint_state_publisher_node ,
        joint_state_publisher_gui_node ,
    ]) 
