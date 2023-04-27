import os

from ament_index_python.packages import get_package_share_path, get_package_share_directory, get_package_prefix

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, RegisterEventHandler, GroupAction, SetEnvironmentVariable
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, Command, PythonExpression
from launch.event_handlers import OnProcessExit

from launch_ros.actions import Node, LoadComposableNodes
from launch_ros.descriptions import ComposableNode
from launch_ros.parameter_descriptions import ParameterValue

from nav2_common.launch import RewrittenYaml

def generate_launch_description():

    # Setup Gazebo variables
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

    ugv_robots_description_path = get_package_share_path("ugv_robots_descriptions")
    model_l1br_path = ugv_robots_description_path / "robots_description/l1br/urdf/l1br.urdf.xacro"
    rviz_config_path = ugv_robots_description_path / "rviz/default.rviz"

    gui_arg = DeclareLaunchArgument(name="gui_joint", default_value="false", choices=["true", "false"], description="Flag to enable joint_state_publisher_gui")
    model_arg = DeclareLaunchArgument(name="model", default_value=str(model_l1br_path), description="Absolute path to robot urdf file")
    rviz_config_arg = DeclareLaunchArgument(name="rvizconfig", default_value=str(rviz_config_path), description="Absolute path to rviz config file")
    rviz_launch_arg = DeclareLaunchArgument(name="rviz", default_value="true", choices=["true", "false"], description="Launch rviz with basic visualization of robot")
    use_sim_time_arg = DeclareLaunchArgument(name="use_sim_time", default_value="true", choices=["true", "false"], description='Flag to enable use_sim_time')
    slam_params_file_arg = DeclareLaunchArgument(
        name='slam_params_file',
        default_value=os.path.join(get_package_share_directory("ugv_simulation"),
                                'config', 'mapper_params_online_async.yaml'),
        description='Full path to the ROS2 parameters file to use for the slam_toolbox node')

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

    # SLAM Node
    start_async_slam_toolbox_node = Node(
        parameters=[
          "slam_params_file",
          {'use_sim_time': LaunchConfiguration("use_sim_time")}
        ],
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen')
    
    delayed_start_async_slam_toolbox_node = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=gazebo,
            on_exit=[start_async_slam_toolbox_node],
        )
    )

    # Navigation Node
    lifecycle_nodes = ['controller_server',
                       'smoother_server',
                       'planner_server',
                       'behavior_server',
                       'bt_navigator',
                       'waypoint_follower',
                       'velocity_smoother']

    remappings = [('/tf', 'tf'),
                  ('/tf_static', 'tf_static')]

    # Create our own temporary YAML files that include substitutions
    param_substitutions = {
        'use_sim_time': LaunchConfiguration('use_sim_time'),
        'autostart': LaunchConfiguration('autostart')}

    configured_params = RewrittenYaml(
            source_file=LaunchConfiguration('params_file'),
            root_key=LaunchConfiguration('namespace'),
            param_rewrites=param_substitutions,
            convert_types=True)

    stdout_linebuf_envvar = SetEnvironmentVariable(
        'RCUTILS_LOGGING_BUFFERED_STREAM', '1')

    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Top-level namespace')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true')

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(get_package_share_directory('ugv_simulation'), 'config', 'nav2_params.yaml'),
        description='Full path to the ROS2 parameters file to use for all launched nodes')

    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart', default_value='true',
        description='Automatically startup the nav2 stack')

    declare_use_composition_cmd = DeclareLaunchArgument(
        'use_composition', default_value='False',
        description='Use composed bringup if True')

    declare_container_name_cmd = DeclareLaunchArgument(
        'container_name', default_value='nav2_container',
        description='the name of conatiner that nodes will load in if use composition')

    declare_use_respawn_cmd = DeclareLaunchArgument(
        'use_respawn', default_value='False',
        description='Whether to respawn if a node crashes. Applied when composition is disabled.')

    declare_log_level_cmd = DeclareLaunchArgument(
        'log_level', default_value='info',
        description='log level')

    load_nodes = GroupAction(
        condition=IfCondition(PythonExpression(['not ', LaunchConfiguration("use_composition")])),
        actions=[
            Node(
                package='nav2_controller',
                executable='controller_server',
                output='screen',
                respawn=LaunchConfiguration('use_respawn'),
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
                remappings=remappings + [('cmd_vel', 'cmd_vel_nav')]),
            Node(
                package='nav2_smoother',
                executable='smoother_server',
                name='smoother_server',
                output='screen',
                respawn=LaunchConfiguration('use_respawn'),
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
                remappings=remappings),
            Node(
                package='nav2_planner',
                executable='planner_server',
                name='planner_server',
                output='screen',
                respawn=LaunchConfiguration('use_respawn'),
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
                remappings=remappings),
            Node(
                package='nav2_behaviors',
                executable='behavior_server',
                name='behavior_server',
                output='screen',
                respawn=LaunchConfiguration('use_respawn'),
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
                remappings=remappings),
            Node(
                package='nav2_bt_navigator',
                executable='bt_navigator',
                name='bt_navigator',
                output='screen',
                respawn=LaunchConfiguration('use_respawn'),
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
                remappings=remappings),
            Node(
                package='nav2_waypoint_follower',
                executable='waypoint_follower',
                name='waypoint_follower',
                output='screen',
                respawn=LaunchConfiguration('use_respawn'),
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
                remappings=remappings),
            Node(
                package='nav2_velocity_smoother',
                executable='velocity_smoother',
                name='velocity_smoother',
                output='screen',
                respawn=LaunchConfiguration('use_respawn'),
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
                remappings=remappings +
                        [('cmd_vel', 'cmd_vel_nav'), ('cmd_vel_smoothed', 'cmd_vel')]),
            Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_navigation',
                output='screen',
                arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
                parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')},
                            {'autostart': LaunchConfiguration('autostart')},
                            {'node_names': lifecycle_nodes}]),
        ]
    )

    load_composable_nodes = LoadComposableNodes(
        condition=IfCondition(LaunchConfiguration("use_composition")),
        target_container=(LaunchConfiguration('namespace'), '/', LaunchConfiguration('container_name')),
        composable_node_descriptions=[
            ComposableNode(
                package='nav2_controller',
                plugin='nav2_controller::ControllerServer',
                name='controller_server',
                parameters=[configured_params],
                remappings=remappings + [('cmd_vel', 'cmd_vel_nav')]),
            ComposableNode(
                package='nav2_smoother',
                plugin='nav2_smoother::SmootherServer',
                name='smoother_server',
                parameters=[configured_params],
                remappings=remappings),
            ComposableNode(
                package='nav2_planner',
                plugin='nav2_planner::PlannerServer',
                name='planner_server',
                parameters=[configured_params],
                remappings=remappings),
            ComposableNode(
                package='nav2_behaviors',
                plugin='behavior_server::BehaviorServer',
                name='behavior_server',
                parameters=[configured_params],
                remappings=remappings),
            ComposableNode(
                package='nav2_bt_navigator',
                plugin='nav2_bt_navigator::BtNavigator',
                name='bt_navigator',
                parameters=[configured_params],
                remappings=remappings),
            ComposableNode(
                package='nav2_waypoint_follower',
                plugin='nav2_waypoint_follower::WaypointFollower',
                name='waypoint_follower',
                parameters=[configured_params],
                remappings=remappings),
            ComposableNode(
                package='nav2_velocity_smoother',
                plugin='nav2_velocity_smoother::VelocitySmoother',
                name='velocity_smoother',
                parameters=[configured_params],
                remappings=remappings +
                           [('cmd_vel', 'cmd_vel_nav'), ('cmd_vel_smoothed', 'cmd_vel')]),
            ComposableNode(
                package='nav2_lifecycle_manager',
                plugin='nav2_lifecycle_manager::LifecycleManager',
                name='lifecycle_manager_navigation',
                parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time'),
                             'autostart': LaunchConfiguration('autostart'),
                             'node_names': lifecycle_nodes}]),
        ],
    )

    delayed_load_nodes = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=gazebo,
            on_exit=[load_nodes],
        )
    )

    delayed_load_composable_nodes = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=gazebo,
            on_exit=[load_composable_nodes],
        )
    )

    # Explore Node
    config = os.path.join(
        get_package_share_directory("explore_lite"), "config", "params.yaml"
    )

    explore_node = Node(
        package="explore_lite",
        name="explore_node",
        namespace=LaunchConfiguration('namespace'),
        executable="explore",
        parameters=[config, {"use_sim_time": LaunchConfiguration('use_sim_time')}],
        output="screen",
        remappings=[("/tf", "tf"), ("/tf_static", "tf_static")],
    )

    delayed_explore_node = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=gazebo,
            on_exit=[explore_node],
        )
    )

    return LaunchDescription([
        # Laser World + L1BR + RViz + twist_mux
        world_gazebo_arg ,
        gazebo_launch ,
        model_arg ,
        twist_mux,
        rviz_config_arg ,
        rviz_launch_arg ,
        use_sim_time_arg ,
        slam_params_file_arg ,
        gui_arg, 
        rviz ,
        gazebo ,
        robot_localization_node ,
        delayed_diff_drive_spawner,
        delayed_joint_broad_spawner,
        robot_state_publisher_node ,
        joint_state_publisher_node ,
        joint_state_publisher_gui_node ,

        #SLAM
        delayed_start_async_slam_toolbox_node,

        # Navigation
        ## Set environment variables
        stdout_linebuf_envvar,
        ## Declare the launch options
        declare_namespace_cmd,
        declare_use_sim_time_cmd,
        declare_params_file_cmd,
        declare_autostart_cmd,
        declare_use_composition_cmd,
        declare_container_name_cmd,
        declare_use_respawn_cmd,
        declare_log_level_cmd,
        ## Add the actions to launch all of the navigation nodes
        delayed_load_nodes,
        delayed_load_composable_nodes,

        # Explore Frontiers
        delayed_explore_node
    ]) 
