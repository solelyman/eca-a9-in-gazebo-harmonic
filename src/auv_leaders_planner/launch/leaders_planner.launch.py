import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Packages
    planner_pkg = 'auv_leaders_planner'
    control_pkg = 'tracking_control'
    model_pkg = 'model'

    # Paths
    planner_share = get_package_share_directory(planner_pkg)
    control_share = get_package_share_directory(control_pkg)
    model_share = get_package_share_directory(model_pkg)

    # Configs
    leaders_cfg = os.path.join(planner_share, 'config', 'leaders_params.yaml')
    vehicle_params_file = os.path.join(control_share, 'config', 'vehicle_params.yaml')
    xacro_path = os.path.join(model_share, 'urdf', 'eca_a9_dave.urdf.xacro')

    dave_prefix = os.environ.get('DAVE_WS_PATH', '/home/lu/dave_ws/install')
    if os.path.isdir(dave_prefix):
        ament_prefix = os.environ.get('AMENT_PREFIX_PATH', '')
        prefixes = ament_prefix.split(':') if ament_prefix else []
        if dave_prefix not in prefixes:
            prefixes.insert(0, dave_prefix)
            os.environ['AMENT_PREFIX_PATH'] = ':'.join(prefixes)

    env_action = SetEnvironmentVariable(
        name='AMENT_PREFIX_PATH',
        value=os.environ.get('AMENT_PREFIX_PATH', ''),
    )

    world_name = LaunchConfiguration('world_name')
    world_path = LaunchConfiguration('world_path')
    gui = LaunchConfiguration('gui')
    rviz = LaunchConfiguration('rviz')

    start_dave = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(model_share, 'launch', 'start_dave.launch.py')
        ),
        launch_arguments={
            'world_name': world_name,
            'world_path': world_path,
            'gui': gui,
        }.items()
    )

    # RViz Configuration
    rviz_config_file = os.path.join(model_share, 'rviz', 'eca_a9.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        condition=IfCondition(rviz),
        output='screen'
    )

    # Agents Configuration
    agent_config = [
        {'id': 0, 'init_pose': [0.0, 0.0, -5.0]},
        {'id': 1, 'init_pose': [-30.0, 30.0, -5.0]},
        {'id': 2, 'init_pose': [-30.0, -30.0, -5.0]},
    ]

    default_world_path = os.path.join(
        model_share,
        'worlds',
        'dave_ocean_waves_fixed.world',
    )
    
    # Spawn Red Goal Marker
    goal_x = 150.0
    goal_y = 50.0
    goal_z = -80.0
    
    spawn_goal_marker = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'goal_marker',
            '-file', os.path.join(planner_share, 'models', 'goal_marker', 'model.sdf'),
            '-x', str(goal_x),
            '-y', str(goal_y),
            '-z', str(goal_z)
        ],
        output='screen'
    )
    
    # Current Predictor Node
    model_path_arg = DeclareLaunchArgument(
        'model_path',
        default_value='/home/lu/paper1/src/network/general/Student_seed1314.pth'
    )
    nc_file_arg = DeclareLaunchArgument(
        'nc_file',
        default_value='/home/lu/paper1/src/network/expt_93_uv3z.nc4'
    )

    current_predictor_node = Node(
        package=planner_pkg,
        executable='current_predictor.py',
        name='current_predictor',
        output='screen',
        parameters=[{
            'nc_file': LaunchConfiguration('nc_file'),
            'model_path': LaunchConfiguration('model_path')
        }]
    )

    launch_entities = [
        DeclareLaunchArgument('world_name', default_value='dave_bimanual_example'),
        DeclareLaunchArgument('world_path', default_value=default_world_path),
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='false'),
        model_path_arg,
        nc_file_arg,
        env_action,
        start_dave,
        rviz_node,
        spawn_goal_marker,
        current_predictor_node,
    ]

    # Global Bridge
    all_bridge_args = ['/ocean_current@geometry_msgs/msg/Vector3]gz.msgs.Vector3d']
    all_bridge_remaps = []

    # Loop
    for agent in agent_config:
        agent_id = agent['id']
        agent_ns = f'auv{agent_id}'
        my_neighbors = [a['id'] for a in agent_config if a['id'] != agent_id]

        robot_desc_cmd = Command(['xacro', ' ', xacro_path, ' ', 'namespace:=', agent_ns])
        spawn_node = Node(
            package='ros_gz_sim',
            executable='create',
            name=f'spawn_{agent_ns}',
            arguments=[
                '-name', agent_ns,
                '-string', robot_desc_cmd,
                '-x', str(agent['init_pose'][0]),
                '-y', str(agent['init_pose'][1]),
                '-z', str(agent['init_pose'][2]),
            ],
            output='screen'
        )

        # Bridge Configuration (GZ->ROS)
        all_bridge_args.append(f'/model/{agent_ns}/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry')
        all_bridge_args.append(f'/model/{agent_ns}/pose@geometry_msgs/msg/Pose[gz.msgs.Pose')
        all_bridge_remaps.append((f'/model/{agent_ns}/odometry', f'/{agent_ns}/odom'))
        all_bridge_remaps.append((f'/model/{agent_ns}/pose', f'/{agent_ns}/pose'))

        # Thruster(ROS -> GZ)
        thrust_bridge = (
            f'/model/{agent_ns}/joint/'
            'thruster_joint/cmd_thrust@std_msgs/msg/Float64]gz.msgs.Double' 
        )
        all_bridge_args.append(thrust_bridge)
        all_bridge_remaps.append(
            (
                f'/model/{agent_ns}/joint/thruster_joint/cmd_thrust',
                f'/{agent_ns}/joint/thruster_joint/cmd_thrust',
            )
        )

        # Fins 0-3(ROS -> GZ)
        for i in range(4):
            topic = f'/model/{agent_ns}/joint/fin{i}_joint/cmd_pos'
            all_bridge_args.append(f'{topic}@std_msgs/msg/Float64]gz.msgs.Double')
            all_bridge_remaps.append((topic, f'/{agent_ns}/joint/fin{i}_joint/cmd_pos'))

        # Thruster Allocator
        allocator_node = Node(
            package=control_pkg,
            executable='thruster_allocator',
            name='thruster_allocator',
            namespace=agent_ns,
            output='screen',
            parameters=[vehicle_params_file]
        )

        #Controller
        sf_controller_node = Node(
            package=control_pkg,
            executable='sf_local_tracking',
            name='sf_controller',
            namespace=agent_ns,
            output='screen',
            parameters=[vehicle_params_file]
        )

        # Planner
        planner_node = Node(
            package=planner_pkg,
            executable='leader_planner_node',
            name='planner',
            namespace=agent_ns,
            output='screen',
            parameters=[
                vehicle_params_file,
                leaders_cfg,
                {'agent_id': agent_id, 'neighbors': my_neighbors, 'is_global_goal_publisher': (agent_id == 0)}
            ]
        )

        # Static TF (Map->Odom)
        static_tf_node = Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name=f'static_tf_{agent_ns}',
            arguments=['0', '0', '0', '0', '0', '0', 'world', f'{agent_ns}/odom'],
            output='screen'
        )

        launch_entities.extend([
            spawn_node,
            allocator_node,
            sf_controller_node,
            planner_node,
            static_tf_node,
        ])

    # Global Bridge Node
    bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='bridge_all_agents',
        arguments=all_bridge_args,
        remappings=all_bridge_remaps,
        output='screen'
    )
    launch_entities.append(bridge_node)

    return LaunchDescription(launch_entities)
