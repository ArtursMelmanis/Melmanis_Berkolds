from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, Shutdown, OpaqueFunction
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node

def create_robot_group(context, *args, **kwargs):
    n = int(LaunchConfiguration('n_robots').perform(context))
    sim_flag = LaunchConfiguration('simulate').perform(context).lower() == 'true'

    nodes = []
    for rid in range(n):
        nodes.append(
            Node(
                package='robot_controller',
                executable='robot_controller',
                namespace=f'robot{rid}',
                name='controller',
                output='screen',
                parameters=[{
                    'id':        rid,
                    'simulate':  sim_flag,
                    'global_pos': True,
                    'all_sensors': False,
                }]
            )
        )
    return nodes


def generate_launch_description():
    arg_n_robots   = DeclareLaunchArgument('n_robots',   default_value='5')
    arg_adaptive   = DeclareLaunchArgument('adaptive',   default_value='true')
    arg_simulate   = DeclareLaunchArgument('simulate',   default_value='true')
    arg_task_rate  = DeclareLaunchArgument('task_rate',  default_value='2.0')
    arg_robot_speed= DeclareLaunchArgument('robot_speed',default_value='1.0')
    arg_rand_seed  = DeclareLaunchArgument('rand_seed',  default_value='42')

    adaptive    = LaunchConfiguration('adaptive')
    task_rate   = LaunchConfiguration('task_rate')
    robot_speed = LaunchConfiguration('robot_speed')
    rand_seed   = LaunchConfiguration('rand_seed')

    robot_sim_node = Node(
        package='task_allocation',
        executable='robot_sim',
        name='robot_sim',
        output='screen',
        parameters=[{
            'num_robots': LaunchConfiguration('n_robots'),
            'area_A_center': [0.0, 0.0],
            'area_A_radius': 1.0,
            'robot_speed': robot_speed,
            'idle_speed':  robot_speed,
        }]
    )

    sim_tasks_node = Node(
        package='task_allocation',
        executable='sim_tasks',
        name='sim_tasks',
        output='screen',
        parameters=[{
            'rate': task_rate,
            'spawn_radius': 1.0,
            'zone_names':  ['B','C','D','E'],
            'zone_centers':[ 5.0, 0.0,
                            -5.0, 0.0,
                             0.0, 5.0,
                             0.0,-5.0 ],
            'rand_seed': rand_seed,
        }]
    )

    fake_camera_node = Node(
        package='task_allocation',
        executable='fake_camera_server',
        name='fake_camera_server',
        output='screen',
    )

    apso_node = Node(
        package='task_allocation',
        executable='apso_task_allocation',
        name='apso_task_allocation',
        output='screen',
        parameters=[{
            'allocation_mode': PythonExpression(
                ["'adaptive' if '", LaunchConfiguration('adaptive'),
                 "' == 'true' else 'static'"]),
            'max_tasks': 10,
            'pso_population': 20,
            'pso_generations': 50,
            'batch_size': 5,
            'alpha': 1.0,
            'collision_penalty': 1e6,
            'gamma': 50.0,
        }],
    )

    viz_node = Node(
        package='task_allocation',
        executable='simple_2d_viz',
        name='simple_2d_viz',
        output='screen',
        parameters=[{
            'zone_names':   ['A','B','C','D','E'],
            'zone_centers': [ 0.0,  0.0,  5.0, 0.0, -5.0, 0.0, 0.0, 5.0, 0.0,-5.0 ],
            'zone_radius':  1.0,
        }]
    )

    shutdown_when_done = RegisterEventHandler(
        OnProcessExit(
            target_action=apso_node,
            on_exit=[Shutdown()]
        )
    )

    return LaunchDescription([
        arg_n_robots, arg_adaptive, arg_simulate,
        arg_task_rate, arg_robot_speed, arg_rand_seed,

        sim_tasks_node,
        robot_sim_node,
        fake_camera_node,
        apso_node,
        viz_node,
        OpaqueFunction(function=create_robot_group),
        shutdown_when_done,
    ])

