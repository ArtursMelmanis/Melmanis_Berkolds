import launch
from launch.actions import RegisterEventHandler, Shutdown
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node

def generate_launch_description():
    n = 5
    for rid in range(n):
        robot_sim_node = Node(
            package='task_allocation',
            executable='robot_sim',
            name='robot_sim',
            namespace=f'robot{rid}',
            output='screen',
            parameters=[{
                'blocked_center': [30.0, 30.0],
                'blocked_radius': 1.0,
                'block_period': 15.0,
                'block_duration': 8.0,
                'slowdown_factor': 0.1,
            
                'drain_rate':  1.0,
                'charge_rate': 30.0,
                'dead_timeout': 30.0,
                'idle_timeout': 3.0,
                'num_robots': 5,
                'area_A_center': [0.0,0.0],
                'area_A_radius': 2.0,
                'robot_speed': 1.0,
                'idle_speed':  1.0,
                'collision_radius': 0.2,
            }]
        )
    sim_tasks_node = Node(
        package='task_allocation',
        executable='sim_tasks',
        name='sim_tasks',
        output='screen',
        parameters=[{'wave_interval': 40.0,
                     'wave_size': 5,
                     'spawn_radius': 1.0,
                     'zone_names': ['Z1','Z2','Z3','Z4','Z5','Z6'],
                     'zone_centers': [
                        -5.0,  5.0,
                        -5.0, -5.0,
                        10.0, 10.0,
                        10.0,-10.0,
                         8.0,  0.0,
                        20.0,  0.0,
                     ],
                     'rand_seed': -1,
        }],
    )
    apso_node = Node(
        package='task_allocation',
        executable='apso_task_allocation',
        name='apso_task_allocation',
        output='screen',
        parameters=[{
            'allocation_mode': 'static',
            'max_tasks': 151,
            'pso_population': 20,
            'pso_generations': 100,
            'optimization_timeout': 5.0,
            'batch_size': 5,
            'alpha': 1.0,
            'beta': 2.5,
            'risk_penalty': 1e4,
            'collision_penalty': 1e5,
            'soc_reserve': 20.0,
            'default_speed': 1.0,
            'wave_size': 5,
        }],
    )
    viz_node = Node(
        package='task_allocation',
        executable='simple_2d_viz',
        name='simple_2d_viz',
        output='screen',
        parameters=[{
            'zone_names':   ['A', 'Z1','Z2','Z3','Z4','Z5','Z6'],
            'zone_centers': [
               0.0,  0.0,   # A
              -5.0,  5.0,   # Z1
              -5.0, -5.0,   # Z2
              10.0, 10.0,   # Z3
              10.0,-10.0,   # Z4
               8.0,  0.0,   # Z5
              20.0,  0.0,   # Z6
            ],
            'zone_radius':  2.0,
            'collision_radius': 0.2,
            'blocked_center': [30.0, 30.0],
            'blocked_radius': 1.0,
            'block_period': 15.0,
            'block_duration': 8.0,
        }],
    )
    shutdown_when_done = RegisterEventHandler(
        OnProcessExit(
            target_action=apso_node,
            on_exit=[Shutdown()]
        )
    )

    return launch.LaunchDescription([
        apso_node,
        sim_tasks_node,
        robot_sim_node,
        viz_node,
        shutdown_when_done,
    ])

