from typing import Literal
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import RegisterEventHandler, TimerAction, ExecuteProcess
from launch.event_handlers import OnProcessExit, OnProcessStart


# === Constants ===


PKG_NAME = "thermofilic_turtles"


# === Parameters ===


turtles_count = 75

simulation_type: Literal["kinesis", "taxis"] = "kinesis"

turtlesim_background_color = (210, 210, 210)

field_params = [
    {"img_file": "smiling_face.bmp"},
    {"timer_period": 1.0},  # seconds
    {"noise_stddev": 5},
]

movement_params = [
    {"linear_vel_min": 0.1},
    {"linear_vel_max": 1.0},
    {"angular_vel_min": 0.1},
    {"angular_vel_max": 3.14 / 4},
]

ctrl_nodes_launch_delay = 0.5  # seconds


# === Nodes ===


turtlesim_node = Node(
    package="turtlesim",
    executable="turtlesim_node",
    name="turtlesim",
    parameters=[
        {
            "background_r": turtlesim_background_color[0],
            "background_g": turtlesim_background_color[1],
            "background_b": turtlesim_background_color[2],
        }
    ],
)

field_publisher_node = Node(
    package=PKG_NAME,
    executable="field_publisher",
    name="field_pub",
    parameters=field_params,
    output="screen",
)

spawn_node = Node(
    package=PKG_NAME,
    executable="turtle_spawner",
    name="spawner",
    parameters=[{"turtles_count": turtles_count}],
    output="screen",
)

ctrl_ready_node = Node(
    package=PKG_NAME,
    executable="ctrl_ready_checker",
    name="ctrl_ready",
    parameters=[{"turtles_count": turtles_count}],
    output="screen",
)


# clear_turtlesim = ExecuteProcess(
#     cmd=["ros2", "service", "call", "/clear", "std_srvs/srv/Empty"], output="screen"
# )


def generate_launch_description():
    """Launch description for the thermophilic turtle simulation."""

    ctrl_nodes = []
    for i in range(1, turtles_count + 1):
        ctrl_node = Node(
            package=PKG_NAME,
            executable=f"{simulation_type}_ctrl",
            name=f"{simulation_type}_ctrl_{i}",
            parameters=[{"turtle_num": i}] + movement_params,
            output="screen",
        )
        ctrl_nodes.append(
            TimerAction(
                period=ctrl_nodes_launch_delay * (i - 1),  # cumulative delay
                actions=[ctrl_node],
            )
        )

    ctrl_chain = RegisterEventHandler(
        OnProcessStart(
            target_action=ctrl_ready_node,
            on_start=TimerAction(
                period=1.0,  # delay for activate ready listener
                actions=ctrl_nodes,
            ),
        )
    )

    turtlesim_chain = RegisterEventHandler(
        OnProcessExit(
            target_action=ctrl_ready_node,
            on_exit=[turtlesim_node],
        )
    )

    spawn_chain = RegisterEventHandler(
        OnProcessStart(
            target_action=turtlesim_node,
            on_start=[spawn_node],
        )
    )

    field_chain = RegisterEventHandler(
        OnProcessExit(target_action=spawn_node, on_exit=[field_publisher_node])
    )

    return LaunchDescription(
        [
            ctrl_ready_node,
            ctrl_chain,
            turtlesim_chain,
            spawn_chain,
            field_chain,
        ]
    )
