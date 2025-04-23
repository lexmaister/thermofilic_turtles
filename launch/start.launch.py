from typing import Literal
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import RegisterEventHandler, TimerAction, ExecuteProcess
from launch.event_handlers import OnProcessExit, OnProcessStart


# === Constants ===


PKG_NAME = "thermofilic_turtles"


# === Parameters ===


turtles_count = 80

simulation_type: Literal["kinesis", "taxis"] = "kinesis"

turtlesim_background_color = (210, 210, 210)

field_params = [
    {"heat_center_x": 4.0},
    {"heat_center_y": 5.0},
    {"heat_sigma": 2.0},
    {"noise_stddev": 0.05},
]

movement_params = [
    {"linear_vel_min": 0.2},
    {"linear_vel_max": 2.0},
    {"angular_vel_min": 0.2},
    {"angular_vel_max": 3.14 / 2},
    {"timer_frequency": 5},  # Hz
]

gradient_painter_params = [
    {"isoline_step": 0.2},
]


ctrl_nodes_launch_delay = 1.0  # seconds


# === Nodes ===


turtlesim_node = Node(
    package="turtlesim",
    executable="turtlesim_node",
    name="sim",
    parameters=[
        {
            "background_r": turtlesim_background_color[0],
            "background_g": turtlesim_background_color[1],
            "background_b": turtlesim_background_color[2],
        }
    ],
)

gradient_painter_node = Node(
    package=PKG_NAME,
    executable="gradient_painter_node.py",
    name="gradient_painter",
    parameters=field_params + gradient_painter_params,
    output="screen",
)

field_service_node = Node(
    package=PKG_NAME,
    executable="field_service_node.py",
    name="field_service",
    parameters=field_params,
    output="screen",
)

spawn_node = Node(
    package=PKG_NAME,
    executable="turtle_spawner_node.py",
    name="turtle_spawner",
    parameters=[{"turtles_count": turtles_count}],
    output="screen",
)

ctrl_ready_node = Node(
    package=PKG_NAME,
    executable="ctrl_ready_node.py",
    name="ctrl_ready",
    parameters=[{"turtles_count": turtles_count}],
    output="screen",
)


clear_turtlesim = ExecuteProcess(
    cmd=["ros2", "service", "call", "/clear", "std_srvs/srv/Empty"], output="screen"
)


def generate_launch_description():
    """Launch description for the thermophilic turtle simulation."""

    ctrl_nodes = []
    for i in range(1, turtles_count + 1):
        ctrl_node = Node(
            package=PKG_NAME,
            executable=f"{simulation_type}_ctrl_node.py",
            name=f"{simulation_type}_controller_{i}",
            parameters=[{"turle_num": i}] + movement_params,
            output="screen",
        )
        ctrl_nodes.append(
            TimerAction(
                period=ctrl_nodes_launch_delay * (i - 1),  # cumulative delay
                actions=[ctrl_node],
            )
        )

    ctrl_chain = RegisterEventHandler(
        OnProcessExit(
            target_action=turtlesim_node, on_exit=[field_service_node, *ctrl_nodes]
        )
    )

    # gradient_painter_chain = RegisterEventHandler(
    #     OnProcessExit(
    #         target_action=dummy_node,
    #         on_exit=[gradient_painter_node],
    #     )
    # )

    spawn_chain = RegisterEventHandler(
        OnProcessExit(target_action=gradient_painter_node, on_exit=[spawn_node])
    )

    return LaunchDescription(
        [
            field_service_node,
            *ctrl_nodes,
            ctrl_ready_node,
            # turtlesim_node,
            # gradient_painter_chain,
            # spawn_chain,
        ]
    )
