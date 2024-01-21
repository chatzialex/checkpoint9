import launch
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import ComposableNodeContainer
from launch.substitutions import LaunchConfiguration
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    """Generate launch description with multiple components."""
    
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_sim_time_arg = DeclareLaunchArgument(
      "use_sim_time",
      default_value="True"
    )
    pre_approach_node = ComposableNode(
      package='my_components',
      plugin='my_components::PreApproach',
      name='pre_approach',
      parameters = [{"use_sim_time" : use_sim_time}]
    )
    attach_server_node = ComposableNode(
      package='my_components',
      plugin='my_components::AttachServer',
      name='attach_server',
      parameters = [{"use_sim_time" : use_sim_time}]
    )

    container = ComposableNodeContainer(
      name='my_container',
      namespace='',
      package='rclcpp_components',
      executable='component_container_mt',
      composable_node_descriptions=[
        pre_approach_node,
        attach_server_node,
      ],
      output='screen',
      parameters = [{"use_sim_time" : use_sim_time}]
    )

    return launch.LaunchDescription([
      use_sim_time_arg,
      container
    ])
