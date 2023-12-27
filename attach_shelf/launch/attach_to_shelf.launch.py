from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    obstacle = LaunchConfiguration("obstacle")
    obstacle_arg = DeclareLaunchArgument(
      "obstacle",
      default_value="0.5"
    )

    degrees = LaunchConfiguration("degrees")
    degrees_arg = DeclareLaunchArgument(
      "degrees",
      default_value="0.0"
    )

    final_approach = LaunchConfiguration("final_approach")
    final_approach_arg = DeclareLaunchArgument(
      "final_approach",
      default_value="False")

    use_sim_time = LaunchConfiguration('use_sim_time')
    use_sim_time_arg = DeclareLaunchArgument(
      "use_sim_time",
      default_value="True"
    )      
    
    attach_shelf_node = Node(
      package="attach_shelf",
      executable="pre_approach_v2",
      output="screen",
      parameters = [{
        "obstacle" : obstacle, 
        "degrees" : degrees, 
        "final_approach" : final_approach, 
        "use_sim_time" : use_sim_time
      }],
      emulate_tty=True
    )
    
    approach_service_server_node = Node(
      package="attach_shelf",
      executable="approach_service_server",
      output="screen",
            parameters = [{"use_sim_time" : use_sim_time}],
            emulate_tty=True
        )
    
    return LaunchDescription([
      obstacle_arg,
      degrees_arg,
      final_approach_arg,
      use_sim_time_arg,
      attach_shelf_node,
      approach_service_server_node          
    ])