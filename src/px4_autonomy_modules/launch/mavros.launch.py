from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python import get_package_share_directory

def generate_launch_description():
    return LaunchDescription([
        #
        DeclareLaunchArgument(
            name="fcu_url",
            default_value="/dev/ttyACM0:921600"
        ),
        #
        DeclareLaunchArgument(
            name="gcs_url",
            default_value=""
        ),
        #
        DeclareLaunchArgument(
            name="tgt_system",
            default_value='1'
        ),
        #
        DeclareLaunchArgument(
            name="tgt_component",
            default_value='1'
        ),
        #
        DeclareLaunchArgument(
            name="pluginlists_yaml",
            default_value=get_package_share_directory("px4_autonomy_modules") + "/launch/px4_pluginlists.yaml"
        ),
        #
        DeclareLaunchArgument(
            name="config_yaml",
            default_value=get_package_share_directory("px4_autonomy_modules") + "/launch/px4_config.yaml"
        ),
        #
        DeclareLaunchArgument(
            name="log_output",
            default_value="screen"
        ),
        #
        DeclareLaunchArgument(
            name="fcu_protocol",
            default_value="v2.0"
        ),
        #
        DeclareLaunchArgument(
            name="respawn_mavros",
            default_value="false"
        ),
        #
        DeclareLaunchArgument(
            name="namespace",
            default_value="mavros"
        ),
        #
        DeclareLaunchArgument(
            name="drone_num",
            default_value="07"
        ),
        #
        Node(
            package="mavros",
            executable="mavros_node",
            namespace=LaunchConfiguration("namespace"),
            # output=LaunchConfiguration("log_output"),
            parameters=[
                {"fcu_url" : LaunchConfiguration("fcu_url")},
                {"gcs_url" : LaunchConfiguration("gcs_url")},
                {"tgt_system" : LaunchConfiguration("tgt_system")},
                {"tgt_component" : LaunchConfiguration("tgt_component") },
                {"fcu_protocol" : LaunchConfiguration("fcu_protocol")},
                LaunchConfiguration("pluginlists_yaml"),
                LaunchConfiguration("config_yaml")
            ]
        ),
#         Node(
#             package="px4_autonomy_modules",  # Replace with the actual package name containing your script
#             executable="camera_pose_relay_node.py",
#             name="camera_pose_relay_node",
# #            output="screen",
#             parameters=[]  # Add any parameters if needed
#         ),
        Node(
            package="px4_autonomy_modules",  # Replace with the actual package name containing your script
            executable="pose_relay_node.py",
            name="pose_relay_node",
#            output="screen",
            parameters=[]  # Add any parameters if needed
        ),
#         Node(
#             package="px4_autonomy_modules",  # Replace with the actual package name containing your script
#             executable="IMX219_camera_rebulisher.py",
#             name="camera_rebulisher_node",
# #            output="screen",
#             parameters=[]  # Add any parameters if needed
#         ),
        Node(
            package="px4_autonomy_modules",
            executable="fe_3.py",
            name=f"rob498_drone_07", #{LaunchConfiguration('drone_num')}",
            parameters=[
                {"drone_num" : LaunchConfiguration('drone_num')}
            ]

        )
    ])
