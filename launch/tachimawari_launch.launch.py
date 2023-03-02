from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    firstPackage = LaunchConfiguration('firstPackage')
    mode = LaunchConfiguration('mode')
    
    firstPackage_launch_arg = DeclareLaunchArgument(
        'firstPackage',
        default_value='tachimawari'
    )
    
    mode_launch_arg = DeclareLaunchArgument(
        'mode',
        default_value=None,
        description='choose mode between sdk/cm740!',
        choices=['sdk', 'cm740']
    )
    
    firstNode = Node(
        package=firstPackage,
        executable='main',
        name='tachimawari',
        arguments=[mode]
    )

    run_first_package = ExecuteProcess(
        cmd=[[
            'ros2 ',
            'run ',
            firstPackage,
            ' main ',
            mode
        ]],
        shell=True
    )

    return LaunchDescription([
        firstPackage_launch_arg,
        mode_launch_arg,
        firstNode,
        run_first_package
    ])
