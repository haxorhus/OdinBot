import launch
from launch.substitutions import Command, LaunchConfiguration
import launch_ros
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    default_model_path = '/home/jose/microros_ws/src/description/odin_description.urdf'
    default_rviz_config_path = '/home/jose/microros_ws/src/odin_nav/rviz/urdf_config.rviz'
    

    robot_state_publisher_node = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': Command(['xacro ', LaunchConfiguration('model')])}]
    )
    joint_state_publisher_node = launch_ros.actions.Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        arguments=[default_model_path],
#        parameters=[{'robot_description': Command(['xacro ', default_model_path])}],
#        condition=launch.conditions.UnlessCondition(LaunchConfiguration('gui'))
    )
    # voy a comentar lo siguiente pq uso gazebo ahora
    #joint_state_publisher_gui_node = launch_ros.actions.Node(
     #   package='joint_state_publisher_gui',
      #  executable='joint_state_publisher_gui',
       # name='joint_state_publisher_gui',
        #condition=launch.conditions.IfCondition(LaunchConfiguration('gui'))base_link
    #)
    rviz_node = launch_ros.actions.Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=[
          '-d' + os.path.join(
              get_package_share_directory('nav2_bringup'),
              'rviz',
              'nav2_default_view.rviz'
          )]    )
    map_generator_node = launch_ros.actions.Node(
        package='odin_nav',
        executable='map_gen',
        name='map_generator',
        output='screen',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]

    )

    map_publisher_node = launch_ros.actions.Node(
        package='odin_nav',  # Cambia 'odin_nav' por el nombre de tu paquete
        executable='map_pub',  # Nombre de tu ejecutable que publica el mapa
        name='map_publisher',
        output='screen',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )
 
    odom_node = launch_ros.actions.Node(
        package='odin_nav',
        executable='odom_node',
        name= 'odom_node',
        output='screen',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )

    control_twist_node = launch_ros.actions.Node(
        package='odin_comms',
        executable='control_twist',
        name='control_twist',
        output='screen',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]

    )

    spawn_entity = launch_ros.actions.Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'odin', 
            '-topic', 'robot_description',

        ],
        output='screen'
    )
    

    robot_localization_node = launch_ros.actions.Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[os.path.join('/home/jose/microros_ws/src/odin_nav/config/', 'ekf.yaml'), {'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )


    return launch.LaunchDescription([
        #launch.actions.DeclareLaunchArgument(name='gui', default_value='True',
        #                                    description='Flag to enable joint_state_publisher_gui'),
        launch.actions.DeclareLaunchArgument(name='model', default_value=default_model_path,
                                            description='Absolute path to robot urdf file'),
        launch.actions.DeclareLaunchArgument(name='rvizconfig', default_value=default_rviz_config_path,
                                            description='Absolute path to rviz config file'),
        launch.actions.ExecuteProcess(cmd=['gzserver', '--verbose', '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so'], output='screen'),
        
        launch.actions.DeclareLaunchArgument(name='use_sim_time', default_value='True',
                                            description='Flag to enable use_sim_time'),
        
        
        map_generator_node,
        odom_node,
        joint_state_publisher_node,
        #joint_state_publisher_gui_node,
        robot_state_publisher_node,
        #spawn_entity,
        robot_localization_node,
        map_publisher_node, 
        control_twist_node,
        rviz_node
    ])