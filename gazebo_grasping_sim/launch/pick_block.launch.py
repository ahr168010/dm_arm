from moveit_configs_utils import MoveItConfigsBuilder
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch import LaunchDescription
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    this_package_path=get_package_share_directory('gazebo_grasping_sim')

    #打开模型文件
    box_desc=open(this_package_path+'/config/box.sdf').read()
    case_desc=open(this_package_path+'/config/case.urdf').read()

    # 启动Gazebo
    gazebo_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            get_package_share_directory('ros_gz_sim')+'/launch/gz_sim.launch.py']),
        launch_arguments=[('gz_args', 'empty.sdf -r --physics-engine gz-physics-bullet-featherstone-plugin')]
    )

    # Clock Bridge
    clock_bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen'
    )

    box_to_gazebo_node = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[ '-string', box_desc, '-x', '0.4', '-y', '0.0', '-z','0.025', '-name', 'box']
    )

    case_to_gazebo_node = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[ '-string', case_desc, '-x', '0.4', '-y', '1.4', '-z','0.08', '-name', 'case']
    )

    packagepath = get_package_share_directory('dm_robotarm_config')
    print(packagepath)

    # Load the robot configuration
    moveit_config = ( MoveItConfigsBuilder("dm_arm", package_name="dm_robotarm_config")
                    .robot_description(this_package_path+'/config/dm_arm.gazebo.friction.urdf.xacro')
                    .moveit_cpp(file_path=this_package_path+"/config/moveit_cpp.yaml")
                    .robot_description_semantic('config/dm_arm.srdf')
                    .to_moveit_configs()
    )

    # 将机械臂添加到Gazebo
    robot_to_gazebo_node = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[ '-string', moveit_config.robot_description['robot_description'], '-x', '0.0', '-y', '0.0', '-z','0.0', '-name', 'dm_arm']
    )

    # 发布机械臂状态
    robot_desc_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[ moveit_config.robot_description,
            {'use_sim_time': True},  #必须使用仿真时间
            {"publish_frequency":30.0},
        ],
    )

    # Launch RViz
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        output="log",
        arguments=["-d", packagepath+"/config/moveit.rviz"],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
            {'use_sim_time': True},
        ],
    )

    #ros2_controller manger节点
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[packagepath+"/config/ros2_controllers.yaml",
            {'use_sim_time': True}],
        output="both",
    )

    #启动关节状态发布器，arm组控制器，夹抓控制器
    controller_spawner_node = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster", "arm_controller", "hand_controller"
        ],
    )

    # 启动move_group node/action server
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_config.to_dict(),
            {'use_sim_time': True}],
    )

    pick_drop_node=Node(
        name="moveit_py",
        package="motion_api",
        executable="pick_drop",
        output="both",
        parameters=[moveit_config.to_dict(),
            {'use_sim_time': True}],       
    )

    return LaunchDescription([
            gazebo_node,
            clock_bridge_node,
            box_to_gazebo_node,
            case_to_gazebo_node,
            robot_to_gazebo_node,
            robot_desc_node,
            rviz_node,
            ros2_control_node,
            controller_spawner_node,
            move_group_node,
            # pick_drop_node,
        ])