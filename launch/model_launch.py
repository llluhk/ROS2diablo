# 导入库
from launch import LaunchDescription
from launch_ros.actions import Node

# 定义函数名称为：generate_launch_description
def generate_launch_description():
    get_joystick_input = Node(
        package="get_joystick_input",
        executable="get_joystick_input"
    )
    motion_ctrl_diablo = Node(
        package="motion_ctrl_diablo",
        executable="motion_ctrl_diablo"
    )
    timestamp_processing_node = Node(
        package="timestamp_processing_node",
        executable="timestamp_processing_node"
    )

    sync_node =Node(
        package="sync_node",
        executable="sync_node"
    )

    label_record_node =Node(
        package="label_record_node",
        executable="label_record_node"
    )
    data_processing_node =Node(
        package="data_processing_node",
        executable="data_processing_node"
    )

    model_node =Node(
       package="model_node",
        executable="model_node"
    )

    motion_mux =Node(
        package="motion_mux",
        executable="motion_mux_node"
    )

    escape_node =Node(
        package="escape_node",
        executable="escape_node"
    )

    diablo_ctrl_node=Node(
        package="diablo_ctrl",
        executable="diablo_ctrl_node"
    )

    collision_time = Node(
        package= "collision_time",
        executable= "collision_time_node"
    )


    # 创建LaunchDescription对象launch_description,用于描述launch文件
    launch_description = LaunchDescription([
        get_joystick_input,
        motion_ctrl_diablo,
        timestamp_processing_node,
        sync_node
        ,label_record_node
        ,data_processing_node
        , model_node
        , motion_mux
        , escape_node
        #,diablo_ctrl_node
        #,collision_time

    ])

    # 返回让 ROS 2 根据 launch 描述执行节点
    return launch_description
