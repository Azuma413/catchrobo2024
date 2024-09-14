from launch import LaunchDescription
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import OpaqueFunction
import sys
import pathlib
import os
from ament_index_python.packages import get_package_share_directory
sys.path.append(str(pathlib.Path(__file__).parent.absolute()))
sys.path.append(os.path.join(get_package_share_directory('realsense2_camera'), 'launch'))
import rs_launch

local_parameters = [{'name': 'camera_name',                  'default': 'camera', 'description': 'camera unique name'},
                    {'name': 'camera_namespace',             'default': 'camera', 'description': 'camera namespace'},
                    {'name': 'device_type',                  'default': "d455", 'description': 'choose device by type'},
                    {'name': 'enable_color',                 'default': 'true', 'description': 'enable color stream'},
                    {'name': 'enable_depth',                 'default': 'true', 'description': 'enable depth stream'},
                    {'name': 'pointcloud.enable',            'default': 'true', 'description': 'enable pointcloud'},
                    {'name': 'pointcloud.pointcloud_qos',    'default': 'SENSOR_DATA', 'description': ''},
                    {'name': 'rgb_camera.profile',           'default': '640,480,15', 'description': 'color image width'},
                    # {'name': 'rgb_camera.profile',           'default': '1280,720,5', 'description': 'color image width'},
                    {'name': 'depth_module.profile',         'default': '640,480,15', 'description': 'depth module profile'},
                    # {'name': 'depth_module.profile',         'default': '1280,720,5', 'description': 'depth module profile'},
                    {'name': 'publish_tf',                   'default': 'false', 'description': '[bool] enable/disable publishing static & dynamic TF'},
                    {'name': 'pointcloud.allow_no_texture_points', 'default': 'true', 'description': "''"},
                    {'name': 'decimation_filter.enable',     'default': 'true', 'description': 'enable_decimation_filter'},
                    {'name': 'spatial_filter.enable',        'default': 'false', 'description': 'enable_spatial_filter'},
                    {'name': 'temporal_filter.enable',       'default': 'true', 'description': 'enable_temporal_filter'},
                    {'name': 'disparity_filter.enable',      'default': 'false', 'description': 'enable_disparity_filter'},
                    {'name': 'hole_filling_filter.enable',   'default': 'false', 'description': 'enable_hole_filling_filter'},
                    {'name': 'align_depth.enable',           'default': 'false', 'description': 'enable align depth filter'},
                   ]
def set_configurable_parameters(local_params):
    return dict([(param['name'], LaunchConfiguration(param['name'])) for param in local_params])
def generate_launch_description():
    params = rs_launch.configurable_parameters
    return LaunchDescription(
        rs_launch.declare_configurable_parameters(local_parameters) +
        rs_launch.declare_configurable_parameters(params) +
        [
        OpaqueFunction(function=rs_launch.launch_setup, kwargs = {'params' : set_configurable_parameters(params)}),
        # Node(
        #     package='mysample', # パッケージ名
        #     executable='sample', # 実行ファイル名
        #     name='sample1', # これを設定することで同じノードを別の名前で起動できる。設定しなければ，executableと同じ名前になる。
        #     parameters=[{'color': color}], # colorパラメータにredを設定
        #     remappings=[('/cmd_vel', '/cmd_vel1'),('/topic', 'topic1')] # (元のトピック名，リマッピング後のトピック名)の形式でトピック名を上書き
        #     namespace=namespace, # ノードの名前空間
        #     output='log' # log:ログファイルに出力, screen:コンソールに出力, none
        # ),
        # Node(
        #     package="catch_cpp",
        #     executable="dynamixel_node",
        #     output="screen",
        # ),
        Node(
            package="catch_cpp",
            executable="read_serial_node",
            output="screen",
        ),
        # Node(
        #     package="catch_cpp",
        #     executable="xl320_node",
        #     output="screen",
        # ),
        Node(
            package="catch_cpp",
            executable="wrist_node",
            output="screen",
        ),
    ])
