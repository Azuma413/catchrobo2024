from launch import LaunchDescription
import launch_ros.actions
from launch.actions import OpaqueFunction
from launch.substitutions import LaunchConfiguration, ThisLaunchFileDir
import os
import sys
import pathlib
import xacro
import tempfile
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
"""
depth
1280x720x5
256x144x90
480x270x15
480x270x30
480x270x5
480x270x60
640x360x30
640x480x15
640x480x30
640x480x5
848x480x10
848x480x5

rgb
1280x720x10
1280x720x15
1280x720x5
1280x800x8
424x240x15
424x240x30
424x240x5
424x240x60
640x480x15
640x480x30
640x480x5
"""
def to_urdf(xacro_path, parameters=None):
    """Convert the given xacro file to URDF file.
    * xacro_path -- the path to the xacro file
    * parameters -- to be used when xacro file is parsed.
    """
    urdf_path = tempfile.mktemp(prefix="%s_" % os.path.basename(xacro_path))

    # open and process file
    doc = xacro.process_file(xacro_path, mappings=parameters)
    # open the output file
    out = xacro.open_output(urdf_path)
    out.write(doc.toprettyxml(indent='  '))

    return urdf_path

def set_configurable_parameters(local_params):
    return dict([(param['name'], LaunchConfiguration(param['name'])) for param in local_params])


def generate_launch_description():
    params = rs_launch.configurable_parameters
    xacro_path = os.path.join(get_package_share_directory('realsense2_description'), 'urdf', 'test_d455_camera.urdf.xacro')
    urdf = to_urdf(xacro_path, {'use_nominal_extrinsics': 'true', 'add_plug': 'true'})
    return LaunchDescription(
        rs_launch.declare_configurable_parameters(local_parameters) +
        rs_launch.declare_configurable_parameters(params) +
        [
        OpaqueFunction(function=rs_launch.launch_setup, kwargs = {'params' : set_configurable_parameters(params)}),
        # launch_ros.actions.Node(
        #     package='rviz2',
        #     namespace='',
        #     executable='rviz2',
        #     name='rviz2',
        #     arguments=['-d', '/home/kikaiken2/Documents/rviz.rviz'],
        #     output='screen',
        #     parameters=[{'use_sim_time': False}]
        # ),
        # launch_ros.actions.Node(
        #     name='model_node',
        #     package='robot_state_publisher',
        #     executable='robot_state_publisher',
        #     namespace='',
        #     output='screen',
        #     arguments=[urdf]
        # )
    ])
