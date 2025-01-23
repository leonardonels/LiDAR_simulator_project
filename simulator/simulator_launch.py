from launch import LaunchDescription
from launch.actions import ExecuteProcess, DeclareLaunchArgument
from launch_ros.actions import Node
import yaml
import os

def generate_launch_description():
    # Percorsi principali
    pkg_dir = os.path.dirname(os.path.abspath(__file__))  # Directory del file launch
    sdf_template_file = os.path.join(pkg_dir, '../sdf/world_template.sdf')  # Template del mondo SDF
    sdf_file = os.path.join(pkg_dir, '../sdf/world.sdf')  # File SDF generato
    gazebo_params_file = os.path.join(pkg_dir, '../gazebo_params.yaml')  # Parametri YAML
    rviz_config_file = os.path.join(pkg_dir, '../rviz_config.rviz')  # File RViz opzionale
    bridge_yaml = os.path.join(pkg_dir, '../bridge.yaml')  # File YAML per il bridge

    # Carica i parametri dal file YAML
    if not os.path.exists(gazebo_params_file):
        raise FileNotFoundError(f"Il file YAML dei parametri non esiste: {gazebo_params_file}")
    
    with open(gazebo_params_file, 'r') as yaml_file:
        params = yaml.safe_load(yaml_file)

    # Verifica che i parametri richiesti esistano
    lidar_params = params.get('lidar', {})
    required_keys = [
        'pose', 'h_samples', 'h_min_angle', 'h_max_angle', 
        'h_resolution', 'v_samples', 'v_min_angle', 'v_max_angle', 
        'v_resolution', 'min_range', 'max_range', 'range_resolution'
    ]
    for key in required_keys:
        if key not in lidar_params:
            raise ValueError(f"Parametro '{key}' mancante nel file YAML.")

    # Sostituisci i segnaposto nel template SDF
    with open(sdf_template_file, 'r') as sdf_file_in:
        sdf_content = sdf_file_in.read()

    sdf_content = sdf_content.replace('${lidar_pose}', ' '.join(map(str, lidar_params['pose'])))
    sdf_content = sdf_content.replace('${lidar_h_samples}', str(lidar_params['h_samples']))
    sdf_content = sdf_content.replace('${lidar_h_min_angle}', str(lidar_params['h_min_angle']))
    sdf_content = sdf_content.replace('${lidar_h_max_angle}', str(lidar_params['h_max_angle']))
    sdf_content = sdf_content.replace('${lidar_h_resolution}', str(lidar_params['h_resolution']))
    sdf_content = sdf_content.replace('${lidar_v_samples}', str(lidar_params['v_samples']))
    sdf_content = sdf_content.replace('${lidar_v_min_angle}', str(lidar_params['v_min_angle']))
    sdf_content = sdf_content.replace('${lidar_v_max_angle}', str(lidar_params['v_max_angle']))
    sdf_content = sdf_content.replace('${lidar_v_resolution}', str(lidar_params['v_resolution']))
    sdf_content = sdf_content.replace('${lidar_min_range}', str(lidar_params['min_range']))
    sdf_content = sdf_content.replace('${lidar_max_range}', str(lidar_params['max_range']))
    sdf_content = sdf_content.replace('${lidar_range_resolution}', str(lidar_params['range_resolution']))

    # Salva il file SDF generato
    with open(sdf_file, 'w') as sdf_file_out:
        sdf_file_out.write(sdf_content)

    # Comando per avviare Ignition Gazebo con il file SDF
    ignition_gazebo = ExecuteProcess(
        cmd=['ign', 'gazebo', sdf_file],
        output='screen'
    )

    # Nodo del ROS 2 bridge
    ros2_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{'config_file': bridge_yaml}],
        output='screen'
    )

    # Nodo per RViz
    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen'
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation time'
        ),
        ignition_gazebo,
        ros2_bridge,
        rviz2
    ])
