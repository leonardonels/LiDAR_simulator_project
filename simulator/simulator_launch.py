from launch import LaunchDescription
from launch.actions import ExecuteProcess, DeclareLaunchArgument
from launch_ros.actions import Node
import yaml
import os
import pandas as pd
import numpy as np

def external_line(df, ext_df, scale=10, min_dist = 0.0):
    df['X']*=scale
    df['Y']*=scale
    X = df["X"].values
    Y = df["Y"].values
    dX = np.diff(X)
    dY = np.diff(Y)
    dX = np.append(dX, dX[-1])
    dY = np.append(dY, dY[-1])
    norm = np.sqrt(dX**2 + dY**2)  # Norma del vettore
    dX_norm = dX / norm
    dY_norm = dY / norm
    Nx = -dY_norm  # Rotazione oraria
    Ny = dX_norm   # Rotazione oraria
    offset = 3
    ext_df['X'] = X + offset * Nx
    ext_df['Y'] = Y + offset * Ny

    new_df = pd.DataFrame(df.iloc[0]).T
    new_ext = pd.DataFrame(ext_df.iloc[0]).T
    i,j = 1,0
    while i < df.shape[0]:
        X1, Y1 = df.iloc[i]["X"], df.iloc[i]["Y"]
        X2, Y2 = new_df.iloc[j]["X"], new_df.iloc[j]["Y"]
        dist = np.sqrt((X2 - X1) ** 2 + (Y2 - Y1) ** 2)
        if dist > min_dist:
            new_df = pd.concat([new_df, df.iloc[[i]]], ignore_index=True)
            j += 1
        i += 1
    i,j = 1,0
    while i < df.shape[0]:
        X1, Y1 = ext_df.iloc[i]["X"], ext_df.iloc[i]["Y"]
        X2, Y2 = new_ext.iloc[j]["X"], new_ext.iloc[j]["Y"]
        dist = np.sqrt((X2 - X1) ** 2 + (Y2 - Y1) ** 2)
        if dist > min_dist:
            new_ext = pd.concat([new_ext, ext_df.iloc[[i]]], ignore_index=True)
            j += 1
        i += 1

    return new_df, new_ext


def generate_launch_description():
    # Percorsi principali
    pkg_dir = os.path.dirname(os.path.abspath(__file__))  # Directory del file launch
    sdf_template_file = os.path.join(pkg_dir, '../sdf/world_template.sdf')  # Template del mondo SDF
    sdf_file = os.path.join(pkg_dir, '../sdf/world.sdf')  # File SDF generato
    gazebo_params_file = os.path.join(pkg_dir, '../params.yaml')  # Parametri YAML
    rviz_config_file = os.path.join(pkg_dir, '../rviz_config.rviz')  # File RViz opzionale
    bridge_yaml = os.path.join(pkg_dir, '../bridge.yaml')  # File YAML per il bridge
    race_track = os.path.join(pkg_dir, '../tracks/race_track.csv')  #File csv con un tracciato in funzione di x e y, le due feature del dataset

    # Carica i parametri dal file YAML
    if not os.path.exists(gazebo_params_file):
        raise FileNotFoundError(f"Il file YAML dei parametri non esiste: {gazebo_params_file}")
    
    with open(gazebo_params_file, 'r') as yaml_file:
        params = yaml.safe_load(yaml_file)

    # Verifica che i parametri richiesti esistano
    lidar_params = params.get('lidar', {})
    vehicle_params = params.get('vehicle',{})
    cones_params = params.get('cones',{})
    required_keys = [
        'pose', 'h_samples', 'h_min_angle', 'h_max_angle', 
        'h_resolution', 'v_samples', 'v_min_angle', 'v_max_angle', 
        'v_resolution', 'min_range', 'max_range', 'range_resolution', 'abs_pose', 'min_dist', 'scale'
    ]
    for key in required_keys:
        if key not in lidar_params and key not in vehicle_params  and key not in cones_params:
            raise ValueError(f"Parametro '{key}' mancante nel file YAML.")

    # Sostituisci i segnaposto nel template SDF
    object_template = """
        <model name='cone_{i}'>
            <static>true</static>
            <pose>{x} {y} 0 0 0 0</pose>
            <link name='cone_link'>
                <inertial>
                    <mass>0.14395</mass>
                    <inertia>
                        <ixx>0.5</ixx>
                        <ixy>0</ixy>
                        <ixz>0</ixz>
                        <iyy>0.5</iyy>
                        <iyz>0</iyz>
                        <izz>0.5</izz>
                    </inertia>
                </inertial>
                <visual name='visual'>
                    <geometry>
                        <mesh>
                            <scale>1 1 1</scale><!-- height scale: 1 = 440mm -->
                            <uri>../models/cone_440.stl</uri>
                        </mesh>
                    </geometry>
                    <material>
                        <ambient>{red} {green} {blue} 1</ambient>
                        <diffuse>{red} {green} {blue} 1</diffuse>
                        <specular>{red} {green} {blue} 1</specular>
                    </material>
                </visual>
                <collision name='collision'>
                    <geometry>
                        <mesh>
                            <uri>../models/cone_440.stl</uri>
                        </mesh>
                    </geometry>
                </collision>
                <sensor name='sensor_contact' type='contact'>
                    <contact>
                        <collision>collision</collision>
                    </contact>
                </sensor>
            </link>
        </model>
    """
    
    with open(sdf_template_file, 'r') as sdf_file_in:
        sdf_content = sdf_file_in.read()

    sdf_content = sdf_content.replace('${vehicle_abs_pose}', ' '.join(map(str, vehicle_params['abs_pose'])))
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

    df = pd.read_csv(race_track)
    ext_df = pd.DataFrame()
    df, ext_df = external_line(df, ext_df, scale=cones_params['scale'], min_dist=cones_params['min_dist'])

    int_red,int_green,int_blue = 0.0,0.0,1.0    #internal line cones color
    ext_red,ext_green,ext_blue = 1.0,1.0,0.0    #external line cones color
    
    cones = ''
    for i in range(df.shape[0]):
        cones += object_template.format(i=i, x=df.iloc[i]['X'], y=df.iloc[i]['Y'], red=int_red, green=int_green, blue=int_blue)
    for i in range(ext_df.shape[0]):
        cones += object_template.format(i=i+df.shape[0], x=ext_df.iloc[i]['X'], y=ext_df.iloc[i]['Y'], red=ext_red, green=ext_green, blue=ext_blue)

    sdf_content = sdf_content.replace('${cones}', str(cones))
    


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

    # Comando per catturare la pressione dei tasti
    ignition_keypress = ExecuteProcess(
        cmd=['ign', 'topic', '-e', '-t', '/keyboard/keypress'],
        output='screen'
    )
    print('\n\n\n[MESSAGE]: To move the car click on the plugins dropdown list in the top right corner (vertical ellipsis), select the Key Publisher.\n\n\n')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation time'
        ),
        ignition_keypress,
        ignition_gazebo,
        ros2_bridge,
        rviz2
    ])
