import os
import yaml  # Importing YAML package
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration,PythonExpression
from launch.conditions import IfCondition
from launch_ros.substitutions import FindPackageShare
import random
import xacro


random_between = lambda a,b : a + random.random()*(b-a)

def generate_launch_description():
    # Specify the name of the package and path to xacro file within the package
    pkg_name              = 'neuromorphic_collision_avoidance'
    
    description_package   = 'sml_nexus_description'
    robot_xacro_subpath   = 'description/urdf/sml_nexus.xacro'
    world_file_subpath    = 'worlds/mocap_world.world'
    config_folder_subpath = 'config'
    
    # Reading initial conditions from YAML file
    yaml_file_path = os.path.join(get_package_share_directory(pkg_name),config_folder_subpath,'initial_conditions.yaml')
    with open(yaml_file_path, 'r') as file:
        initial_conditions = yaml.safe_load(file)
        
    

    # Reading initial conditions from YAML file
   
    world_path = os.path.join(get_package_share_directory(description_package),world_file_subpath)

    
    headless      = LaunchConfiguration('headless')
    use_sim_time  = LaunchConfiguration('use_sim_time')
    use_simulator = LaunchConfiguration('use_simulator')
    world         = LaunchConfiguration('world')

    declare_simulator_cmd = DeclareLaunchArgument(
    name='headless',
    default_value='False',
    description='Whether to execute gzclient')
        
    declare_use_sim_time_cmd = DeclareLaunchArgument(
    name='use_sim_time',
    default_value='true',
    description='Use simulation (Gazebo) clock if true')

    declare_use_simulator_cmd = DeclareLaunchArgument(
    name='use_simulator',
    default_value='True',
    description='Whether to start the simulator')

    declare_world_cmd = DeclareLaunchArgument(
    name='world',
    default_value=world_path,
    description='Full path to the world model file to load')

    # Specify the actions

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
        launch_arguments=[('verbose', 'true'),('world', world)],  # Pass verbose argument to included launch file
    )
 
 
    # Create the launch description and populate
    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(declare_simulator_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_use_simulator_cmd)
    ld.add_action(declare_world_cmd)

    # Add any actions
    ld.add_action(gazebo)


    
    # Add a launch argument for verbose option
    verbose_arg = DeclareLaunchArgument('verbose', default_value='false', description='Set to true to enable verbose output')
    
    
    ld.add_action(verbose_arg)
    print(initial_conditions.items())
    
    for agent_name, info in initial_conditions.items():
        agent_id = info["agent_id"]
        pose     = info["pose"]
        x,y      = pose["x"],pose["y"]
        
        start_positions  = [x + random_between(-1.5,1.5),y + random_between(-1.5,1.5)]
        goal_position    = [-x + random_between(-1.5,1.5),-y + random_between(-1.5,1.5)]
    

        # Use xacro to process the file with the dynamic namespace mapping for each robot
        xacro_file = os.path.join(get_package_share_directory(description_package), robot_xacro_subpath)
        robot_description_config = xacro.process_file(xacro_file, mappings={"namespace": f"agent_{agent_id}"})  # this function takes the xacro file and replaces the variables in the xacro with the ones you specify. You could see the xacro as a function practically
        robot_desc = robot_description_config.toxml()  

        args = [
            '-topic', f"agent_{agent_id}"+'/robot_description', # this is the topic at which the robot description in published and that should be read by the spawner
            '-entity', f"agent_{agent_id}",
            '-x', str(start_positions[0]),
            '-y', str(start_positions[1]),
            '-z', str(pose['z']),
            '-Y', str(pose['yaw']),
        ]
        

        # this node will take the robot description from the topic and spawn the robot in the gazebo world namesspace/robot_description
        node = Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=args,
            output='screen',
            condition=IfCondition(LaunchConfiguration('verbose'))
        )
        ld.add_action(node)
        
        # this node will publishe the robot description XML file in the namesspace/robot_description topic so that it can be read by the spawner node above
        # usually both gazebo and rviz get the description of a robot from the robot description topic.
        node_robot_state_publisher = Node(
            package='robot_state_publisher',
            namespace=f"agent_{agent_id}",
            executable='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_desc}],
        )
        ld.add_action(node_robot_state_publisher)
        
        if agent_id == 1:
        
            controller_node = Node(
                package='neuromorphic_collision_avoidance',
                executable='velocity_controller.py',
                name=f'controller_{agent_id}',
                namespace=f"agent_{agent_id}",
                output='screen',
                parameters=[{'agent_id': agent_id,
                            'number_of_agents': len(initial_conditions), 
                            'goal_coordinates':goal_position,
                            'start_coordinates':start_positions,
                            "use_neuromorphic_controller":True}], # these parameters are declared inside the controller node
            )
        
        else :
            controller_node = Node(
                package='neuromorphic_collision_avoidance',
                executable='velocity_controller.py',
                name=f'controller_{agent_id}',
                namespace=f"agent_{agent_id}",
                output='screen',
                parameters=[{'agent_id': agent_id,
                            'number_of_agents': len(initial_conditions), 
                            'goal_coordinates':goal_position,
                            'start_coordinates':start_positions}], # these parameters are declared inside the controller node
            )
            
        ld.add_action(controller_node)
        
    
 
    

    return ld
