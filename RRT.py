import pybullet as p
import pybullet_data
import time
import math
import random


# Global parameters, feel free to change
GOAL_ZONE = 0.2
OBSTACLE_SIZE = 0.2
end_effector_link_index = 3

# Defining a node in RRT tree as having a position, parent, and child
class RRT_Node:
    def __init__(self, position):
        self.position = position
        self.parent = None
        self.children = []

    def set_parent(self, parent):
        self.parent = parent
        
    def get_parent(self):
        return self.parent
        
    def get_position(self):
        return self.position
    
    def set_position(self, position):
        self.position = position

    def add_child(self, child):
        self.children.append(child)

# Visualization function for certain points of interest
def draw_sphere_marker(position, radius, color):
   vs_id = p.createVisualShape(p.GEOM_SPHERE, radius=radius, rgbaColor=color)
   marker_id = p.createMultiBody(basePosition=position, baseCollisionShapeIndex=-1, baseVisualShapeIndex=vs_id)
   return marker_id


def sample_configuration(robot_id, obstacles):
    num_joints = p.getNumJoints(robot_id) - 1
    lower_limits = [-3.14] * num_joints
    upper_limits = [3.14] * num_joints
    
    while True:
        random_joint_values = [
            random.uniform(lower_limits[i], upper_limits[i]) for i in range(num_joints)
        ]
        if not check_collision(robot_id, random_joint_values, obstacles):
            return random_joint_values

   
def find_nearest(rand_node, node_list):
    
    q_start_xyz = node_list[0].get_position()
    rand_node_xyz = rand_node.get_position()
    
    # initialize nearest node to be start node. Updates if node in node_list beats the start node
    nearest_node = None
    min_distance = find_distance(q_start_xyz, rand_node_xyz)
    
    for node in node_list:
        current_node_xyz = node.get_position()
        distance = find_distance(current_node_xyz, rand_node_xyz)
        if(distance <= min_distance):
            min_distance = distance
            nearest_node = node

    return nearest_node
    
# Euclidean distance helper function
def find_distance(node1_xyz, node2_xyz):
    return math.sqrt((node1_xyz[0] - node2_xyz[0]) ** 2 + (node1_xyz[1] - node2_xyz[1]) ** 2 + (node1_xyz[2] - node2_xyz[2]) ** 2)
    
# Checks if interpolated values between previous node and potential node are collision-free
def steer_to(robot_id, start_config, goal_config, obstacles, step_size=0.05):
    num_joints = len(start_config)
    steps = int(1.0 / step_size)
    interpolated_configs = [
        [
            start_config[j] + (goal_config[j] - start_config[j]) * (i / steps)
            for j in range(num_joints)
        ]
        for i in range(steps + 1)
    ]
    
    for config in interpolated_configs:
        if check_collision(robot_id, config, obstacles):
            return None  # Collision detected
    return goal_config  # Valid path


# Checking collision of all links in robot arm
def check_collision(robot_id, joint_values, obstacles):
    # Apply the joint values to the robot
    for i, value in enumerate(joint_values):
        p.resetJointState(robot_id, i, value)
    
    # Check collisions between each link of the robot and obstacles
    num_joints = p.getNumJoints(robot_id)
    for link_index in range(num_joints):
        for obstacle in obstacles:
            # Check the distance between the current link and the obstacle
            collision_points = p.getClosestPoints(
                bodyA=robot_id, bodyB=obstacle, distance=0.01, linkIndexA=link_index
            )
            if collision_points:  # Non-empty means a collision
                return True  # Collision detected
    return False  # No collision


def insert_node(old_node, new_node):
    old_node.add_child(new_node)
    new_node.set_parent(old_node)
   

def RRT(robot_id, start_config, goal_config, obstacles):
    start_node = RRT_Node(start_config)
    tree = [start_node]
    
    while True:
        random_config = sample_configuration(robot_id, obstacles)
        random_node = RRT_Node(random_config)
        
        nearest_node = find_nearest(random_node, tree)
        new_config = steer_to(robot_id, nearest_node.get_position(), random_config, obstacles)
        
        # Add node to tree if valid node
        if new_config:
            new_node = RRT_Node(new_config)
            tree.append(new_node)
            insert_node(nearest_node, new_node)
            
            if find_distance(new_config, goal_config) < GOAL_ZONE:
                goal_node = RRT_Node(goal_config)
                insert_node(new_node, goal_node)
                break
    
    # Extract the path
    path = []
    current_node = goal_node
    while current_node.get_parent():
        path = [current_node.get_position()] + path
        current_node = current_node.get_parent()

    path = [start_node.get_position()] + path
    return path


def command_to_location(body, ee_id, values):
    num_joints = p.getNumJoints(body) - 1

    joints = range(num_joints)

    print("len joints: ", len(joints))
    print("len values: ", len(values))
    
    assert len(joints) == len(values)
    for joint, value in zip(joints, values):
        p.resetJointState(body, joint, value)


if __name__ == "__main__":

    # set up simulator
    physicsClient = p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setPhysicsEngineParameter(enableFileCaching=0)
    p.setGravity(0, 0, -9.8)
    p.configureDebugVisualizer(p.COV_ENABLE_GUI, False)
    p.configureDebugVisualizer(p.COV_ENABLE_SHADOWS, True)
    p.resetDebugVisualizerCamera(cameraDistance=1.400, cameraYaw=58.000, cameraPitch=-42.200, cameraTargetPosition=(0.0, 0.0, 0.0))

    # load objects
    plane = p.loadURDF("plane.urdf")
    ur5 = p.loadURDF('assets/ur5/ur5.urdf', basePosition=[0, 0, 0.02], useFixedBase=True)
    obstacle1 = p.loadURDF('assets/block.urdf',
                           basePosition=[1/4, 0, 1/2],
                           useFixedBase=True)
    obstacle2 = p.loadURDF('assets/block.urdf',
                           basePosition=[2/4, 0, 2/3],
                           useFixedBase=True)
    obstacles = [plane, obstacle1, obstacle2]



    # start and goal
    start_position = (0.3998897969722748, -0.3993956744670868, 0.6173484325408936)
    goal_position = (0.35317009687423706, 0.35294029116630554, 0.7246701717376709)
    start_marker = draw_sphere_marker(position=start_position, radius=0.02, color=[0, 1, 0, 1])
    goal_marker = draw_sphere_marker(position=goal_position, radius=0.02, color=[1, 0, 0, 1])

    start_config = p.calculateInverseKinematics(bodyUniqueId=ur5, endEffectorLinkIndex=end_effector_link_index, targetPosition=start_position)
    goal_config = p.calculateInverseKinematics(bodyUniqueId=ur5, endEffectorLinkIndex=end_effector_link_index, targetPosition=goal_position)

    # Position robot at start node
    command_to_location(ur5, end_effector_link_index, start_config)

    
    path_configs = None
    path_configs = RRT(ur5, start_config, goal_config, obstacles)


    if path_configs is None:
        # pause here
        input("no collision-free path is found within the time budget, finish?")
    else:
        print("PATH FOUND!")
        command_to_location(ur5, end_effector_link_index, start_config)

    # Execute successful path
    while(True):
        for config in path_configs:

            for i, joint_value in enumerate(config):
                p.setJointMotorControl2(ur5, i, p.POSITION_CONTROL, targetPosition=joint_value)
            
            for _ in range(500):
                p.stepSimulation()
                time.sleep(1 / 240)  # Simulation timestep


        print("path complete, redriving from beginning!")
        command_to_location(ur5, end_effector_link_index, path_configs[0])
        time.sleep(1)






