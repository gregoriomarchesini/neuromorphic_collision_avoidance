#!/usr/bin/env python3
import rclpy
import torch
import tf2_ros


from rclpy.node import Node
from functools import partial
from geometry_msgs.msg import Twist, PoseStamped, TransformStamped
from tf2_ros import LookupException
from rclpy.time import Time
from rcl_interfaces.msg import ParameterDescriptor
from .support_module import Vector, sum_vectors, scalar_multiplication, dot, normalize, norm, StandardScaler, Network, atan, pi
from scipy.optimize import  LinearConstraint, minimize
from ament_index_python.packages import get_package_share_directory



class VelocityController(Node):
    """
    This class is a STL-QP (Signal Temporal Logic-Quadratic Programming) controller for omnidirectional robots and therefore does not consider the orientation of the robots.
    It is responsible for solving the optimization problem and publishing the velocity commands to the agents. 
    """
    
    nominal_velocity    = 0.4
    max_velocity        = 0.8 # considered sudden change
    alpha               = 1.4
    collision_radius    = 0.8

    def __init__(self):
        # Initialize the node
        super().__init__('controller')

        # Velocity Command Message
        self.vel_cmd_msg = Twist()
        
        package_share_directory = get_package_share_directory('neuromorphic_collision_avoidance')
        
        # this parameter declaration lets the agent read the parameters when launched in the command line
        param_descriptor = ParameterDescriptor( description='agent ID', type = int(rclpy.Parameter.Type.INTEGER.value))
        self.declare_parameter('agent_id',value=0, descriptor=param_descriptor)
        param_descriptor = ParameterDescriptor( description='number of neighbours', type = rclpy.Parameter.Type.INTEGER.value)
        self.declare_parameter('number_of_agents',value=1, descriptor=param_descriptor)
        param_descriptor = ParameterDescriptor( description='2d coordinates of the goal location', type = rclpy.Parameter.Type.DOUBLE_ARRAY.value)
        self.declare_parameter('goal_coordinates',value=[0.,0.], descriptor=param_descriptor) 
        param_descriptor = ParameterDescriptor( description='2d coordinates of the start location', type = rclpy.Parameter.Type.DOUBLE_ARRAY.value)
        self.declare_parameter('start_coordinates',value=[0.,0.], descriptor=param_descriptor) 
        param_descriptor = ParameterDescriptor( description='use pre-trained neuromorphic controller', type = rclpy.Parameter.Type.BOOL.value)
        self.declare_parameter('use_neuromorphic_controller',value=False, descriptor=param_descriptor) 
        
        self.latest_self_transform = TransformStamped()
  
        self.identifier             = self.get_parameter('agent_id').get_parameter_value().integer_value
        self.number_of_agents       = self.get_parameter('number_of_agents').get_parameter_value().integer_value
        goal_coordinates            = self.get_parameter('goal_coordinates').get_parameter_value().double_array_value
        start_position              = self.get_parameter('start_coordinates').get_parameter_value().double_array_value 
        use_neuromorphic_controller = self.get_parameter('use_neuromorphic_controller').get_parameter_value().bool_value
        
        self.goal_position     = Vector(goal_coordinates[0],goal_coordinates[1])
        self.position          = Vector(start_position[0],start_position[1])
        self._last_update_pos_time = self.get_clock().now()
        
        
        # Setup publishers
        self.vel_pub         = self.create_publisher(Twist, f"/agent_{self.identifier}/cmd_vel", 10)
        self.agent_pose_pub  = self.create_publisher(PoseStamped, f"/agent_{self.identifier}/agent_pose", 10)
        
        
        for id in range(0, self.number_of_agents): # subscribe to the pose of the other agents and your own
            self.create_subscription(PoseStamped, f"/agent_{id}/agent_pose", 
                                    partial(self.agent_pose_callback, agent_id=id), 10)

        
        self.check_transform_timer = self.create_timer(0.001, self.pose_publishing_callback) # This timer is used to update the agent's pose
        self.control_loop          = self.create_timer(0.1, self.compute_optimal_control) # This timer is used to continuously compute the optimized input
        
        
        # Setup transform subscriber
        self.tf_buffer   = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
    
        # This timer is used to continuously compute the optimized input
        self.agents_states      = {}
        
        
        
        
        #############################################################
        
        self.position   = Vector(0.,0.)           # current position of the agent !receive start position by message
        self.velocity   = Vector(0.,0.)           # current velocity of the agent
        self.other_agents_position_current = {} # each key is an identity for the agent and the value is the position of the agent 
        self.other_agents_position_past    = {} # each key is an identity for the agent and the value is the position of the agent 
        self.other_agents_worse_impact     = {} # each key is an identity for the agent and the value is the worse velocity of the agent
        
        
        self.scaler : "StandardScaler"|None = None
        self.neuromorphic_controller_set = False
        self.neuromorphic_controller :Network|None = None
        
        if use_neuromorphic_controller:
            self.neuromorphic_controller  = Network(train_mode=False)
            self.neuromorphic_controller.load_state_dict(torch.load(package_share_directory  + '/config/model.pth'))
            self.neuromorphic_controller_set = True
            self.scaler = StandardScaler()
            self.scaler.load(package_share_directory  + '/config/scaler_params.pkl')
        
        
        self.other_agents_velocity = {} # each key is an identity for the agent and the value is the velocity of the agent
        self.position_callbacks    = [] # takes the callbacks from all the other agents
        self.optimal_velocity_change = Vector(0,0)
        
        
    
    
    def compute_barrier_constraints(self) :
        
        # barrier ->    ||r_me-r_other|| -c_r^2
        
        # dr =  (r_me-r_other)
        # dv =  ( v_me+v_change - (v_other +v_change_other))
        

    
        # so : normal direction = -(r_me-r_other)
        #      b                = -(alpha*(|r_me-r_other|^2-cr^2)  + worse_impact)
        
        normal_vector = []
        b_vector            = []
        
        for agent_id,position_other in self.other_agents_position_current.items():
            
            relative_position = Vector(self.position.x - position_other.x,self.position.y - position_other.y)
            distance_square   = dot(relative_position,relative_position) #|r_me-r_other|^2

            dr = [-(relative_position.x),-(relative_position.y)]
            b  =  -(-self.alpha*(distance_square-self.collision_radius**2)/2 )
            
            normal_vector.append(dr)
            b_vector.append(b)
            
        return normal_vector,b_vector
    
    def input_constraint(self):
         # Ax <= b
        A =  [[ 1 ,  0 ],
              [-1 ,  0 ],
              [ 0 , -1 ],
              [ 0 ,  1 ]]
    
        b = [self.max_velocity,self.max_velocity,self.max_velocity,self.max_velocity]
    
        return A,b
    
    def current_desired_velocity(self):
        
        direction_to_goal = normalize(Vector(self.goal_position.x-self.position.x,self.goal_position.y-self.position.y))
        distance_to_goal  = norm(Vector(self.goal_position.x-self.position.x,self.goal_position.y-self.position.y))
        desired_velocity  = scalar_multiplication(direction_to_goal,atan(distance_to_goal)/pi*self.nominal_velocity) 
    
        return desired_velocity
   
    def cost(self,x, desired_velocity: Vector,current_velocity:Vector):
        """select velocity change that leads the closest to the desired velocity"""
        return    10*(desired_velocity.x - x[0])**2 + 10*(desired_velocity.y - x[1])**2 
    
    
    def cost_jac(self,x, desired_velocity: Vector,current_velocity:Vector) :
            
        return [-20*(desired_velocity.x - (x[0])),
                -20*(desired_velocity.y - (x[1]))]
    
        
    def compute_optimal_velocity_change(self):
        
        
        if not self.neuromorphic_controller_set:
            A_barrier,b_barrier = self.compute_barrier_constraints()
            
            A,b = self.input_constraint()
            
            A = A_barrier + A
            b = b_barrier + b
        
            desired_velocity = self.current_desired_velocity()
            x0 = [0.0,0.0]
            
            current_velocity = self.velocity
            linear_constraint = LinearConstraint(A, ub =b)
            
            res = minimize(self.cost, x0, 
                        method      = 'SLSQP', 
                        constraints = linear_constraint,
                        jac         = self.cost_jac,
                        args        = (desired_velocity, current_velocity), 
                        options     = {'disp': False,"ftol":1e-3,"maxiter":10})
            

            if res.success:
                self._logger.debug("Solution found... speed change : "+str(res.x))
                return Vector(res.x[0],res.x[1])
            else :
                self._logger.error("No solution found. Output message from the solver: "+res.message)
                return Vector(0.,0.)
       
        else:
                
            # relative_position_other_agents
            # relative_velocity_other_agents
            # relative_position_from_goal
            
            if len(self.other_agents_position_current) != (self.number_of_agents-1): # this is to avoid problems initially when not all the agents might be ready send their position
                return Vector(0.0,0.0)
           
            # the order of the input types matter but not the order of the agents 
            input_data = []
            self._logger.info("I am neuromorphic")
            for _,position_other in self.other_agents_position_current.items():
                 
                input_data += [self.position.x - position_other.x,self.position.y - position_other.y]
            
            for _,velocity_other in self.other_agents_velocity.items():
                input_data += [self.velocity.x - velocity_other.x,self.velocity.y - velocity_other.y]
        
            
            input_data += [self.goal_position.x-self.position.x,self.goal_position.y-self.position.y]
            self._logger.info(f"Length input data: {len(input_data)}")
            input  = torch.tensor(input_data).float()
            

            if self.scaler != None :
                self._logger.info("scaling input")
                input  = self.scaler.transform(input)

                
            output = self.neuromorphic_controller(input).detach()
            self._logger.info(f"Output: {output}")
                

            return Vector(float(output[0]),float(output[1]))
        
    def compute_optimal_control(self):
        
        self.optimal_input = self.compute_optimal_velocity_change()
        self.velocity = self.optimal_input# adding some noise to the velocity
        
        
        vel_msg = Twist()   
        vel_msg.linear.x = self.velocity.x
        vel_msg.linear.y = self.velocity.y
        
        self.vel_pub.publish(vel_msg)
        
       
    
    
        
    #  ==================== ROS1 Callbacks ====================

    def velocity_callback(self, msg:Twist):
        """
        Callback function to store all the agents' poses.
        
        Args:
            msg (PoseStamped): The pose message of the other agents.
            agent_id (int): The ID of the agent extracted from the topic name.

        """
        self.vel_pub.publish(msg)
        
    
    def agent_pose_callback(self, msg:PoseStamped, agent_id):
        """
        Callback function to store all the agents' poses.
        
        Args:
            msg (PoseStamped): The pose message of the other agents.
            agent_id (int): The ID of the agent extracted from the topic name.

        """  
        pose  =  Vector(msg.pose.position.x,msg.pose.position.y)
        dt  = (self.get_clock().now() - self._last_update_pos_time).nanoseconds/1e9
        self._last_update_pos_time = self.get_clock().now()
        if agent_id != self.identifier:
            self.other_agents_position_past[agent_id]    = self.other_agents_position_current.get(agent_id,pose)
            self.other_agents_position_current[agent_id] = pose
            current_pos = self.other_agents_position_current[agent_id]
            past_pos    = self.other_agents_position_past[agent_id]
            self.other_agents_velocity[agent_id] = Vector((current_pos.x-past_pos.x)/dt ,(current_pos.y-past_pos.y)/dt) # estimation of the current velocity of the agent
            
        elif agent_id == self.identifier:
              self.position = pose
        

    def pose_publishing_callback(self): 
           
        try:

            trans = self.tf_buffer.lookup_transform("world",f"agent_{self.identifier}", Time())
            # update self tranform
            self.latest_self_transform = trans

            # Send your position to the other agents
            position_msg = PoseStamped()
            position_msg.header.stamp = self.get_clock().now().to_msg()
            position_msg.pose.position.x = trans.transform.translation.x
            position_msg.pose.position.y = trans.transform.translation.y
            self.agent_pose_pub.publish(position_msg)

  
        except LookupException as e:
            self._logger.error('failed to get transform {} \n'.format(repr(e)))
    
    

def main(args=None):
    rclpy.init(args=args)

    controller = VelocityController()

    rclpy.spin(controller)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()