from autonomy_pkg.helpers import LatLong, calculate_distance, calculate_goal_heading, calculate_heading_error
import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float64, Bool, Int64
from autonomy_pkg.pid import PidController
import time

class PlannerNode(Node):

    def __init__(self):

        super().__init__('autonomy_ptp_planner_node')

        # distance controller parameters
        self.declare_parameter('distance_kp', 1.0)
        self.declare_parameter('distance_ki', 0.000)
        self.declare_parameter('distance_kd', 0.0)
        self.declare_parameter('distance_net_k', 0.1)
        self.declare_parameter('internal_location_tolerance_meters', 2.0)

        # heading controller parameters
        self.declare_parameter('heading_kp', -0.04)
        self.declare_parameter('heading_ki', 0.000)
        self.declare_parameter('heading_kd', 0.0)
        self.declare_parameter('heading_net_k', 0.1)
        self.declare_parameter('internal_heading_tolerance_degrees', 5.0)

        # outgoing control output publishing parameters
        self.declare_parameter('output_control_topic','auto/cmd_vel')
        self.declare_parameter('output_control_topic_frequency_hz', 10.0)
        self.declare_parameter('output_feedback_topic','/planner_reached_goal')

        # human interfacing subsciption parameter
        self.declare_parameter('input_goal_gps_sub_topic','/goal_gps')

        # incoming sensor output subsription parameters
        self.declare_parameter('input_current_gps_sub_topic','/mavros/global_position/global')
        self.declare_parameter('input_current_heading_sub_topic','/mavros/global_position/compass_hdg')

        self.declare_parameter('planner_enabled', True)
        self.declare_parameter('debugging', True)

        self.made_to_goal = 0
        self.GOAL_CONFIRM_COUNT = 30 # number of succussful times in a row that we are within range
        self.target_object_id = 99
        self.intermediate_tolerance = 3.0
        self.FREEZE = False

        self.mavros_qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        ) 

        # sends command velocities to drivebase
        self.output_control_pub = self.create_publisher(
            msg_type= Twist,
            topic= self.get_parameter('output_control_topic').get_parameter_value().string_value,
            qos_profile= 10,
            )
        
            # sends command velocities to drivebase
        self.planner_feedback_pub = self.create_publisher(
            msg_type= Bool,
            topic= self.get_parameter('output_feedback_topic').get_parameter_value().string_value,
            qos_profile= 10,
            )
        
        # recieves goal gps locations
        self.input_goal_gps_sub = self.create_subscription(
            msg_type= NavSatFix,
            topic= self.get_parameter('input_goal_gps_sub_topic').get_parameter_value().string_value,
            callback= self.update_goal_gps,
            qos_profile= 10
            )
        
        # recieves current rover gps location
        self.input_current_gps_sub = self.create_subscription(
            msg_type= NavSatFix,
            topic= self.get_parameter('input_current_gps_sub_topic').get_parameter_value().string_value,
            callback= self.update_curr_gps,
            qos_profile= self.mavros_qos_profile
            )
        
        # recieves current rover heading information
        self.input_current_heading_sub = self.create_subscription(
            msg_type= Float64,
            topic= self.get_parameter('input_current_heading_sub_topic').get_parameter_value().string_value,
            callback= self.update_curr_heading,
            qos_profile= self.mavros_qos_profile
            )
        
        self.create_subscription(Int64, '/target_object_id', self.target_id_callback, 10)

        
        # da bread and da butter
        self.create_timer((1.0/self.get_parameter('output_control_topic_frequency_hz').get_parameter_value().double_value),self.execute_control_output)
    
        self.goal_lat_long: LatLong = LatLong(0,0)
        self.curr_lat_long: LatLong = LatLong(0,0)
        self.curr_heading_degrees: float = None 

        self.heading_controller = PidController()
        self.distance_controller = PidController()
    
    def target_id_callback(self, msg):
        self.target_object_id = msg.data
        self.get_logger().info(f'Received target object ID: {self.target_object_id}')

    def is_within_tolerance(self, error, tolerance) -> bool:
        '''checks if a given error is WITHIN OR AT a given tolerance'''
        return abs(error) <= tolerance

    def print_if_debug(self, text: str) -> None:
        '''gets around ros's annoying method wrapping'''
        if self.get_parameter('debugging').get_parameter_value().bool_value:
            self.get_logger().info(text)

    def update_goal_gps(self, goal_gps_msg: NavSatFix) -> None:
        '''recieves ros heading data, saves in local node object to self.goal_lat_long'''
        message_is_not_empty = not (goal_gps_msg.latitude == 0.0 and goal_gps_msg.longitude == 0.0) 
        
        if message_is_not_empty:
          #  self.print_if_debug(f'updating goal gps location: {goal_gps_msg.latitude},{goal_gps_msg.longitude}')
            self.goal_lat_long.lat = goal_gps_msg.latitude
            self.goal_lat_long.long = goal_gps_msg.longitude
            self.FREEZE = False
            self.made_to_goal = 0

    def update_curr_gps(self, curr_gps_msg: NavSatFix) -> None:
        '''updates internal current gps point after recieving a message from localization sensor'''
        #self.print_if_debug(f'updating curret gps location: {curr_gps_msg.latitude},{curr_gps_msg.longitude}')
        self.curr_lat_long.lat = curr_gps_msg.latitude
        self.curr_lat_long.long = curr_gps_msg.longitude 

    def update_curr_heading(self, heading_msg: Float64) -> None:
        '''recieves ros heading data, saves in local node object to self.curr_heading_degrees'''
        #self.print_if_debug(f'current heading: {heading_msg.data}')
        self.curr_heading_degrees = heading_msg.data

    def execute_control_output(self) -> None:
        '''take in current gps information, calculate distance and heading, apply controller, execute controller outputs'''

        if self.goal_lat_long.lat == 0.0 or self.goal_lat_long.long == 0.0:
            # self.get_logger().info("Have not recieved a goal, not starting control loop")
            return
        if self.curr_lat_long.lat == 0.0 or self.curr_lat_long.long == 0.0:
            # self.get_logger().info("Have not recieved the rover's current location, not starting control loop")
            return
        if self.curr_heading_degrees is None:
            # self.get_logger().info("Have not recieved the rover's current heading, not starting control loop")
            return
        
        # positional error and control calculation
        curr_goal_distance = calculate_distance(self.goal_lat_long.lat, self.goal_lat_long.long, self.curr_lat_long.lat, self.curr_lat_long.long)
       
        #self.print_if_debug(f'current distance error {curr_goal_distance}')
       
        linear_control_velocity = self.distance_controller.update(
            current_error=curr_goal_distance,
            kp=self.get_parameter('distance_kp').get_parameter_value().double_value,
            ki=self.get_parameter('distance_ki').get_parameter_value().double_value,
            kd=self.get_parameter('distance_kd').get_parameter_value().double_value,
            total_gain=self.get_parameter('distance_net_k').get_parameter_value().double_value,
        )
        #self.print_if_debug(f'linear output control (m/s) {linear_control_velocity}')

        # rotational error and control calculation
        curr_compass_goal_heading_degrees = calculate_goal_heading(self.goal_lat_long.lat, self.goal_lat_long.long, self.curr_lat_long.lat, self.curr_lat_long.long)
        
        #TODO REMOVE MAGIC NUMBER!!!!!!!!!!!!
        curr_goal_heading_error = calculate_heading_error(curr_compass_goal_heading_degrees, self.curr_heading_degrees + 180.0)
        
        #self.print_if_debug(f'current heading error {curr_goal_heading_error}')
        
        angular_control_velocity = self.heading_controller.update(
            current_error=curr_goal_heading_error,
            kp=self.get_parameter('heading_kp').get_parameter_value().double_value,
            ki=self.get_parameter('heading_ki').get_parameter_value().double_value,
            kd=self.get_parameter('heading_kd').get_parameter_value().double_value,
            total_gain=self.get_parameter('heading_net_k').get_parameter_value().double_value,
        )
        #self.print_if_debug(f'output angular control velcity (rad/sec) {angular_control_velocity}\n\n')

        executed_twist = Twist()
        sucess_feedback = Bool()

        reached_location = self.is_within_tolerance(curr_goal_distance, self.get_parameter('internal_location_tolerance_meters').get_parameter_value().double_value)
        reached_heading = self.is_within_tolerance(curr_goal_heading_error, self.get_parameter('internal_heading_tolerance_degrees').get_parameter_value().double_value)
        

        intermediate_reached_location = reached_location = self.is_within_tolerance(curr_goal_distance, self.intermediate_tolerance)
      

        if reached_location:
            self.made_to_goal += 1  # if we reached the goal increment number of times in a row
        else:
            self.made_to_goal = 0 # if not at goal set to zero
        
        # execute controller
        executed_twist.linear.x = linear_control_velocity
        if not reached_heading:
            executed_twist.angular.z = angular_control_velocity
            sucess_feedback.data = False

        if self.FREEZE:
            self.output_control_pub.publish(Twist())
            return
        
        # stop condition: intermediate waypoint: Note: avoid bad aruco marker 17
        if intermediate_reached_location and self.target_object_id >= 20 and self.made_to_goal == 1:
            #self.get_logger().info('sucessfully reached goal')
            sucess_feedback.data = True
            self.planner_feedback_pub.publish(sucess_feedback)
            executed_twist.linear.x = 0.0
            executed_twist.angular.z = 0.0
            self.output_control_pub.publish(executed_twist)
            self.made_to_goal +=1

        # stop condition: object waypoint
        if self.made_to_goal == self.GOAL_CONFIRM_COUNT and self.target_object_id < 10:
            #self.get_logger().info('sucessfully reached goal')
            sucess_feedback.data = True
            self.FREEZE = True
            self.planner_feedback_pub.publish(sucess_feedback)
            executed_twist.linear.x = 0.0
            executed_twist.angular.z = 0.0
            self.output_control_pub.publish(executed_twist)
            self.made_to_goal += 1


        # clamp linear velocity
        if (executed_twist.linear.x > 1.0):
            executed_twist.linear.x = 1.0

        if self.get_parameter('planner_enabled').get_parameter_value().bool_value:
            self.output_control_pub.publish(executed_twist)
            

def main(args=None):
    rclpy.init(args=args)
    node = PlannerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
