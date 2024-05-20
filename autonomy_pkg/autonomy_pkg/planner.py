'''
https://github.com/mavlink/mavros/tree/ros2/mavros_msgs
https://docs.px4.io/main/en/ros/mavros_offboard_python.html

while have_a_goal_location

    did i get a return request signal

        yes
            
            update goal_location
        no
            

    did i get a teleop request signal

        yes
            exit
        no

            am i at the goal location

                yes

                    stop, lights
                    break, run loop
                
                no

                    what is my location
                    apply controls to go to location
                    break, run loop

x minutes will be determined by the euclidean distance between the 


Take in state information 
    gps, roll, pitch, yaw, maybe velocity - keep simple for now 
Take in goal location
Output to command velocity 
'''

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import PoseStamped
import time 

class PIDController():

    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki 
        self.kd = kd

        self.previous_error = 0 
        self.goal = 0 
        self.dt = 0.01

    def update_goal(self, goal):
        self.goal = goal 
    
    def update_control_ouput(self, current_state) -> float:
        time.sleep(self.dt)
        error = self.goal - current_state
        control_adjustment = (error*self.kp) + (((self.previous_error - error)/self.dt)*self.ki) + ((error/self.dt)*self.kd)

        self.previous_error = error 
        return control_adjustment

    
class PlannerNode(Node):

    def __init__(self):

        super().__init__('autonomy_ptp_planner_node')

        self.declare_parameter('output_control_topic','cmd_vel')

        self.declare_parameter('input_current_gps_sub_topic','/mavros/global_position/global')
        self.declare_parameter('input_gps_state_sub_topic','/goal_gps')


        self.output_control_pub = self.create_publisher(
            msg_type= Twist,
            topic= self.get_parameter('output_control_topic').get_parameter_value().string_value,
            qos_profile= 10,
            )
        
        self.input_gps_state_sub = self.create_subscription(
            msg_type= NavSatFix,
            topic= self.get_parameter('input_gps_state_sub_topic').get_parameter_value().string_value,
            callback= self.apply_control_output,
            qos_profile= 1
            )
        
        self.input_gps_goal_sub = self.create_subscription(
            msg_type= NavSatFix,
            topic= self.get_parameter('input_gps_goal_sub_topic').get_parameter_value().string_value,
            callback= self.update_goal_gps,
            qos_profile= 1
            )
        
        self.goal_gps_location: NavSatFix = None


    def update_goal_gps(self, msg: NavSatFix) -> None:

        # https://design.ros2.org/articles/legacy_interface_definition.html
        # empty float64 defaults to 0.0 (lat and long components of NavSatFix)
        message_is_not_empty = not (msg.latitude == 0.0 and msg.longitude == 0.0)

        if message_is_not_empty:
            self.goal_gps_location = msg
    
    def apply_control_output(self, msg: NavSatFix) -> None:

        def calculate_controls() -> Twist:
            pass

        if self.goal_gps_location is None:
            pass

    





   
def main(args=None):
    rclpy.init(args=args)
    node = PlannerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()