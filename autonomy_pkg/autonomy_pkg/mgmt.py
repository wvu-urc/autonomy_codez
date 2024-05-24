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


pid planner current control flow 

node makes bunch of parameters
stores parameters in local variables 
makes a bunch of thread locks 


subscriptions
    current waypoint
publishings
    current local pose
    current status
    
services


'''

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import PoseStamped, Point
import time 
from math import sqrt

class MgmtNode(Node):

    def __init__(self):

        super().__init__('autonomy_mgmt_node')

        self.declare_parameter('output_control_topic','cmd_vel')
        self.declare_parameter('input_current_gps_sub_topic','/mavros/global_position/global')
        self.declare_parameter('input_gps_state_sub_topic','/goal_gps')


        
def main(args=None):
    rclpy.init(args=args)
    node = MgmtNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

