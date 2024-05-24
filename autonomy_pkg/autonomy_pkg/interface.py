import rclpy, threading
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

class LatLong():

    def __init__(self, lat, long):
        self.lat: float = lat
        self.long: float = long

class AutonomyInterfaceNode(Node):

    def __init__(self):

        super().__init__('autonomy_text_interface_node')
        self.declare_parameter('input_goal_gps_sub_topic','/goal_gps')
        self.declare_parameter('input_current_gps_sub_topic','/mavros/global_position/global')
        self.declare_parameter('input_current_heading_sub_topic','/mavros/global_position/compass_hdg')
        self.declare_parameter('debugging', True)

        self.mavros_qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        ) 

        self.goal_gps_pub = self.create_publisher(
            msg_type= NavSatFix,
            topic= self.get_parameter('input_goal_gps_sub_topic').get_parameter_value().string_value,
            qos_profile= 10,
            )
        
        self.input_current_gps_sub = self.create_subscription(
            msg_type= NavSatFix,
            topic= self.get_parameter('input_current_gps_sub_topic').get_parameter_value().string_value,
            callback= self.update_curr_gps,
            qos_profile= self.mavros_qos_profile
        )
        self.current_gps_location = NavSatFix()

        threading.Thread(target=self.send_gps_goal, daemon=True).start()
        
    def print_if_debug(self, text: str) -> None:
        '''gets around ros's annoying method wrapping'''
        if self.get_parameter('debugging').get_parameter_value().bool_value:
            self.get_logger().info(text)

    def update_curr_gps(self, curr_gps_msg: NavSatFix) -> None:
        # self.print_if_debug(f'updating curret gps location: {curr_gps_msg.latitude},{curr_gps_msg.longitude}')
        self.current_gps_location = curr_gps_msg 

    def send_gps_goal(self) -> None:
        quit = input("to quit, type quit\nto continue, hit enter\n").upper()
        if quit == 'QUIT':
            self.destroy_node()
            exit()
        
        new_goal_gps = NavSatFix()
        send_current = input("would you like to send the robot's current location? y/n\n").upper()
        if send_current == 'Y':
            new_goal_gps = self.current_gps_location
        else:
            try:
                lat = float(input("input latitude:\n"))
                long = float(input("input longitude:\n"))
            except TypeError as e:
                self.get_logger().info(f'TypeError {e}')
                return
            new_goal_gps.latitude = lat
            new_goal_gps.longitude = long

        should_send = input("would you like to send? y/n\n").upper()
        if should_send == 'Y':
            if new_goal_gps.latitude == 0.0 or new_goal_gps.longitude == 0:
                self.get_logger().info(f"not sending, have not updated the robot's current location")
            else:
                self.get_logger().info(f"sending {new_goal_gps}")
                self.goal_gps_pub.publish(new_goal_gps)
        else:
            self.get_logger().info(f"not sending")

        self.send_gps_goal()
    
def main(args=None):
    rclpy.init(args=args)
    node = AutonomyInterfaceNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()