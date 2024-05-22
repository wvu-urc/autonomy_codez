import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
class LatLong():

    def __init__(self, lat, long):
        self.lat: float = lat
        self.long: float = long

class AutonomyInterfaceNode(Node):

    def __init__(self):

        super().__init__('autonomy_text_interface_node')
        self.declare_parameter('input_goal_gps_sub_topic','/goal_gps')

        self.goal_gps_pub = self.create_publisher(
            msg_type= NavSatFix,
            topic= self.get_parameter('input_goal_gps_sub_topic').get_parameter_value().string_value,
            qos_profile= 10,
            )
        
        self.send_gps_goal()
    

    def send_gps_goal(self) -> None:
        quit = input("to quit, type quit\nto continue, hit enter\n").upper()
        if quit == 'QUIT':
            self.destroy_node()
            exit()
        try:
            lat = float(input("input latitude:\n"))
            long = float(input("input latitude:\n"))
        except TypeError as e:
            self.get_logger().info(f'TypeError {e}')
            return
        
        new_goal_gps = NavSatFix()
        new_goal_gps.latitude = lat
        new_goal_gps.longitude = long

        should_send = input("would you like to send? y/n\n").upper()

        if should_send == 'Y':
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