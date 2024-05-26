import rclpy
from rclpy.node import Node
from robot_interfaces.msg import UrcCustomPath, UrcCustomPoint
from std_msgs.msg import Bool, Int64, String
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import PointStamped

class WaypointQueue(Node):
    def __init__(self):
        super().__init__('path_subscriber_node')
        
        # Subscribers
        self.path_sub = self.create_subscription(UrcCustomPath, '/waypoint_list', self.path_callback, 10)
        self.object_reached_sub = self.create_subscription(Bool, '/object_reached', self.object_reached_callback, 10)
        self.planner_reached_goal_sub = self.create_subscription(Bool, '/planner_reached_goal', self.planner_reached_goal_callback, 10)
        self.set_index_sub = self.create_subscription(Int64, '/set_waypoint', self.set_waypoint_callback, 10)
        
        # Publishers
        self.goal_gps_pub = self.create_publisher(NavSatFix, 'goal_gps', 10)
        self.object_target_id_pub = self.create_publisher(Int64, 'object_target_id', 10)
        self.led_color_pub = self.create_publisher(String, 'led_color_topic', 10)

        self.freeze = False
        self.current_goal_index = 0
        self.path = None

    def path_callback(self, msg):
        self.path = msg.points
    
    def set_waypoint_callback(self,msg):
        self.current_goal_index = msg.data - 1 # this is so the number matches the gui waypoint id TODO

    def object_reached_callback(self, msg):
        if msg.data:
            self.get_logger().info('Object reached, jumping to the end of the list')
            self.current_goal_index = len(self.path) - 1
            self.freeze = True
            led_arrival_msg = String(data='arrival')
            self.led_color_pub.publish(led_arrival_msg)

    def planner_reached_goal_callback(self, msg):
        if msg.data:
            self.get_logger().info('Planner reached goal, sending next waypoint')
            self.publish_next_goal()

    def publish_next_goal(self):
        if self.path and self.current_goal_index < len(self.path) and not self.freeze:
            current_goal = self.path[self.current_goal_index]
            goal_msg = NavSatFix()
            goal_msg.latitude = current_goal.point.point.x
            goal_msg.longitude = current_goal.point.point.y

            self.goal_gps_pub.publish(goal_msg)
            self.object_target_id_pub.publish(Int64(data=current_goal.aruco_id))
            self.led_color_pub.publish(String(data="autonomous"))

            self.get_logger().info(f'Published next goal: {current_goal.location_label}')
        else:
            self.get_logger().info('No more goals to publish')
            self.freeze = True
            led_arrival_msg = String(data='arrival')
            self.led_color_pub.publish(led_arrival_msg)


def main(args=None):
    rclpy.init(args=args)
    node = WaypointQueue()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()