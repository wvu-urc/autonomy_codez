'''
My guess is you will need the following:

Lights switching is probably broken in code somewhere

Need way to parse a teleop request signal and change lights 

Need to interrupt search and remove generated search points upon seeing aruco/objects and updating goal location 

Will likely need a 2d array

    search points 
    |
    V
[] [] [] [] [] [] [] [] [] [] [] []

   [] 
   []
   []
   []


   if while at a nested array (searching), and see aruco or objects, update goal so it is at object, delete or skip to avoid traversing now outdated search points

'''



import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Bool, String
from autonomy_pkg.helpers import generate_n_random_gps_points, LatLong
from robot_interfaces.srv import GUIWaypointPath 
from robot_interfaces.msg import UrcCustomPoint
from std_srvs.srv import Trigger

class MgmtLatLong(LatLong):
    '''internal class for just pure lat long, and if this system needs to change lights upon goal arrival'''
    def __init__(self, lat, long, flash_lights):
        super.__init__(lat,long)
        self.flash_lights = flash_lights

class MgmtNode(Node):

    def __init__(self):

        super().__init__('autonomy_mgmt_node')

        self.declare_parameter('output_feedback_topic','/planner_reached_goal')
        self.declare_parameter('input_goal_gps_sub_topic','/goal_gps')
        self.declare_parameter('camera_view_distance_meters', 10.0)

        self.planner_reached_goal = False

        self.goal_gps_pub = self.create_publisher(
            msg_type= NavSatFix,
            topic= self.get_parameter('input_goal_gps_sub_topic').get_parameter_value().string_value,
            qos_profile= 10,
            )
        
        self.planner_sucess_feedback_sub = self.create_subscription(
            msg_type= Bool,
            topic= self.get_parameter('output_feedback_topic').get_parameter_value().string_value,
            callback= self.update_if_planner_has_reached_goal_and_flash,
            qos_profile= 10
            )
        
        self.recieve_goals_server = self.create_service(
            GUIWaypointPath,
            'send_waypoints_from_gui',
            self.recieve_gps_points,
        )

        self.recieve_go_to_next_point_server = self.create_service(
            Trigger,
            'go_to_next_point',
            self.go_to_next_location_service_handler,
        )

        self.recieve_go_to_next_point_server = self.create_service(
            Trigger,
            'go_to_prev_point',
            self.go_to_prev_location_service_handler,
        )

        self.autonomy_led_pub = self.create_publisher(
            String,
            'led_color_topic',
            10
            )

        self.gps_location_queue: list[MgmtLatLong] = [] 
        self.should_flash_lights_at_curr_goal: bool = False
        self.current_location_queue_index = 0 
        
    def recieve_gps_points(self, request: GUIWaypointPath.Request, response: GUIWaypointPath.Response):
        '''recieves and parses points into self.gps_location_queue, resets queue index for when new gui paths are given'''
        self.current_location_queue_index = 0 
        len_incoming_list = len(request.path.points.data)
        for urc_cust_point in request.path.points.data:
            curr_label = urc_cust_point.location_label

            if curr_label == 'gps':
                new_goal = MgmtLatLong(urc_cust_point.point.point.x,urc_cust_point.point.point.y, True)
                self.gps_location_queue.append(new_goal)

            elif curr_label == 'intermediary':
                new_goal = MgmtLatLong(urc_cust_point.point.point.x,urc_cust_point.point.y, False)
                self.gps_location_queue.append(new_goal)

            elif curr_label == 'aruco_marker':
                center_lat = urc_cust_point.point.point.x
                center_long = urc_cust_point.point.point.y
                radius = urc_cust_point.error_radius
                n = 10
                new_goals = [MgmtLatLong(point_tuple[0], point_tuple[1],False) for point_tuple in (generate_n_random_gps_points(center_lat, center_long, radius, n))]
                self.gps_location_queue.extend(new_goals)

            elif curr_label == 'object_hammer':
                center_lat = urc_cust_point.point.point.x
                center_long = urc_cust_point.point.point.y
                radius = urc_cust_point.error_radius
                n = 10
                new_goals = [MgmtLatLong(point_tuple[0], point_tuple[1],False) for point_tuple in (generate_n_random_gps_points(center_lat, center_long, radius, n))]
                self.gps_location_queue.extend(new_goals)

            elif curr_label == 'object_bottle':
                center_lat = urc_cust_point.point.point.x
                center_long = urc_cust_point.point.point.y
                radius = urc_cust_point.error_radius
                n = 10
                new_goals = [MgmtLatLong(point_tuple[0], point_tuple[1],False) for point_tuple in (generate_n_random_gps_points(center_lat, center_long, radius, n))]
                self.gps_location_queue.extend(new_goals)
            else:
                self.get_logger().info(f'A recieved point in the path from the gui has a misshaped label {curr_label}')

        response = GUIWaypointPath.Response()
        if len_incoming_list >= len(self.gps_location_queue):
            response.is_successful = True
        else:
            response.is_successful = False
            self.get_logger().info('Not all sent points in the path from the gui were properly parsed')
        return response

    
    def go_to_next_location_service_handler(self, request, response):
        response = Trigger.Response()

        if (len(self.gps_location_queue)) >= self.current_location_queue_index:
            self.get_logger().info("No more locations are accessible in the queue, the end has been reached. Please provide more locations")
            response.message = f'No more locations are accessible in the queue, the end has been reached. Please provide more locations'
            response.success = False
            return response
        
        next_location = self.gps_location_queue[self.current_location_queue_index]
        self.current_location_queue_index += 1 
        self.go_to_location(next_location)
        
        response.success = True
        response.message = f'going to next location: ({next_location.lat},{next_location.long})'
        return response
    
    def go_to_prev_location_service_handler(self, request, response):
        response = Trigger.Response()

        if self.current_location_queue_index - 1 < 0:
            response.message = f'No previous location exists (currently at the first location)'
            response.success = False
            return 
        
        prev_location = self.gps_location_queue[self.current_location_queue_index-1]
        self.go_to_location(prev_location)
        response.success = True
        response.message = f'going to next location: ({prev_location.lat},{prev_location.long})'
        return response


    def go_to_location(self, next_location: MgmtLatLong):
        """publishes to planner in send_new_goal, updates if lights need flashing upon arrival"""
        self.should_flash_lights_at_curr_goal = next_location.flash_lights
        self.send_new_goal(next_location.lat,next_location.long)

    def go_to_previous_location(self,):
        pass

    def update_if_seen_aruco(self,):

        # if see aruco, stop current search 
        # set goal point,
        # set self.should_flash_lights_at_curr_goal,
        # need to handle deletion of never reached or will not need search points

        pass

    def update_if_seen_object(self,):

        # if see object, stop current search 
        # set goal point,
        # set self.should_flash_lights_at_curr_goal,
        # need to handle deletion of never reached or will not need search points
        pass

    def update_if_planner_has_reached_goal_and_flash(self, msg: Bool):
        '''listens to feedback from planner, changes lights if need be'''

        self.planner_reached_goal = msg.data

        if self.planner_reached_goal and self.should_flash_lights_at_curr_goal:
            self.autonomy_led_pub(String(data='arrival'))

    def send_new_goal(self, lat, long):
        '''just atomically sends new goal'''
        new_goal_gps = NavSatFix()
        new_goal_gps.latitude = lat
        new_goal_gps.longitude = long
        self.goal_gps_pub.publish(new_goal_gps)
        self.autonomy_led_pub.publish(String(data='autonomous'))

def main(args=None):
    rclpy.init(args=args)
    node = MgmtNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
