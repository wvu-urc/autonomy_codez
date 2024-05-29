from autonomy_pkg.helpers import LatLong, calculate_gps_from_relative_distance_vector
import rclpy
from rclpy.node import Node
import math
from geometry_msgs.msg import Pose, PoseArray, Point, TransformStamped, PoseStamped
from std_msgs.msg import Int64, Bool
from std_msgs.msg import Float64
import tf2_ros
import tf_transformations
import tf2_geometry_msgs
from sensor_msgs.msg import NavSatFix
from ros2_aruco_interfaces.msg import ArucoMarkers
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

'''
target objects
0 = start post = aruco marker 0
1 = post 1 = aruco marker 1
2 = post 2 = aruco marker 2
3 = post 3 = aruco marker 3
4 = hammer = object 1
5 = keyboard = object 2
6 = water bottle = object 3

9 = gps (no object in there)

99 = any other id
'''


class ObjectChaser(Node):
    def __init__(self):
        super().__init__('object_chaser')

        self.mavros_qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        ) 

        # Subscribers
        self.create_subscription(ArucoMarkers, '/object_poses', self.objects_callback, 10)
        self.create_subscription(ArucoMarkers, '/aruco_markers', self.aruco_callback, 10)
        self.create_subscription(NavSatFix, '/mavros/global_position/global', self.rover_pose_callback, qos_profile= self.mavros_qos_profile)
        self.create_subscription(Float64, '/mavros/global_position/compass_hdg', self.heading_callback,qos_profile= self.mavros_qos_profile)
        self.create_subscription(Int64, '/target_object_id', self.target_id_callback, 10)


        # Publisher
        self.waypoint_pub = self.create_publisher(NavSatFix, '/goal_gps', 10)
        self.reached_goal_pub = self.create_publisher(Bool, '/object_reached', 10)

        # TF buffer and listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)


        # Internal state
        self.target_object_id = None 
        self.objects = None        # in base link frame
        self.aruco_markers = None  # in base link frame
        
        self.rover_pose = None # in LAT LONG
        self.rover_heading = None # in LAT LONG YAW
        self.target_pose = None # in base link frame

    def target_id_callback(self, msg):
        if msg.data >=1 and msg.data <=9:
            self.target_object_id = msg.data
        self.get_logger().info(f'Received target object ID: {self.target_object_id}')
        self.update_waypoint()

    def heading_callback(self, msg):
        self.rover_heading = msg.data

    def objects_callback(self, msg):
        self.objects = msg
        # convert id's to target object id (see mapping at top in comments)
        for i, id in enumerate(self.objects.marker_ids):
            if id == 1:
                self.objects.marker_ids[i] = 4  # Change the id to 4
            elif id == 2:
                self.objects.marker_ids[i] = 5  # Change the id to 5
            elif id == 3:
                self.objects.marker_ids[i] = 6  # Change the id to 6
            else:
                self.objects.marker_ids[i] = 100  # Change the id to 100
        
        try:
            transform = self.tf_buffer.lookup_transform('base_link', msg.header.frame_id, rclpy.time.Time())
            self.objects = self.transform_poses(msg, transform)
        except tf2_ros.LookupException as ex:
            self.get_logger().warn(f'Transform not found: {ex}')
        except tf2_ros.ExtrapolationException as ex:
            self.get_logger().warn(f'Transform error: {ex}')

        self.update_waypoint()


    def aruco_callback(self, msg):
        try:
            transform = self.tf_buffer.lookup_transform('base_link', msg.header.frame_id, rclpy.time.Time())
            self.aruco_markers = self.transform_poses(msg, transform)
        except tf2_ros.LookupException as ex:
            self.get_logger().warn(f'Transform not found: {ex}')
        except tf2_ros.ExtrapolationException as ex:
            self.get_logger().warn(f'Transform error: {ex}')

        self.update_waypoint()

    def rover_pose_callback(self, msg):
        self.rover_pose = [msg.latitude, msg.longitude]
        self.update_waypoint()

    def transform_poses(self, msg, transform):
        transformed_msg = ArucoMarkers()
        transformed_msg.header = msg.header
        transformed_msg.marker_ids = msg.marker_ids

        for pose in msg.poses:
            transformed_pose = self.transform_pose(pose, transform)
            transformed_msg.poses.append(transformed_pose)

        return transformed_msg

    def transform_pose(self, pose, transform):
        # Convert pose to homogeneous transformation matrix
        pose_mat = tf_transformations.quaternion_matrix([
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w
        ])
        pose_mat[0, 3] = pose.position.x
        pose_mat[1, 3] = pose.position.y
        pose_mat[2, 3] = pose.position.z

        # Convert transform to homogeneous transformation matrix
        trans_mat = tf_transformations.quaternion_matrix([
            transform.transform.rotation.x,
            transform.transform.rotation.y,
            transform.transform.rotation.z,
            transform.transform.rotation.w
        ])
        trans_mat[0, 3] = transform.transform.translation.x
        trans_mat[1, 3] = transform.transform.translation.y
        trans_mat[2, 3] = transform.transform.translation.z

        # Apply the transformation
        transformed_mat = tf_transformations.concatenate_matrices(trans_mat, pose_mat)

        # Extract the transformed pose
        transformed_pose = Pose()
        transformed_pose.position.x = transformed_mat[0, 3]
        transformed_pose.position.y = transformed_mat[1, 3]
        transformed_pose.position.z = transformed_mat[2, 3]
        transformed_quat = tf_transformations.quaternion_from_matrix(transformed_mat)
        transformed_pose.orientation.x = transformed_quat[0]
        transformed_pose.orientation.y = transformed_quat[1]
        transformed_pose.orientation.z = transformed_quat[2]
        transformed_pose.orientation.w = transformed_quat[3]

        return transformed_pose


    def update_waypoint(self):
        if self.rover_pose is None or self.target_object_id is None or self.rover_heading is None:
            # self.get_logger().warn('waiting for rover pose or target object id')
            return  # Wait until all data is available

        found_obj = False

        # if the target object is aruco marker check in aruco marker messages
        if self.target_object_id <= 3 and self.aruco_markers is not None:
            for i, marker_id in enumerate(self.aruco_markers.marker_ids):
                if marker_id == self.target_object_id:
                    self.target_pose = self.aruco_markers.poses[i]
                    found_obj = True
                    break

        # otherwise check yolo objects 
        elif self.objects is not None:
            for i, marker_id in enumerate(self.objects.marker_ids):
                if marker_id == self.target_object_id:
                    self.target_pose = self.objects.poses[i]
                    found_obj = True
                    break

        # If still not found, do nothing
        if not found_obj:
            return

        
        # Calculate distance to target_pose
        distance = math.sqrt((self.target_pose.position.x)**2 +
                             (self.target_pose.position.y)**2 +
                             (self.target_pose.position.z)**2)

        if distance < 2.0:
            self.target_object_id = 99 # explicitly set to 99 so we don't go back to the same object we already reached 
            # self.get_logger().info('We are close enough to the target.')
            self.reached_goal_pub.publish(Bool(data=True))
            
        else:
        
            # Calculate the waypoint based on the target pose
            goal_latlong = calculate_gps_from_relative_distance_vector(self.rover_pose[0], self.rover_pose[1], self.rover_heading, self.target_pose.position.x, -self.target_pose.position.y)
            waypoint = NavSatFix()
            waypoint.latitude = goal_latlong[0]
            waypoint.longitude = goal_latlong[1]

            self.waypoint_pub.publish(waypoint)
            # self.get_logger().info(f'Published waypoint: ({waypoint.latitude}, {waypoint.longitude})')

        # Reset the found lists
        self.aruco_markers = None
        self.objects = None


def main(args=None):
    rclpy.init(args=args)
    node = ObjectChaser()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
