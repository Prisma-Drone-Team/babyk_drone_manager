#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from px4_msgs.msg import VehicleOdometry
import math

class VioBridge(Node):
    def __init__(self):
        super().__init__('simple_vio_bridge')
        
        # Sottoscrizione al topic di OpenVINS
        self.odom_sub = self.create_subscription(
            Odometry,
            '/base_link_pose',  # Controlla che questo sia il topic corretto!
            self.odom_callback,
            10
        )
        
        # Publisher verso la Pixhawk
        self.px4_pub = self.create_publisher(
            VehicleOdometry,
            '/fmu/in/vehicle_visual_odometry',
            10
        )
        
        self.get_logger().info('Simple VIO Bridge avviato. In attesa di odometria...')

    def odom_callback(self, msg: Odometry):
        px4_msg = VehicleOdometry()
        
        # Timestamp: converte il timestamp di ROS 2 (secondi/nanosecondi) in microsecondi per PX4
        # NOTA: Per la Pixhawk è vitale che il tempo sia sincronizzato col suo orologio interno.
        # MicroXRCEAgent solitamente gestisce la sincronizzazione, quindi usiamo il tempo di ricezione.
        px4_msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        px4_msg.timestamp_sample = px4_msg.timestamp

        # Frame NED
        px4_msg.pose_frame = VehicleOdometry.POSE_FRAME_NED
        
        # Posizione (copia diretta, assumendo che VIO sia già NED)
        px4_msg.position[0] = msg.pose.pose.position.x
        px4_msg.position[1] = msg.pose.pose.position.y
        px4_msg.position[2] = msg.pose.pose.position.z
        
        # Orientamento (copia diretta NED)
        px4_msg.q[0] = msg.pose.pose.orientation.w
        px4_msg.q[1] = msg.pose.pose.orientation.x
        px4_msg.q[2] = msg.pose.pose.orientation.y
        px4_msg.q[3] = msg.pose.pose.orientation.z
        
        # Velocità e Frame Velocità
        px4_msg.velocity_frame = VehicleOdometry.VELOCITY_FRAME_BODY_FRD
        px4_msg.velocity[0] = msg.twist.twist.linear.x
        px4_msg.velocity[1] = msg.twist.twist.linear.y
        px4_msg.velocity[2] = msg.twist.twist.linear.z
        
        px4_msg.angular_velocity[0] = msg.twist.twist.angular.x
        px4_msg.angular_velocity[1] = msg.twist.twist.angular.y
        px4_msg.angular_velocity[2] = msg.twist.twist.angular.z

        # Inseriamo invarianze generiche
        px4_msg.position_variance = [0.1, 0.1, 0.1]
        px4_msg.orientation_variance = [0.1, 0.1, 0.1]
        px4_msg.velocity_variance = [0.1, 0.1, 0.1]

        self.px4_pub.publish(px4_msg)


def main(args=None):
    rclpy.init(args=args)
    bridge = VioBridge()
    try:
        rclpy.spin(bridge)
    except KeyboardInterrupt:
        pass
    finally:
        bridge.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
