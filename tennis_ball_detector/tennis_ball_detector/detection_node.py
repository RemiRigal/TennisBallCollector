#usr/bin/python3
import rclpy
from rclpy.node import Node
from interfaces.msg import BallList
from interfaces.msg import Ball 
from sensor_msgs.msg import Image 
from geometry_msgs.msg import PoseStamped
from .scripts.tracking import track_balls, draw_boxes_from_center_coord, show_img
from .scripts.detection import detect_robot
import numpy as np
import cv2
import math

def euler_from_quaternion(quaternion):
    """
    Converts quaternion (w in last place) to euler roll, pitch, yaw
    quaternion = [x, y, z, w]
    Bellow should be replaced when porting for ROS 2 Python tf_conversions is done.
    """
    x = quaternion.x
    y = quaternion.y
    z = quaternion.z
    w = quaternion.w

    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = np.arctan2(sinr_cosp, cosr_cosp)

    sinp = 2 * (w * y - z * x)
    pitch = np.arcsin(sinp)

    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = np.arctan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw

def quaternion_from_euler(roll, pitch, yaw):
    """
    Converts euler roll, pitch, yaw to quaternion (w in last place)
    quat = [x, y, z, w]
    Bellow should be replaced when porting for ROS 2 Python tf_conversions is done.
    """
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    q = [0] * 4
    q[0] = cy * cp * cr + sy * sp * sr
    q[1] = cy * cp * sr - sy * sp * cr
    q[2] = sy * cp * sr + cy * sp * cr
    q[3] = sy * cp * cr - cy * sp * sr

    return q

class Detector(Node):

    def __init__(self):
        super().__init__('robot_balls_publisher')
        self.balls_publisher_ = self.create_publisher(BallList, '/ball_list', 10)
        self.robot_pose_publisher_ = self.create_publisher(PoseStamped, '/robot_pose', 10)

        self.cam_subscriber = self.create_subscription(
            Image,                                              
            '/zenith_camera/image_raw',
            self.image_callback,
            10)
        self.last_detected_balls = []
        self.valid_detected_balls = []
        self.ground_dx = 8
        self.ground_dy = 15

    
    def image_callback(self, msg):
        t = msg.header.stamp.sec + 10e-9 * msg.header.stamp.nanosec
        # self.get_logger().info('I heard: "%s"' % str(t)) 

        # =============================== Image extraction =======================================

        img_width = msg.width
        img_height = msg.height
        frame = np.array(msg.data).reshape((img_height,img_width,3)) #RGB images
        frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        # show_img("frame", frame)
        # cv2.waitKey(10)
        # cv2.imwrite("images/image_" + str(t) + ".png", frame)


        # =============================== Balls detection =======================================

        self.valid_detected_balls, self.last_detected_balls = track_balls(t, frame, self.last_detected_balls, self.valid_detected_balls)
        
        #print(self.valid_detected_balls)
        #draw_boxes_from_center_coord(frame, self.last_detected_balls, (0,0,255), 1)
        #draw_boxes_from_center_coord(frame, self.valid_detected_balls, (0,255,0), 1)
        # show_img("frame", frame)
        # cv2.waitKey()

        ball_list = []
        for b in self.valid_detected_balls:
            msg = Ball()
            msg.t = b[4]
            msg.x_center = b[0]
            msg.y_center = b[1]
            msg.width = b[2]
            msg.height = b[3]
            ball_list.append(msg)
        msg_list = BallList()
        msg_list.ball_list = ball_list
        self.balls_publisher_.publish(msg_list)

        # =============================== Robot detection =======================================
        
        _, robot = detect_robot(frame)
        # print(robot)
        if robot != None:
            x, y, theta = robot[1]*self.ground_dx, -robot[0]*self.ground_dy, robot[2]
            print(x, y, theta)
            pose_msg = PoseStamped()
            pose_msg.header.frame_id = "map"
            # pose_msg.header.stamp = t
            pose_msg.pose.position.x = x
            pose_msg.pose.position.y = y
            pose_msg.pose.position.z = 0.
            
            quaternion = quaternion_from_euler(0, 0, theta)
            pose_msg.pose.orientation.x = quaternion[0]
            pose_msg.pose.orientation.y = quaternion[1]
            pose_msg.pose.orientation.z = quaternion[2]
            pose_msg.pose.orientation.w = quaternion[3]

            self.robot_pose_publisher_.publish(pose_msg)

def main(args=None):
    rclpy.init(args=args)

    detector = Detector()

    rclpy.spin(detector)

    detector.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
