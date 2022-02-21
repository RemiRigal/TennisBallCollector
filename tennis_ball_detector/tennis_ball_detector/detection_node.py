# usr/bin/python3
import rclpy
from rclpy.node import Node
from interfaces.msg import BallList
from interfaces.msg import Ball 
from sensor_msgs.msg import Image 
from tracking import track_balls
import numpy as np
# import cv2

class BallsDetector(Node):

    def __init__(self):
        super().__init__('balls_publisher')
        self.balls_publisher_ = self.create_publisher(BallList, '/ball_list', 10)     # CHANGE
        self.cam_subscriber = self.create_subscription(
            Image,                                              
            '/zenith_camera/image_raw',
            self.image_callback,
            10)
        # timer_period = 0.5
        # self.timer = self.create_timer(timer_period, self.timer_callback)
        # self.i = 0
        self.last_detected_balls = []
        self.valid_detected_balls = []

    # def timer_callback(self):
    #     msg = Ball()                                           
    #     msg.t = self.t                                      
    #     self.balls_publisher_.publish(msg)
    #     self.get_logger().info('Publishing: "%d"' % msg.t)  
    #     self.i += 1
    
    def image_callback(self, msg):
        self.get_logger().info('I heard: "%d"' % msg.header.stamp) 
        img_width = msg.width
        img_height = msg.height
        frame = np.array(msg.data).reshape((img_height,img_width))
        self.valid_detected_balls, self.last_detected_balls = track_balls(msg.header.stamp, frame, self.last_detected_balls, self.valid_detected_balls)
        print(self.valid_detected_balls)
        ball_list = []
        for b in self.valid_detected_balls:
            msg = Ball()
            msg.t = b[4]
            msg.x_center = b[0]
            msg.y_center = b[1]
            msg.width = b[2]
            msg.height = b[3]
            ball_list.append(msg)
        msg_list = BallList(ball_list)
        self.balls_publisher_.publish(msg_list)

        
    

def main(args=None):
    rclpy.init(args=args)

    balls_detector = BallsDetector()

    rclpy.spin(balls_detector)

    balls_detector.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
