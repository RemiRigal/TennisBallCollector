#usr/bin/python3
import rclpy
from rclpy.node import Node
from interfaces.msg import BallList
from interfaces.msg import Ball 
from sensor_msgs.msg import Image 
from .scripts.tracking import track_balls, draw_boxes_from_center_coord, show_img
import numpy as np
import cv2

class BallsDetector(Node):

    def __init__(self):
        super().__init__('balls_publisher')
        self.balls_publisher_ = self.create_publisher(BallList, '/ball_list', 10)
        self.cam_subscriber = self.create_subscription(
            Image,                                              
            '/zenith_camera/image_raw',
            self.image_callback,
            10)
        self.last_detected_balls = []
        self.valid_detected_balls = []

    
    def image_callback(self, msg):
        t = msg.header.stamp.sec + 10e-9 * msg.header.stamp.nanosec
        #self.get_logger().info('I heard: "%s"' % str(t)) 
        img_width = msg.width
        img_height = msg.height
        img_step = msg.step # Full row length in bytes
        frame = np.array(msg.data).reshape((img_height,img_width,3)) #RGB images
        # cv2.imshow("image", frame)
        frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        # cv2.imwrite("images/image_" + str(t) + ".png", frame)
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

        
    

def main(args=None):
    rclpy.init(args=args)

    balls_detector = BallsDetector()

    rclpy.spin(balls_detector)

    balls_detector.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
