from turtle import pos
import rclpy
from rclpy.timer import Rate
from rclpy.node import Node
from geometry_msgs.msg import Twist, Vector3,PoseStamped,Pose,PoseArray
from sensor_msgs.msg import LaserScan
import numpy as np
import math

def sawtooth(x):
    return (x+np.pi)%(2*np.pi)-np.pi   # or equivalently   2*arctan(tan(x/2))

def euler_from_quaternion(x, y, z, w):
        """
        Convert a quaternion into euler angles (roll, pitch, yaw)
        roll is rotation around x in radians (counterclockwise)
        pitch is rotation around y in radians (counterclockwise)
        yaw is rotation around z in radians (counterclockwise)
        """
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)
     
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)
     
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)
     
        return yaw_z # in radians

class Control(Node):

    def __init__(self):
        super().__init__("control_node")
        self.lidar_ranges = []

        self.target = PoseArray()
        self.p_robot = Pose()
        self.th_robot = 0.0

        self.balls = []
        self.num_ball = 0

        self.cmd_vel_publisher = self.create_publisher(Twist,"cmd_vel",10)
        self.laser_scan_subscriber = self.create_subscription(LaserScan,"scan",self.laser_scan_callback,10)

        self.target_subscriber = self.create_subscription(PoseArray,"target",self.target_callback,10)

    def laser_scan_callback(self,msg) :
        self.lidar_ranges = msg.ranges
    
    def target_callback(self,msg) :
        quat_robot = msg.poses[0].orientation
        self.p_robot = msg.pose[0].position
        self.th_robot = euler_from_quaternion(quat_robot.x,quat_robot.y,quat_robot.z,quat_robot.w)

        self.balls = msg.poses[1:]

    def control(self,v) :
        th_robot = self.th_robot

        px = np.array([[self.p_robot.x],[self.p_robot.y]])


        current_ball  = self.balls[self.num_ball]
        phat = np.array([[current_ball.position.x],[current_ball.position.y]])

        w = px-phat #ecart entre la balle et le robot
        w_norm = np.linalg.norm(w)
        if w_norm < 0.8 :
            self.num_ball+=1
        
        print(self.num_ball, " ", w_norm)

        vbar = 0.1*w_norm

        th_bar = np.arctan2(w[1,0],w[0,0])
        e_th = sawtooth(th_bar-th_robot)

        u1 = v + np.arctan(0.5*(vbar-v)) #commande vitesse
        u2 = -0.6*np.arctan(e_th) #cap desire

        return u1,u2

def main(args=None):

    rclpy.init(args=args)
    n = Control()

    v = 0.5
    th = 0.0

    while rclpy.ok():
        rclpy.spin_once(n)

        v,th = n.control(v) 
        cmd_fw = Twist(linear=Vector3(x=v),angular=Vector3(z=th))
        n.cmd_vel_publisher.publish(cmd_fw)

        



    # rclpy.spin(n)

    n.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
