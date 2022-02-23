from turtle import pos
import rclpy
from rclpy.timer import Rate
from rclpy.node import Node
from geometry_msgs.msg import Twist, Vector3, Point, PoseStamped,Pose
from gazebo_msgs.msg import ModelStates
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float64
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
        self.target = Pose()
        self.p_robot = Pose()
        self.th_robot = 0.0

        self.true_th = 0.0
        self.true_p = Point()

        self.cmd_vel_publisher = self.create_publisher(Twist,"cmd_vel",10)
        self.laser_scan_subscriber = self.create_subscription(LaserScan,"scan",self.laser_scan_callback,10)

        self.target_subscriber = self.create_subscription(PoseStamped,"target",self.target_callback,10)
        self.p_robot_subscriber = self.create_subscription(PoseStamped,"robot_pose",self.target_callback,10)

        self.true_th_subscriber = self.create_subscription(ModelStates,"model_states",self.true_th_callback,10)

    def true_th_callback(self,msg) :
        id_robot = msg.name.index('robochon')
        quat_robot = msg.pose[id_robot].orientation
        self.true_p = msg.pose[id_robot].position
        self.true_th = euler_from_quaternion(quat_robot.x,quat_robot.y,quat_robot.z,quat_robot.w)


    def laser_scan_callback(self,msg) :
        self.lidar_ranges = msg.ranges
    
    def target_callback(self,msg) :
        self.target = msg.pose

    def p_robot_callback(self,msg) :
        self.p_robot = msg.pose
        self.th_robot = euler_from_quaternion(self.p_robot.orientation.x,self.p_robot.orientation.y,self.p_robot.orientation.z,self.p_robot.orientation.w)



    def control(self,v) :
        th_robot = self.th_robot

        px = np.array([[self.p_robot.position.x],[self.p_robot.position.y]])
        #px = np.array([[self.true_p.x],[self.true_p.y]])

        phat = np.array([[self.target.position.x],[self.target.position.y]])
        #phat = np.array([[0.0],[10.0]])

        w = px-phat#consigne
        vbar = 0.1*np.linalg.norm(w)

        th_bar = np.arctan2(w[1,0],w[0,0])
        e_th = sawtooth(th_bar-th_robot)
    
        u1 = v + np.arctan(0.5*(vbar-v)) #commande vitesse
        # u2 = np.arctan(np.tan((thbar-th_robot)/2)) #commande orientation
        u2 = -0.6*np.arctan(e_th) #cap desire
        return u1,u2

def main(args=None):

    rclpy.init(args=args)
    n = Control()

    v = 0.8
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
