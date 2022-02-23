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

        self.true_th = 0.0
        self.true_p = Point()

        self.phat = np.array([[0.0],[3.0]])
        self.true_balls = []
        self.num_ball = 0
        self.memo_num_ball = 0
        self.flag_zone  = 0

        self.nb_balls = 10
        self.memo_nb_balls = 7

        self.cmd_vel_publisher = self.create_publisher(Twist,"cmd_vel",10)
        self.laser_scan_subscriber = self.create_subscription(LaserScan,"scan",self.laser_scan_callback,10)

        self.true_th_subscriber = self.create_subscription(ModelStates,"model_states",self.true_th_callback,10)

    def true_th_callback(self,msg) :
        id_robot = msg.name.index('robochon')
        quat_robot = msg.pose[id_robot].orientation
        self.true_p = msg.pose[id_robot].position
        self.th_true = euler_from_quaternion(quat_robot.x,quat_robot.y,quat_robot.z,quat_robot.w)
        l_names = msg.name
        self.true_balls = []
        for i in range (len(l_names)) :
            if l_names[i][0] == 'b' :
                self.true_balls.append(msg.pose[i].position)
        self.nb_balls = len(self.true_balls)


    def laser_scan_callback(self,msg) :
        self.lidar_ranges = msg.ranges


    def control(self,v) :

        th_robot = self.th_true
        px = np.array([[self.true_p.x],[self.true_p.y]])

        current_ball  = self.true_balls[self.num_ball]
        
        if self.flag_zone == 0 :
            self.phat = np.array([[current_ball.x],[current_ball.y]])


        if self.memo_num_ball != self.num_ball and self.flag_zone == 0 : 
            self.flag_zone = 1
            self.phat = np.array([[4.6],[10.1]])



        w = px-self.phat #distance robot-balle
        w_norm = np.linalg.norm(w)

        if w_norm < 0.6 and self.flag_zone == 0:
            self.num_ball+=1
            print("go near zone")

        if w_norm < 0.6 and self.flag_zone == 1:
            self.flag_zone = 2
            self.phat = np.array([[6.4],[13.3]])
            w = px-self.phat
            w_norm = np.linalg.norm(w)
        
        if w_norm < 0.6 and self.flag_zone == 2:
            self.flag_zone = 3
            self.phat = np.array([[4.6],[10.1]])
            w = px-self.phat
            w_norm = np.linalg.norm(w)

        if w_norm < 1.0 and self.flag_zone == 3:
            self.flag_zone = 0 
            self.memo_num_ball += 1
        

        vbar = 0.075*w_norm
        u1 = v + np.arctan(0.5*(vbar-v)) #commande vitesse


        th_bar = np.arctan2(w[1,0],w[0,0])
        e_th = sawtooth(th_bar-th_robot)
        u2 = -0.3*np.arctan(e_th) #cap desire

        if np.pi-abs(e_th) > 0.1:
            u1 = 0.05 
            u2 = u2*1.5
        else :
            u1 = 0.5 + np.arctan(0.5*(vbar-v))

        if u1 > 1.0 :
            u1 = 1.0

        if self.flag_zone == 3 :
            if self.memo_nb_balls == self.nb_balls :
                u1 = 0.0
                u2 = 0.0
            else : 
                self.memo_nb_balls = self.nb_balls
                u1 = -0.5
                u2 = 0.0

        print(self.memo_nb_balls, " ",self.nb_balls, " flag_zone :",self.flag_zone)
        return u1,u2

def main(args=None):

    rclpy.init(args=args)
    n = Control()
    v = 0.5
    th = 0.0

    first = 0

    while rclpy.ok():
        rclpy.spin_once(n)

        v,th = n.control(v) 
        cmd_fw = Twist(linear=Vector3(x=v),angular=Vector3(z=th))
        n.cmd_vel_publisher.publish(cmd_fw)

        if first ==0 :
            n.memo_nb_balls = n.nb_balls
            first = 1

        



    # rclpy.spin(n)

    n.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
