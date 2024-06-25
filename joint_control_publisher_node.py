import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_msgs.msg import Header
import numpy as np
import random

class wall():
    def __init__(self,px , py, rx ,ry ) -> None:
        self.px = px
        self.py = py
        self.rx = rx
        self.ry = ry

class JointControlPublisher(Node):

    def __init__(self):
        super().__init__('joint_trajectory_publisher')
        self.publisher_ = self.create_publisher(JointTrajectory, '/set_joint_trajectory', 10)
        timer_period = 0.3 # seconds (change as needed)
        self.timer = self.create_timer(timer_period, self.timer_callback)

        w1 = wall(-1.7, 1, 4, 0.4)
        w2 = wall(-3.4, 1.03, 0.4, 3.3)
        w3 = wall(-1.7, -3.2, 0.4, 2.9)
        w4 = wall(-1.7, -3.2, 3.8, 0.4)
        w5 = wall(-0.4, -5.5, 0.4, 0.7)
        self.wall_ls = [w1,w2,w3,w4,w5]
        self.x_lower = -5.6
        self.x_upper = 2.3
        self.y_lower = -5.4
        self.y_upper = 4.3
        self. counter = 0

    def timer_callback(self):
        msg = JointTrajectory()
        msg.header = Header()
        msg.header.frame_id = "footprint_link"
        msg.joint_names = ["x_pose", "y_pose", "rot_joint"]

        point = JointTrajectoryPoint()
        x_pos, y_pos, orientation = self.new_state()
        point.positions = [x_pos, y_pos, orientation]
        point.time_from_start.sec = 1  # Change as needed

        msg.points = [point]
        self.publisher_.publish(msg)
        self.counter += 1
        self.get_logger().info('Publishing joint trajectory, counter = '+str(self.counter))
    def new_state(self):
        orientation  = random.uniform(-np.pi, np.pi)
        while True:
            x = random.uniform(self.x_lower, self.x_upper)
            y = random.uniform(self.y_lower, self.y_upper)
            if self.check_not_inside_walls(x,y):
                break

        return x,y, orientation
    
       
    def check_not_inside_walls(self, x,y):
        out = True
        for wall in self.wall_ls:
            if wall.px<x<wall.px+wall.rx and wall.py <y<wall.py+ wall.ry:
                out = False
                return out
        return out
            
def main(args=None):
    rclpy.init(args=args)
    joint_control_publisher = JointControlPublisher()
    rclpy.spin(joint_control_publisher)
    joint_control_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
