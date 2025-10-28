import rclpy, random, math
from rclpy.node import Node
from geometry_msgs.msg import PoseArray, Pose
import numpy as np

class Mover(Node):
    def __init__(self):
        super().__init__('mover')
        self.targets = int(self.declare_parameter('targets', 5).value)
        self.rate = float(self.declare_parameter('rate', 10.0).value)
        self.world = [-8, 8, -6, 6]
        self.dt = 1.0/self.rate
        self.pub = self.create_publisher(PoseArray, '/gt/poses', 10)
        rng = np.random.default_rng(123)
        self.state = []
        for i in range(self.targets):
            x=rng.uniform(-5,5); y=rng.uniform(-3,3); vx=rng.uniform(-1,1); vy=rng.uniform(-1,1)
            self.state.append(np.array([x,y,vx,vy], dtype=float))
        self.timer = self.create_timer(self.dt, self.on_timer)
        self.get_logger().info(f"Mover started: {self.targets} targets @ {self.rate} Hz")

    def on_timer(self):
        pa = PoseArray(); pa.header.frame_id='map'
        for i in range(len(self.state)):
            s=self.state[i]
            s[0]+=s[2]*self.dt; s[1]+=s[3]*self.dt
            if not (self.world[0] < s[0] < self.world[1]): s[2]*=-1.0
            if not (self.world[2] < s[1] < self.world[3]): s[3]*=-1.0
            p=Pose(); p.position.x=float(s[0]); p.position.y=float(s[1]); pa.poses.append(p)
        self.pub.publish(pa)

def main():
    rclpy.init(); n=Mover(); rclpy.spin(n); rclpy.shutdown()
if __name__=='__main__':
    main()
