import rclpy, random
from rclpy.node import Node
from geometry_msgs.msg import PoseArray, Pose
import numpy as np

class Detector(Node):
    def __init__(self):
        super().__init__('detector')
        self.noise = float(self.declare_parameter('noise', 0.25).value)
        self.fp_rate = float(self.declare_parameter('fp_rate', 0.1).value)
        self.fn_rate = float(self.declare_parameter('fn_rate', 0.05).value)
        self.pub = self.create_publisher(PoseArray, '/detections2d', 10)
        self.sub = self.create_subscription(PoseArray, '/gt/poses', self.on_gt, 10)
        self.get_logger().info("Detector ready")

    def on_gt(self, msg: PoseArray):
        out = PoseArray(); out.header = msg.header
        for p in msg.poses:
            # false negative?
            if np.random.rand() < self.fn_rate:
                continue
            q = Pose()
            q.position.x = float(np.random.normal(p.position.x, self.noise))
            q.position.y = float(np.random.normal(p.position.y, self.noise))
            out.poses.append(q)
        # false positives
        n_fp = np.random.poisson(self.fp_rate * max(1, len(msg.poses)))
        for _ in range(n_fp):
            q = Pose()
            q.position.x = float(np.random.uniform(-8,8))
            q.position.y = float(np.random.uniform(-6,6))
            out.poses.append(q)
        self.pub.publish(out)

def main():
    rclpy.init(); n=Detector(); rclpy.spin(n); rclpy.shutdown()
if __name__=='__main__':
    main()
