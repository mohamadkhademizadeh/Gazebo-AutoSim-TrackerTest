import rclpy, subprocess, os, random
from rclpy.node import Node

class Spawner(Node):
    def __init__(self):
        super().__init__('spawner')
        self.targets = int(self.declare_parameter('targets', 5).value)
        self.model = self.declare_parameter('model', 'models/target/model.sdf').value
        self.spawned = False
        self.timer = self.create_timer(1.0, self.on_timer)

    def on_timer(self):
        if self.spawned:
            return
        self.spawned = True
        self.get_logger().info(f"Spawning {self.targets} targets in Gazebo...")
        for i in range(self.targets):
            x = random.uniform(-5, 5); y = random.uniform(-4, 4)
            cmd = [
                'ros2','run','gazebo_ros','spawn_entity.py',
                '-file', self.model,
                '-entity', f'target_{i}',
                '-x', str(x), '-y', str(y), '-z', '0.2'
            ]
            try:
                subprocess.Popen(cmd)
            except Exception as e:
                self.get_logger().warn(f"spawn failed: {e} (ensure gazebo_ros is installed)")

def main():
    rclpy.init()
    n=Spawner(); rclpy.spin(n); rclpy.shutdown()

if __name__=='__main__':
    main()
