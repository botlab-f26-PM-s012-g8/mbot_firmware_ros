import rclpy
import time
from rclpy.node import Node
from geometry_msgs.msg import Twist

def main():
    rclpy.init()
    node = Node('simple_step_cmd')
    pub = node.create_publisher(Twist, '/cmd_vel', 10)

    def move(linear_x, angular_z, duration):
        msg = Twist()
        msg.linear.x = float(linear_x)
        msg.angular.z = float(angular_z)

        # Publish for the specified duration
        start_time = time.time()
        while time.time() - start_time < duration:
            pub.publish(msg)
            time.sleep(0.05)

    print("Starting sequence...")

    # 1. Drive forward 0.5 m/s for 4s
    move(0.5, 0.0, 4.0)

    # 2. Wait 1s
    move(0.0, 0.0, 1.0)

    # 3. Rotate CW (negative) for 4s
    move(0.0, -1.0, 4.0)

    # 4. Wait 1s
    move(0.0, 0.0, 1.0)

    # 5. Rotate CCW (positive) for 4s
    move(0.0, 1.0, 4.0)

    # Stop and cleanup
    pub.publish(Twist())
    print("Done.")
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()