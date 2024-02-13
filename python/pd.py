## Example of PD controller for a single motor

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, Float64
from geometry_msgs.msg import WrenchStamped
from sensor_msgs.msg import JointState

import numpy as np


class MyNode(rclpy.node.Node):

    def __init__(self):
        super().__init__('odri2')
        
        # Create a publisher
        self.publisher = self.create_publisher(Int32, 'motor_cmd1', 10)
        
        # Create a subscriber
        self.subscription = self.create_subscription(
            JointState,
            'motor_state',
            self.callback,
            10)
        self.subscription  

        # A counter for demonstration purposes
        self.counter = 0

        # Create a timer to publish messages periodically
        self.timer = self.create_timer(0.001, self.timer_callback)

        self.motor_position = np.zeros(2)
        self.motor_velocity = np.zeros(2)

    def callback(self, msg):
        # self.get_logger().info('I heard joint : "%s"' % msg.wrench.torque.z)
        self.motor_position = np.array(msg.position)
        print(self.motor_position)
        self.motor_velocity = np.array(msg.velocity)

    def timer_callback(self):
        msg = Int32()
        freq = 10000
        amp = 0.5
        des_pos = amp * np.pi * np.sin(2 * np.pi*self.counter/freq)
        des_vel = (1/freq)*2 * np.pi* amp * np.pi * np.sin(2 * np.pi*self.counter/freq)
        msg.data = 300#int(2500 * (des_pos - self.motor_position[0]) + 35 * (des_vel  - self.motor_velocity[0]))
        self.publisher.publish(msg)
        # self.get_logger().info(f"Publishing: {msg.data}")
        self.counter += 1


def main(args=None):
    rclpy.init(args=args)

    my_node = MyNode()

    try:
        rclpy.spin(my_node)
    except KeyboardInterrupt:
        pass

    my_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()