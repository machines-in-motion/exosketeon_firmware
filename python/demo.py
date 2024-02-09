import rclpy
from std_msgs.msg import String

def callback(msg):
    print("I heard: {}".format(msg.data))

def main(args=None):
    rclpy.init(args=args)

    # Create a ROS 2 node
    node = rclpy.create_node('simple_subscriber')

    # Create a subscriber for the 'chatter' topic, using the String message type
    subscription = node.create_subscription(
        String,
        'chatter',
        callback,
        10  # QoS profile depth
    )
    subscription  # prevent unused variable warning

    print("Listening for messages on 'chatter' topic. Press Ctrl+C to exit.")

    # Spin the node so the callback is called
    rclpy.spin(node)

    # Clean up when the node is shutdown
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()