import rclpy
from rclpy.node import Node

# from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
# 수정 S1. 라이브러리 종속성 (Subscriber 수정 내용)


class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        self.publisher = self.create_publisher(Image, 'camera', 10)
        # 수정 P1. publisher 선언 및 publish할 토픽 지정


        self.subscription = self.create_subscription(
            # String,
            # 'topic',
            Image,
            '/demo_cam/camera1/image_raw',
            # 수정 S2. Subscription 대상 토픽 (Subscriber 수정 내용)
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning


    def listener_callback(self, msg):
        # self.get_logger().info('I heard: "%s"' % msg.data)
        print(msg.height)
        print(msg.width)
        # 터미널에 표시할 변수
        pub_msg = msg
        self.publisher.publish(pub_msg)
        # publish 할 메세지에 subscribe 한 값 넣고 Publish


def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
