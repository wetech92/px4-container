import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan
# 수정 S1. 라이브러리 종속성 (Subscriber 수정 내용)
from sensor_msgs.msg import Image
# 수정 M1. 라이브러리 종속성

class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        self.publisher_l = self.create_publisher(LaserScan, 'laser', 10)
        # 수정 P1. publisher 선언 및 publish할 토픽 지정 (Publisher 수정 내용)
        self.publisher_c = self.create_publisher(Image, 'camera', 10)
        # 수정 M2. publisher 선언 및 publish할 토픽 지정

        self.subscription_l = self.create_subscription(
            # String,
            # 'topic',
            LaserScan,
            '/ray/laserscan',
            # 수정 S2. Subscription 대상 토픽
            self.listener_callback_l,
            10)
        self.subscription_l  # prevent unused variable warning
        # 수정 M3. subscription 함수 수정 1


        self.subscription_c = self.create_subscription(
            # String,
            # 'topic',
            Image,
            '/demo_cam/camera1/image_raw',
            # 수정 M4. 추가 Subscription 대상 토픽
            self.listener_callback_c,
            10)
        self.subscription_c  # prevent unused variable warning
        # 수정 M5. subscription 함수 수정 2


    def listener_callback_l(self, sub_msg_l):
        # self.get_logger().info('I heard: "%s"' % msg.data)
        print(sub_msg_l.ranges)
        # 수정 S3. 터미널에 표시할 변수 (float32[ ])
        pub_msg_l = sub_msg_l
        self.publisher_l.publish(pub_msg_l)
        # 수정 M6. listener callback 수정 1


    def listener_callback_c(self, sub_msg_c):
        # self.get_logger().info('I heard: "%s"' % msg.data)
        print(sub_msg_c.height)
        # 수정 M7. 터미널에 표시할 변수 (uint32)
        print(sub_msg_c.width)
        # 수정 M8. 터미널에 표시할 변수 (uint32)
        pub_msg_c = sub_msg_c
        self.publisher_c.publish(pub_msg_c)
        # 수정 M9. listener callback 수정 1


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
