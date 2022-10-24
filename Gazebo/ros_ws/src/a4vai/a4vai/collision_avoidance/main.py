
import rclpy


# from rclpy.executors import MultiThreadedExecutor

from a4vai.collision_avoidance.nmpc_net_module import NMPC_NET_Node


def main(args=None):
    rclpy.init(args=args)
    JBNU_module = NMPC_NET_Node()
    try:
        rclpy.spin(JBNU_module)
    except KeyboardInterrupt:
        JBNU_module.get_logger().info('Keyboard Interrupt (SIGINT)')
    finally:
        JBNU_module.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
