
import rclpy


# from rclpy.executors import MultiThreadedExecutor

from a4vai.collision_avoidance.nmpc_net_module import NMPC_NET_Node


def main(args=None):
    rclpy.init(args=args)
    nmpc_net_module = NMPC_NET_Node()
    try:
        rclpy.spin(nmpc_net_module)
    except KeyboardInterrupt:
        nmpc_net_module.get_logger().info('Keyboard Interrupt (SIGINT)')
    finally:
        nmpc_net_module.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
