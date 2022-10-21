
import rclpy


# from rclpy.executors import MultiThreadedExecutor

from a4vai.path_following.pf_attitude_cmd_module import PFAttitudeCmdModule


def main(args=None):
    rclpy.init(args=args)
    pf_attitude_cmd_module = PFAttitudeCmdModule()
    try:
        rclpy.spin(pf_attitude_cmd_module)
    except KeyboardInterrupt:
        pf_attitude_cmd_module.get_logger().info('Keyboard Interrupt (SIGINT)')
    finally:
        pf_attitude_cmd_module.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
