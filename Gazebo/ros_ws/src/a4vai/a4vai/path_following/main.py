
import rclpy


# from rclpy.executors import MultiThreadedExecutor

from a4vai.path_following.mppi_module import MPPINode


def main(args=None):
    rclpy.init(args=args)
    mppi_module = MPPINode()
    try:
        rclpy.spin(mppi_module)
    except KeyboardInterrupt:
        mppi_module.get_logger().info('Keyboard Interrupt (SIGINT)')
    finally:
        mppi_module.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
