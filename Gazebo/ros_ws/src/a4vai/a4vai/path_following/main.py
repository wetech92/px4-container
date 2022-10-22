
import rclpy
import threading

# from rclpy.executors import MultiThreadedExecutor

from a4vai.path_following.pf_attitude_cmd_module import PFAttitudeCmdModule
from a4vai.path_following.pf_gpr_module import PF_GPR_Module
from a4vai.path_following.pf_guid_param_module import PFGuidModule


def main(args=None):
    rclpy.init(args=args)
    pf_attitude_cmd_module = PFAttitudeCmdModule()
    pf_gpr_module = PF_GPR_Module()
    pf_guid_module = PFGuidModule()
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(pf_attitude_cmd_module)
    executor.add_node(pf_gpr_module)
    executor.add_node(pf_guid_module)
    try:
        executor_thread = threading.Thread(target=executor.spin, daemon=True)
        executor_thread.start()
        # rclpy.spin(pf_attitude_cmd_module)
        # rclpy.spin(pf_gpr_module)
        # rclpy.spin(pf_guid_module)
    except KeyboardInterrupt:
        pf_attitude_cmd_module.get_logger().info('Keyboard Interrupt (SIGINT)')
        pf_gpr_module.get_logger().info('Keyboard Interrupt (SIGINT)')
        pf_guid_module.get_logger().info('Keyboard Interrupt (SIGINT)')
    finally:
        pf_attitude_cmd_module.destroy_node()
        pf_gpr_module.destroy_node()
        pf_guid_module.destroy_node()
        executor.shutdown()
        # rclpy.shutdown()


if __name__ == '__main__':
    main()
