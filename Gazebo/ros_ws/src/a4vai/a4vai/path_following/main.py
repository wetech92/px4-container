
import rclpy
import threading

# from rclpy.executors import MultiThreadedExecutor

from a4vai.path_following.pf_attitude_cmd_module import PFAttitudeCmdModule
from a4vai.path_following.pf_gpr_module import PF_GPR_Module
from a4vai.path_following.pf_guid_param_module import PFGuidModule


def main(args=None):
    rclpy.init(args=args)
    
    pf_attitude_cmd_module = PFAttitudeCmdModule()
    print("*************** Debug 1 ***************")
    # pf_gpr_module = PF_GPR_Module()
    # pf_guid_module = PFGuidModule()
    # executor = rclpy.executors.MultiThreadedExecutor()
    # print("*************** Debug 2 ***************")
    # # executor.add_node(pf_attitude_cmd_module)
    # # executor.add_node(pf_gpr_module)
    # # executor.add_node(pf_guid_module)
    # print("*************** Debug 3 ***************")
    # # executor_thread = threading.Thread(target=executor.spin, daemon=True)
    # print("*************** Debug 4 ***************")
    # try:
    #     # executor_thread.start()
    #     print("*************** Debug 5 ***************")
    #     # rclpy.spin(pf_attitude_cmd_module)
    #     # rclpy.spin(pf_gpr_module)
    #     # rclpy.spin(pf_guid_module)
    # except Exception as e:
    #                 executor_thread.get_logger().info(
    #                     'Multi Thread Start failed %r' % (e,))
    # finally:
    #     # pf_gpr_module.destroy_node()
    #     print("*************** Debug 6 ***************")
    #     # executor.shutdown()
    #     # rclpy.shutdown()
    try : 
        rclpy.spin(pf_attitude_cmd_module)
        # rclpy.spin(pf_guid_module)
    except Exception as e:
                    pf_attitude_cmd_module.get_logger().info(
                        'MPPI module Start failed %r' % (e,))
    finally :
        # pf_attitude_cmd_module.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
