
import rclpy


# from rclpy.executors import MultiThreadedExecutor

from a4vai.controller.controller import ControllerNode
from a4vai.controller.path_plan_service import PathPlanningService


def main(args=None):
    rclpy.init(args=args)
    Controller = ControllerNode()    
    try:
        rclpy.spin(Controller)
    except KeyboardInterrupt:
        Controller.get_logger().info('Keyboard Interrupt (SIGINT)')
    finally:
        Controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
