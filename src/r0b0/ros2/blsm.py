import os
from r0b0.config import CSR_PEM, KEY_PEM
import rclpy
from rclpy.executors import MultiThreadedExecutor


from r0b0.ros2.page import BlsmPageNode
from r0b0.ros2.robot import BLSM_PAGES_FOLDER, BlsmRobotNode, HeadRobotNode

def main():

    rclpy.init()
    # robot_node = BlsmRobotNode("robot_node")
    robot_node = HeadRobotNode("robot_node")

    page_node = BlsmPageNode("web_page_node",
        template_folder=os.path.join(
            BLSM_PAGES_FOLDER, "templates"),
        static_folder=os.path.join(
            BLSM_PAGES_FOLDER, "static"),
        certfile=CSR_PEM, keyfile=KEY_PEM)

    executor = MultiThreadedExecutor()
    executor.add_node(robot_node)
    executor.add_node(page_node)
    page_node.start()

    try:
        rclpy.spin(robot_node)
        rclpy.spin(page_node)
    except KeyboardInterrupt:
        robot_node.get_logger().info("Shutting down WebPageNode...")
        page_node.get_logger().info("Shutting down WebPageNode...")
    finally:
        robot_node.destroy_node()
        page_node.destroy_node()
        rclpy.shutdown()

if __name__=="__main__":
    main()