import os
from r0b0.config import CSR_PEM, KEY_PEM
import rclpy
from rclpy.executors import MultiThreadedExecutor


from r0b0.ros2.page import BlsmPageNode
from r0b0.ros2.robot import (
    BLSM_PAGES_FOLDER,
    BlsmRobotNode,
    DEG2DXL,
    DEG2SERVO,
)


def main():
    rclpy.init()
    robot_node = BlsmRobotNode(
        name="robot_node",
        # motor_map=DEG2SERVO,
        motor_map=DEG2DXL,
        # port="/dev/serial0"
        port=os.environ.get("BLSM_PORT", "/dev/ttyACM0"),
    )
    # robot_node = HeadRobotNode("robot_node")
    page_kwargs = {
        "name": "web_page_node",
        "template_folder": os.path.join(BLSM_PAGES_FOLDER, "templates"),
        "static_folder": os.path.join(BLSM_PAGES_FOLDER, "static"),
    }
    if os.path.exists(CSR_PEM) and os.path.exists(KEY_PEM):
        page_kwargs.update({"certfile": CSR_PEM, "keyfile": KEY_PEM})

    page_node = BlsmPageNode(**page_kwargs)

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


if __name__ == "__main__":
    main()
