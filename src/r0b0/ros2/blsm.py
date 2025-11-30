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
from r0b0.utils.cert_manager import ensure_https_certificates


def main():
    rclpy.init()
    robot_node = BlsmRobotNode(
        name="robot_node",
        # motor_map=DEG2SERVO,
        motor_map=DEG2DXL,
        port=os.environ.get("BLSM_PORT", "/dev/ttyACM0"),
    )
    # robot_node = HeadRobotNode("robot_node")
    page_kwargs = {
        "name": "web_page_node",
        "template_folder": os.path.join(BLSM_PAGES_FOLDER, "templates"),
        "static_folder": os.path.join(BLSM_PAGES_FOLDER, "static"),
    }

    # Ensure HTTPS certificates exist (auto-generate if needed)
    try:
        cert_path, key_path = ensure_https_certificates(CSR_PEM, KEY_PEM)
        print(f"✓ HTTPS enabled with certificates: {cert_path}")
    except Exception as e:
        print(f"⚠ Warning: Could not set up HTTPS certificates: {e}")
        print("  Continuing without HTTPS...")
        cert_path, key_path = None, None

    if cert_path and key_path:
        page_kwargs.update(
            {"certfile": str(cert_path), "keyfile": str(key_path)}
        )

    page_node = BlsmPageNode(**page_kwargs)

    executor = MultiThreadedExecutor()
    executor.add_node(robot_node)
    executor.add_node(page_node)
    page_node.start()

    try:
        executor.spin()
        # rclpy.spin(page_node)
        # rclpy.spin(robot_node)

    except KeyboardInterrupt:
        robot_node.get_logger().info("Shutting down WebPageNode...")
        page_node.get_logger().info("Shutting down WebPageNode...")
    finally:
        robot_node.destroy_node()
        page_node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
