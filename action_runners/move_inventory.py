#!/usr/bin/env python3

"""MoveInventory Action Client for ROS2."""

import argparse
import sys

import rclpy
from inventory_interfaces.action import MoveInventory
from inventory_interfaces_utils.inventory_interfaces_utils import (
    MoveInventoryFeedbackEnum,
    MoveInventoryResultEnum,
)
from rclpy.action import ActionClient
from rclpy.executors import SingleThreadedExecutor
from rclpy.logging import LoggingSeverity
from rclpy.node import Node
from rclpy.task import Future


class MoveInventoryActionClient(Node):
    """ROS2 Action Client for MoveInventory action."""

    def __init__(self, node_name: str = "move_inventory_client", action_server_name: str = "move_inventory") -> None:
        """
        Initialize the MoveInventory action client.

        Args:
            node_name: Name for the ROS2 node.
            action_server_name: Name of the action server to connect to.
        """
        super().__init__(node_name)
        self._action_client = ActionClient(self, MoveInventory, action_server_name)
        self.get_logger().info(f"MoveInventory Action Client initialized for server '{action_server_name}'")

    def send_goal(self, source: str, target: str, quantity: float, consume_into: bool) -> bool:
        """
        Send goal to the action server.

        Args:
            source: Source location for the inventory move.
            target: Target location for the inventory move.
            quantity: Quantity to move (-1 for all).
            consume_into: Whether to consume into target location.

        Returns:
            True if goal was sent successfully, False otherwise.
        """
        if not self._action_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error("Action server not available after waiting")
            return False

        goal_msg = MoveInventory.Goal()
        goal_msg.source = source
        goal_msg.target = target
        goal_msg.quantity = quantity
        goal_msg.consume_into = consume_into

        self.get_logger().info(f"Sending MOVE goal: {source} -> {target}, quantity={quantity}, consume_into={consume_into}")

        self._send_goal_future = self._action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        self._send_goal_future.add_done_callback(self.goal_response_callback)
        return True

    def goal_response_callback(self, future: Future) -> None:
        """
        Handle goal response from action server.

        Args:
            future: Future containing the goal handle.
        """
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().error("Goal rejected by action server")
            return

        self.get_logger().info("Goal accepted by action server")

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def feedback_callback(self, feedback_msg) -> None:
        """
        Handle feedback from action server.

        Args:
            feedback_msg: Feedback message from the action server.
        """
        feedback = feedback_msg.feedback

        try:
            status_enum = MoveInventoryFeedbackEnum(feedback.status)
            status_name = status_enum.name
        except ValueError:
            status_name = f"UNKNOWN_STATUS({feedback.status})"

        self.get_logger().info(f"Received feedback - Status: {status_name} ({feedback.status})")

    def get_result_callback(self, future: Future) -> None:
        """
        Handle result from action server.

        Args:
            future: Future containing the action result.
        """
        result = future.result().result

        try:
            result_enum = MoveInventoryResultEnum(result.result)
            result_name = result_enum.name
        except ValueError:
            result_name = f"UNKNOWN_RESULT({result.result})"

        if result.result == MoveInventoryResultEnum.SUCCESS:
            self.get_logger().info("Action completed successfully!")
            self.get_logger().info(f"Result: {result_name} ({result.result})")
        else:
            self.get_logger().error(f"Action failed: {result_name} ({result.result}) - {result.message}")

        rclpy.shutdown()


def main() -> None:
    """Main function with argument parsing and execution."""
    parser = argparse.ArgumentParser(
        description="ROS2 Action Client for MoveInventory",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  python3 move_inventory.py --source src1 --target tgt1 --quantity -1.0 --consume-into --action-server-name /mod/move_inventory
        """,
    )
    parser.add_argument("--source", "-s", type=str, required=True, help="Source location")
    parser.add_argument("--target", "-t", type=str, required=True, help="Target location")
    parser.add_argument("--quantity", "-q", type=float, default=-1.0, help="Quantity to move (default: -1.0 for all)")
    parser.add_argument("--consume-into", "-c", action="store_true", help="Consume into (default: False)")
    parser.add_argument(
        "--action-server-name", "-a", type=str, required=True, help="Name of the MoveInventory action server"
    )
    parser.add_argument("--node-name", "-n", type=str, default="move_inventory_client", help="Name for the ROS2 node")
    parser.add_argument(
        "--log-level", "-l", type=str, choices=["debug", "info", "warn", "error"], default="info", help="Logging level"
    )

    args = parser.parse_args()

    rclpy.init(args=None)

    try:
        client_node = MoveInventoryActionClient(args.node_name, args.action_server_name)

        # Set log level
        log_level_map = {
            "debug": LoggingSeverity.DEBUG,
            "info": LoggingSeverity.INFO,
            "warn": LoggingSeverity.WARN,
            "error": LoggingSeverity.ERROR,
        }
        client_node.get_logger().set_level(log_level_map[args.log_level])

        success = client_node.send_goal(
            source=args.source,
            target=args.target,
            quantity=args.quantity,
            consume_into=args.consume_into,
        )

        if not success:
            sys.exit(2)

        executor = SingleThreadedExecutor()
        executor.add_node(client_node)

        try:
            executor.spin()
        except KeyboardInterrupt:
            client_node.get_logger().info("Interrupted by user")
        finally:
            executor.shutdown()
            client_node.destroy_node()

    except Exception as e:
        print(f"Error: {e}", file=sys.stderr)
        sys.exit(1)

    finally:
        try:
            rclpy.shutdown()
        except Exception as e:
            print(f"Error shutting down rclpy: {e}", file=sys.stderr)
            sys.exit(1)


if __name__ == "__main__":
    main()
