#!/usr/bin/env python3

"""Handover Action Client for ROS2."""

import argparse
import json
import os
import sys

import rclpy
from inventory_interfaces.action import Handover
from inventory_interfaces.msg import Inventory, Location
from inventory_interfaces_utils.inventory_interfaces_utils import (
    HandoverFeedbackEnum,
    HandoverResultEnum,
)
from rclpy.action import ActionClient
from rclpy.executors import SingleThreadedExecutor
from rclpy.logging import LoggingSeverity
from rclpy.node import Node
from rclpy.task import Future


def inventory_from_json(json_path: str) -> Inventory:
    """
    Load Inventory from a JSON file.

    Args:
        json_path: Path to the JSON file containing inventory data.

    Returns:
        Inventory message populated with data from JSON file.

    Raises:
        FileNotFoundError: If the JSON file doesn't exist.
        json.JSONDecodeError: If the JSON file is malformed.
    """
    with open(json_path, "r") as f:
        data = json.load(f)

    # Use ROS message's from_dict if available, else manual mapping
    inv = Inventory()
    inv.root_path = data.get("root_path", "")

    # For locations, you may need to recursively build Location/Container/Item, etc.
    # Here we assume locations is a list of dicts that can be assigned directly if fields match
    if hasattr(inv, "locations"):
        inv.locations = []
        for loc in data.get("locations", []):
            loc_msg = Location()
            for k, v in loc.items():
                if hasattr(loc_msg, k):
                    setattr(loc_msg, k, v)
            inv.locations.append(loc_msg)

    return inv


class HandoverActionClient(Node):
    """ROS2 Action Client for Handover action."""

    def __init__(self, node_name: str = "handover_client", action_server_name: str = "handover") -> None:
        """
        Initialize the Handover action client.

        Args:
            node_name: Name for the ROS2 node.
            action_server_name: Name of the action server to connect to.
        """
        super().__init__(node_name)
        self._action_client = ActionClient(self, Handover, action_server_name)
        self.get_logger().info(f"Handover Action Client initialized for server '{action_server_name}'")

    def send_goal(self, handover_reference: str, inventory: Inventory, consume_into: bool) -> bool:
        """
        Send goal to the action server.

        Args:
            handover_reference: Reference identifier for the handover.
            inventory: Inventory data to handover.
            consume_into: Whether to consume into the target location.

        Returns:
            True if goal was sent successfully, False otherwise.
        """
        if not self._action_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error("Action server not available after waiting")
            return False

        goal_msg = Handover.Goal()
        goal_msg.handover_reference = handover_reference
        goal_msg.inventory = inventory
        goal_msg.consume_into = consume_into

        self.get_logger().info(f"Sending HANDOVER goal for reference: {handover_reference}")

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
            status_enum = HandoverFeedbackEnum(feedback.status)
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
            result_enum = HandoverResultEnum(result.result)
            result_name = result_enum.name
        except ValueError:
            result_name = f"UNKNOWN_RESULT({result.result})"

        if result.result == HandoverResultEnum.SUCCESS:
            self.get_logger().info("Action completed successfully!")
            self.get_logger().info(f"Result: {result_name} ({result.result})")
        else:
            self.get_logger().error(f"Action failed: {result_name} ({result.result}) - {result.message}")

        rclpy.shutdown()


def main() -> None:
    """Main function with argument parsing and execution."""
    parser = argparse.ArgumentParser(
        description="ROS2 Action Client for Handover",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  python3 handover.py
    --handover-reference ref1
    --inventory-json inventory.json
    --consume-into
    --action-server-name /buffer/handover
        """,
    )
    parser.add_argument("--handover-reference", "-r", type=str, required=True, help="Reference for the handover")
    parser.add_argument("--inventory-json", "-j", type=str, required=True, help="Path to inventory JSON file")
    parser.add_argument("--consume-into", "-c", action="store_true", help="Consume into (default: False)")
    parser.add_argument("--action-server-name", "-s", type=str, required=True, help="Name of the Handover action server")
    parser.add_argument("--node-name", "-n", type=str, default="handover_client", help="Name for the ROS2 node")
    parser.add_argument(
        "--log-level", "-l", type=str, choices=["debug", "info", "warn", "error"], default="info", help="Logging level"
    )

    args = parser.parse_args()

    if not os.path.isfile(args.inventory_json):
        print(f"Inventory JSON file not found: {args.inventory_json}", file=sys.stderr)
        sys.exit(1)

    try:
        inventory = inventory_from_json(args.inventory_json)
    except (FileNotFoundError, json.JSONDecodeError) as e:
        print(f"Error loading inventory JSON: {e}", file=sys.stderr)
        sys.exit(1)

    rclpy.init(args=None)

    try:
        client_node = HandoverActionClient(args.node_name, args.action_server_name)

        # Set log level
        log_level_map = {
            "debug": LoggingSeverity.DEBUG,
            "info": LoggingSeverity.INFO,
            "warn": LoggingSeverity.WARN,
            "error": LoggingSeverity.ERROR,
        }
        client_node.get_logger().set_level(log_level_map[args.log_level])

        success = client_node.send_goal(
            handover_reference=args.handover_reference,
            inventory=inventory,
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
