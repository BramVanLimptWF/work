#!/usr/bin/env python3

"""RunProcess Action Client for ROS2."""

import argparse
import sys
from typing import List

import rclpy
from diagnostic_msgs.msg import KeyValue
from process_control_interfaces.action import RunProcess
from process_control_interfaces_utils.process_control_interfaces_utils import (
    RunProcessFeedbackEnum,
    RunProcessResultEnum,
)
from rclpy.action import ActionClient
from rclpy.executors import SingleThreadedExecutor
from rclpy.logging import LoggingSeverity
from rclpy.node import Node
from rclpy.task import Future


class RunProcessActionClient(Node):
    """ROS2 Action Client for RunProcess action."""

    def __init__(self, node_name: str = "run_process_client", action_server_name: str = "run_process") -> None:
        """
        Initialize the RunProcess action client.

        Args:
            node_name: Name for the ROS2 node.
            action_server_name: Name of the action server to connect to.
        """
        super().__init__(node_name)
        self._action_client = ActionClient(self, RunProcess, action_server_name)
        self.get_logger().info(f"RunProcess Action Client initialized for server '{action_server_name}'")

    def send_goal(self, process_name: str, fields: List[KeyValue]) -> bool:
        """
        Send goal to the action server.

        Args:
            process_name: Name of the process to run.
            fields: List of key-value pairs for process configuration.

        Returns:
            True if goal was sent successfully, False otherwise.
        """
        if not self._action_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error("Action server not available after waiting")
            return False

        # Create goal message
        goal_msg = RunProcess.Goal()
        goal_msg.process_name = process_name
        goal_msg.fields = fields

        self.get_logger().info(f"Sending RUN goal for process: {process_name}")
        if fields:
            self.get_logger().info(f"Fields: {[f'{kv.key}={kv.value}' for kv in fields]}")

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
            status_enum = RunProcessFeedbackEnum(feedback.status)
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
            result_enum = RunProcessResultEnum(result.result)
            result_name = result_enum.name
        except ValueError:
            result_name = f"UNKNOWN_RESULT({result.result})"

        if result.result == RunProcessResultEnum.SUCCESS:
            self.get_logger().info("Action completed successfully!")
            self.get_logger().info(f"Result: {result_name} ({result.result})")
            if result.message:
                self.get_logger().info(f"Message: {result.message}")
        else:
            self.get_logger().error("Action failed!")
            self.get_logger().error(f"Result: {result_name} ({result.result})")
            if result.message:
                self.get_logger().error(f"Error message: {result.message}")

        rclpy.shutdown()


def parse_fields(fields_list: List[str]) -> List[KeyValue]:
    """
    Parse field strings into KeyValue messages.

    Args:
        fields_list: List of strings in format "key=value".

    Returns:
        List of KeyValue messages.

    Raises:
        SystemExit: If field format is invalid.
    """
    fields = []
    for item in fields_list:
        if "=" not in item:
            print(f"Invalid field format: '{item}'. Use key=value.", file=sys.stderr)
            sys.exit(1)

        key, value = item.split("=", 1)
        kv = KeyValue()
        kv.key = key
        kv.value = value
        fields.append(kv)

    return fields


def main() -> None:
    """Main function with argument parsing and execution."""
    parser = argparse.ArgumentParser(
        description="ROS2 Action Client for RunProcess",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Run a process by name
  python3 run_process.py --process-name process_001 --action-server-name /conveyor_1/run_process

  # Run a process with fields
  python3 run_process.py --process-name process_001 --fields key1=val1 key2=val2 --action-server-name /conveyor_1/run_process
        """,
    )
    parser.add_argument("--process-name", "-p", type=str, required=True, help="Name of the process to run")
    parser.add_argument("--fields", "-f", nargs="*", default=[], help="Fields as key=value pairs (optional)")
    parser.add_argument(
        "--action-server-name",
        "-s",
        type=str,
        required=True,
        help="Name of the RunProcess action server (e.g., /conveyor_1/run_process)",
    )
    parser.add_argument(
        "--node-name",
        "-n",
        type=str,
        default="run_process_client",
        help="Name for the ROS2 node (default: run_process_client)",
    )
    parser.add_argument(
        "--log-level",
        "-l",
        type=str,
        choices=["debug", "info", "warn", "error"],
        default="info",
        help="Logging level (default: info)",
    )

    args = parser.parse_args()

    fields = parse_fields(args.fields)

    rclpy.init(args=None)

    try:
        client_node = RunProcessActionClient(args.node_name, args.action_server_name)

        # Set log level
        log_level_map = {
            "debug": LoggingSeverity.DEBUG,
            "info": LoggingSeverity.INFO,
            "warn": LoggingSeverity.WARN,
            "error": LoggingSeverity.ERROR,
        }
        client_node.get_logger().set_level(log_level_map[args.log_level])

        success = client_node.send_goal(
            process_name=args.process_name,
            fields=fields,
        )

        if not success:
            print("Failed to send goal", file=sys.stderr)
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
            print(f"Error: {e}", file=sys.stderr)
            sys.exit(1)


if __name__ == "__main__":
    main()
