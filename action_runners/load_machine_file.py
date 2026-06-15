#!/usr/bin/env python3

"""LoadMachineFile Action Client for ROS2."""

import argparse
import os
import sys

import rclpy
from process_control_interfaces.action import LoadMachineFile
from process_control_interfaces_utils.process_control_interfaces_utils import (
    LoadMachineFileFeedbackEnum,
    LoadMachineFileGoalActionEnum,
    LoadMachineFileResultEnum,
)
from rclpy.action import ActionClient
from rclpy.action.client import ClientGoalHandle
from rclpy.executors import SingleThreadedExecutor
from rclpy.logging import LoggingSeverity
from rclpy.node import Node
from rclpy.task import Future


class LoadMachineFileActionClient(Node):
    """ROS2 Action Client for LoadMachineFile action."""

    def __init__(self, node_name: str = "load_machine_file_client", action_server_name: str = "load_machine_file") -> None:
        """
        Initialize the LoadMachineFile action client.

        Args:
            node_name: Name for the ROS2 node.
            action_server_name: Name of the action server to connect to.
        """
        super().__init__(node_name)
        self._action_client = ActionClient(self, LoadMachineFile, action_server_name)
        self.get_logger().info(f"LoadMachineFile Action Client initialized for server '{action_server_name}'")

    def send_goal(
        self, file_path: str, machine_type: str, file_type_version: str, file_id: str, action: LoadMachineFileGoalActionEnum
    ) -> bool:
        """
        Send goal to the action server.

        Args:
            file_path: Path to the machine file to load.
            machine_type: Type of machine (e.g., CNC_MILL, ROBOT_ARM).
            file_type_version: Version of the file type format.
            file_id: Unique identifier for the machine file.
            action: Action to perform (LOAD or CHECK).

        Returns:
            True if goal was sent successfully, False otherwise.
        """
        if not self._action_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error("Action server not available after waiting")
            return False

        file_data = []
        if file_path:
            try:
                with open(file_path, "rb") as f:
                    file_data = list(f.read())
            except FileNotFoundError:
                self.get_logger().error(f"File not found: {file_path}")
                return False
            except Exception as e:
                self.get_logger().error(f"Error reading file: {e}")
                return False

        goal_msg = LoadMachineFile.Goal()
        goal_msg.file.machine_type = machine_type
        goal_msg.file.file_type_version = file_type_version
        goal_msg.file.file_id = file_id
        goal_msg.file.data = file_data
        goal_msg.action = action.value

        action_name = "LOAD" if action == LoadMachineFileGoalActionEnum.LOAD else "CHECK"
        self.get_logger().info(f"Sending {action_name} goal")

        if file_path:
            self.get_logger().info(f"File path: {file_path}")
            self.get_logger().info(f"File size: {len(file_data)} bytes")

        if file_id:
            self.get_logger().info(f"File ID: {file_id}")

        if machine_type:
            self.get_logger().info(f"Machine type: {machine_type}")

        if file_type_version:
            self.get_logger().info(f"File type version: {file_type_version}")

        self._send_goal_future = self._action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        self._send_goal_future.add_done_callback(self.goal_response_callback)
        return True

    def goal_response_callback(self, future: Future) -> None:
        """
        Handle goal response from action server.

        Args:
            future: Future containing the goal handle.
        """
        goal_handle: ClientGoalHandle = future.result()

        if not goal_handle.accepted:
            self.get_logger().error("Goal rejected by action server")
            return

        self.get_logger().info("Goal accepted by action server")

        self._get_result_future: Future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def feedback_callback(self, feedback_msg) -> None:
        """
        Handle feedback from action server.

        Args:
            feedback_msg: Feedback message from the action server.
        """
        feedback = feedback_msg.feedback

        try:
            status_enum = LoadMachineFileFeedbackEnum(feedback.status)
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
        result: LoadMachineFile.Result = future.result().result

        try:
            result_enum = LoadMachineFileResultEnum(result.result)
            result_name = result_enum.name
        except ValueError:
            result_name = f"UNKNOWN_RESULT({result.result})"

        if result.result == LoadMachineFileResultEnum.SUCCESS:
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


def main() -> None:
    """Main function with argument parsing and execution."""
    parser = argparse.ArgumentParser(
        description="ROS2 Action Client for LoadMachineFile",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Load a machine file by path
  python3 load_machine_file.py --file-path /path/to/machine.json --action load --action-server-name /buffer/load_machine_file

  # Check a machine file by ID
  python3 load_machine_file.py --file-id "machine_001" --action check --action-server-name /buffer/load_machine_file

  # Load with all optional parameters
  python3 load_machine_file.py --file-path /path/to/machine.xml \\
                               --machine-type "ROBOT_ARM" \\
                               --file-type-version "2.1" \\
                               --action load \\
                               --action-server-name /buffer/load_machine_file
        """,
    )

    parser.add_argument("--file-path", "-f", type=str, default="", help="Path to the machine file to load or check")
    parser.add_argument(
        "--file-id",
        "-i",
        type=str,
        default="",
        help="Unique identifier for the machine file (overrides file content if both are provided)",
    )
    parser.add_argument(
        "--machine-type",
        "-t",
        type=str,
        default="",
        help="Type of machine (optional, e.g., CNC_MILL, ROBOT_ARM, 3D_PRINTER)",
    )
    parser.add_argument(
        "--file-type-version", "-v", type=str, default="", help="Version of the file type format (optional, e.g., 1.0, 2.1)"
    )
    parser.add_argument(
        "--action-server-name",
        "-s",
        type=str,
        required=True,
        help="Name of the LoadMachineFile action server (e.g., /buffer/load_machine_file)",
    )
    parser.add_argument(
        "--action",
        "-a",
        type=str,
        choices=["load", "check"],
        required=True,
        help="Action to perform: load or check the machine file",
    )
    parser.add_argument(
        "--node-name",
        "-n",
        type=str,
        default="load_machine_file_client",
        help="Name for the ROS2 node (default: load_machine_file_client)",
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

    # Validate file path if provided
    if args.file_path and not os.path.isfile(args.file_path):
        print(f"Error: File does not exist: {args.file_path}", file=sys.stderr)
        sys.exit(1)

    # Convert action string to enum
    action_map = {"load": LoadMachineFileGoalActionEnum.LOAD, "check": LoadMachineFileGoalActionEnum.CHECK}
    action_enum = action_map[args.action.lower()]

    # Initialize ROS2
    rclpy.init(args=None)

    try:
        # Create action client node
        client_node = LoadMachineFileActionClient(args.node_name, args.action_server_name)

        # Set log level
        log_level_map = {
            "debug": LoggingSeverity.DEBUG,
            "info": LoggingSeverity.INFO,
            "warn": LoggingSeverity.WARN,
            "error": LoggingSeverity.ERROR,
        }
        client_node.get_logger().set_level(log_level_map[args.log_level])

        # Send goal
        success = client_node.send_goal(
            file_path=args.file_path,
            machine_type=args.machine_type,
            file_type_version=args.file_type_version,
            file_id=args.file_id,
            action=action_enum,
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
