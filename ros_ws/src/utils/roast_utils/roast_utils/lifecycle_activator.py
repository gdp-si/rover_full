"""Activate and deactivate ROS2 lifecycle nodes."""

import argparse
from typing import List

import rclpy
from lifecycle_msgs.msg import State, Transition
from lifecycle_msgs.srv import ChangeState, GetState
from rclpy.lifecycle import LifecycleNode
from rclpy.node import Node


class LifecycleActivator(Node):
    """Activate and deactivate ROS2 lifecycle nodes."""

    def __init__(self, node_name: str, lifecycle_nodes: List[LifecycleNode]):
        """Initialize LifecycleActivator."""
        super().__init__(node_name)
        self._lifecycle_nodes = lifecycle_nodes
        self._change_state_services = []
        self._get_state_services = []
        self._transition_map = {
            Transition.TRANSITION_CONFIGURE: State.PRIMARY_STATE_INACTIVE,
            Transition.TRANSITION_ACTIVATE: State.PRIMARY_STATE_ACTIVE,
            Transition.TRANSITION_DEACTIVATE: State.PRIMARY_STATE_INACTIVE,
            Transition.TRANSITION_CLEANUP: State.PRIMARY_STATE_UNCONFIGURED,
            Transition.TRANSITION_DESTROY: State.PRIMARY_STATE_FINALIZED,
        }

        for node in self._lifecycle_nodes:
            self._change_state_services.append(
                self.create_client(ChangeState, node + "/change_state")
            )
            self._get_state_services.append(
                self.create_client(GetState, node + "/get_state")
            )

    def activate(self):
        """Activate lifecycle nodes."""
        self._change_state(Transition.TRANSITION_CONFIGURE)
        self._change_state(Transition.TRANSITION_ACTIVATE)

    def deactivate(self):
        """Deactivate lifecycle nodes."""
        self._change_state(Transition.TRANSITION_DEACTIVATE)
        self._change_state(Transition.TRANSITION_CLEANUP)

    def _change_state(self, transition: Transition):
        for node, change_state_service, get_state_service in zip(
            self._lifecycle_nodes, self._change_state_services, self._get_state_services
        ):
            self.get_logger().info(f"Changing state of {node}.")
            # Prepare transition request
            request = ChangeState.Request()
            request.transition = Transition(id=transition)
            future = change_state_service.call_async(request)
            rclpy.spin_until_future_complete(self, future)
            response = future.result()
            if response.success:
                self.get_logger().info(
                    f"Transition {transition} for {node} successful."
                )
            else:
                self.get_logger().error(f"Transition {transition} for {node} failed.")
            future = get_state_service.call_async(GetState.Request())
            rclpy.spin_until_future_complete(self, future)

            response = future.result()
            if response.current_state.id == self._transition_map[transition]:
                self.get_logger().info(
                    f"{node} is in state {response.current_state.label}."
                )
            else:
                self.get_logger().error(
                    f"{node} is in state {response.current_state.label} "
                    f"instead of {self._transition_map[transition]}."
                )


def main():
    """Run LifecycleActivator."""
    parser = argparse.ArgumentParser(
        description="Activate and deactivate ROS2 lifecycle nodes."
    )
    parser.add_argument(
        "--activate",
        action="store_true",
        help="Activate lifecycle nodes.",
    )
    parser.add_argument(
        "--deactivate",
        action="store_true",
        help="Deactivate lifecycle nodes.",
    )
    parser.add_argument(
        "--node-name", type=str, help="Name of the node or list of nodes."
    )
    args = parser.parse_args()

    rclpy.init()
    node = LifecycleActivator("lifecycle_activator", [args.node_name])

    if args.activate:
        node.activate()
    elif args.deactivate:
        node.deactivate()
    else:
        print("Please specify --activate or --deactivate.")
    rclpy.shutdown()


if __name__ == "__main__":
    main()
