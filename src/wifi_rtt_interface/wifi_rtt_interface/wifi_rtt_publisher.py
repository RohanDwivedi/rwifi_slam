"""ROS 2 node that publishes Wi-Fi RTT scan observations from a configurable backend."""

import rclpy
from rclpy.node import Node
from wifi_rtt_msgs.msg import APRangeObservation


class WifiRttPublisher(Node):
    """Publishes access-point range observations from simulation or Android backend.

    Publishes on the wifi_rtt_scan topic at 5 Hz, consistent with the IEEE 802.11mc
    round-trip time (RTT) scan rate limit.
    """

    def __init__(self) -> None:
        """Initialize the node, create publisher, and configure backend from parameters.

        Loads backend_type from parameters and instantiates the corresponding backend
        (e.g. SimulationBackend for simulation_isaac_sim). Starts a 5 Hz timer that
        delegates to the backend to fetch and publish observations.

        Raises:
            NotImplementedError: If backend_type is 'android' (not implemented).
            ValueError: If backend_type is not 'simulation_isaac_sim' or 'android'.
        """
        super().__init__(
            "wifi_rtt_publisher",
            allow_undeclared_parameters=True,
            automatically_declare_parameters_from_overrides=True,
        )

        self.publisher_ = self.create_publisher(APRangeObservation, "wifi_rtt_scan", 5)
        self.use_sim_time = self.get_parameter("use_sim_time").value
        self.clock = self.get_clock()

        self.backend = self.get_parameter("backend_type").value
        if self.backend == "simulation_isaac_sim":
            from wifi_rtt_interface.simulation_backend import SimulationBackend

            self.backend_interface = SimulationBackend(
                simulation_type=self.backend,
                clock=self.clock,
                node=self,
                publisher=self.publisher_,
            )
        elif self.backend == "android":
            raise NotImplementedError("Android backend not implemented yet")
        elif self.backend not in ["simulation_isaac_sim", "android"]:
            self.get_logger().error(f"Invalid backend type: {self.backend}")
            raise ValueError(f"Invalid backend type: {self.backend}")

        self._timer = self.create_timer(0.2, self._on_timer)

    def _on_timer(self) -> None:
        """Invoke the backend to fetch the current RTT observation and publish it."""
        self.backend_interface.publish_observation()


def main(args: list[str] | None = None) -> None:
    """Create the Wi-Fi RTT publisher node and run the ROS 2 spin loop.

    Args:
        args: Command-line arguments forwarded to rclpy.init. If None, sys.argv is used.
    """
    rclpy.init(args=args)
    node = WifiRttPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.destroy_node()
        rclpy.shutdown()
