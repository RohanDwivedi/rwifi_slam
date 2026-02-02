import rclpy
from rclpy.node import Node
from rclpy.parameter import ParameterDescriptor

from wifi_rtt_msgs.msg import APRange, APRangeScan


class WifiRttPublisher(Node):
    def __init__(self):
        super().__init__("wifi_rtt_publisher")

        # publisher frequency rate limited by wifi IEEE 802.11mc standard
        self.publisher_ = self.create_publisher(APRangeScan, "wifi_rtt_scan", 5)

        self.declare_parameter(
            "select_backend",
            "simulation",
            descriptor=ParameterDescriptor(
                description="Select the backend to use for Wi-Fi RTT scanning. Options: simulation, android"
            ),
        )

        self.backend = self.get_parameter("select_backend").value
        
        if self.backend == "simulation":
            from wifi_rtt_interface.simulation_backend import SimulationBackend

            self.backend_interface = SimulationBackend()
            self.backend_interface.start()

        elif self.backend == "android":
            from wifi_rtt_interface.android_backend import AndroidBackend

            self.backend_interface = AndroidBackend()
            self.backend_interface.start()

        else:
            self.get_logger().error(f"Invalid backend: {self.backend}")
            raise ValueError(f"Invalid backend: {self.backend}")


    def publish_scan(self):
        self.backend_interface.get_rtt()
        

def main(args=None):
    rclpy.init(args=args)
    node = WifiRttPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.destroy_node()
        rclpy.shutdown()
