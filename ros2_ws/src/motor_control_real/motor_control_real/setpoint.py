#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from rcl_interfaces.msg import SetParametersResult
import numpy as np
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy, QoSDurabilityPolicy

class SetpointPublisherNode(Node):
    """
    This node publishes a configurable sine, square, or custom waveform as a setpoint signal.
    It can operate in cyclic mode (alternating waveforms) or fixed mode (holding a single waveform).
    Supports dynamic parameter updates and simulation toggling via service.
    """
    def __init__(self):
        super().__init__("SetpointPublisher")

        # Declare parameters for controlling waveform behavior
        self.declare_parameter("timer_period", 0.1)         # Timer period (s)
        self.declare_parameter("amplitude", 8.0)            # Signal amplitude
        self.declare_parameter("frequency", 0.05)           # Frequency (Hz)

        # Declare new parameters for fixed mode
        self.declare_parameter("hold_single", True)         # If True, maintain a single waveform
        self.declare_parameter("fixed_wave", "step")        # Fixed waveform to use (step, sine, etc.)

        # Get parameters as variables
        self.timer_period = self.get_parameter("timer_period").value
        self.amplitude = self.get_parameter("amplitude").value
        self.frequency = self.get_parameter("frequency").value
        self.hold_single = self.get_parameter("hold_single").value
        self.fixed_wave = self.get_parameter("fixed_wave").value

        # Calculate angular frequency (2*pi*f)
        self.angular_frequency = 2 * np.pi * self.frequency

        # Segment duration for alternating waveform (10 cycles of the sine wave)
        self.segment_duration = 10 * (1 / self.frequency)

        # Define a reliable QoS profile for communication
        qos_profile = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE
        )

        # Create publisher for setpoint signal and a timer for periodic callbacks
        self.signal_publisher = self.create_publisher(Float32, "setpoint", qos_profile)
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

        # Initialize message and reference times for signal generation
        self.signal_msg = Float32()
        self.start_time = self.get_clock().now()
        self.segment_start_time = self.get_clock().now()

        # Available waveforms for cyclic mode
        self.wave_types = ["sine", "square", "step", "trapezoidal", "ramp", "sawtooth"]
        self.current_wave_index = 0

        # Set up dynamic parameter callback
        self.add_on_set_parameters_callback(self.parameter_callback)

        self.get_logger().info("Setpoint Node Started 🚀")

    def timer_callback(self):
        """
        Generates and publishes the setpoint waveform based on parameters and mode (fixed or cyclic).
        If `hold_single` is True, a fixed waveform (like sine, square, etc.) is maintained.
        Otherwise, cyclically alternates between available waveforms.
        """
        current_time = self.get_clock().now()
        elapsed_total = (current_time - self.start_time).nanoseconds / 1e9  # Elapsed time in seconds
        
        if self.hold_single:
            # In fixed mode, use the waveform specified in `fixed_wave`
            wave_type = self.fixed_wave if self.fixed_wave in self.wave_types else "sine"
            cycle_period = 1.0 / self.frequency  # One period of the waveform

            # Generate waveform based on selected type
            if wave_type == "sine":
                self.signal_msg.data = self.amplitude * np.sin(self.angular_frequency * elapsed_total)
            elif wave_type == "square":
                self.signal_msg.data = self.amplitude * np.sign(np.sin(self.angular_frequency * elapsed_total))
            elif wave_type == "sawtooth":
                t = elapsed_total % cycle_period
                self.signal_msg.data = self.amplitude * (t / cycle_period)
            elif wave_type == "trapezoidal":
                t = elapsed_total % cycle_period
                quarter = cycle_period / 4.0
                if t < quarter:
                    self.signal_msg.data = self.amplitude * (t / quarter)  # Ramp up
                elif t < 2 * quarter:
                    self.signal_msg.data = self.amplitude  # High plateau
                elif t < 3 * quarter:
                    self.signal_msg.data = self.amplitude * (1 - (t - 2 * quarter) / quarter)  # Ramp down
                else:
                    self.signal_msg.data = 0.0  # Low plateau
            elif wave_type == "step":
                t = elapsed_total % cycle_period
                self.signal_msg.data = 0.0 if t < cycle_period / 2 else self.amplitude
            elif wave_type == "ramp":
                ramp_duration = self.segment_duration
                t = elapsed_total % ramp_duration
                self.signal_msg.data = self.amplitude * (t / ramp_duration)
            else:
                self.signal_msg.data = self.amplitude * np.sin(self.angular_frequency * elapsed_total)
        
        else:
            # In cyclic mode, alternate between waveforms in `wave_types`
            segment_elapsed = (current_time - self.segment_start_time).nanoseconds / 1e9
            if segment_elapsed >= self.segment_duration:
                # Cycle through waveforms
                self.current_wave_index = (self.current_wave_index + 1) % len(self.wave_types)
                self.segment_start_time = current_time
                self.get_logger().info(f"Switching to signal: {self.wave_types[self.current_wave_index]}")
                segment_elapsed = 0.0

            wave_type = self.wave_types[self.current_wave_index]

            # Generate setpoint based on current waveform type
            if wave_type == "sine":
                self.signal_msg.data = self.amplitude * np.sin(self.angular_frequency * segment_elapsed)
            elif wave_type == "square":
                self.signal_msg.data = self.amplitude * np.sign(np.sin(self.angular_frequency * segment_elapsed))
            elif wave_type == "step":
                self.signal_msg.data = 0.0 if segment_elapsed < self.segment_duration / 2 else self.amplitude
            elif wave_type == "trapezoidal":
                cycle_period = 1.0 / self.frequency
                cycle_elapsed = segment_elapsed % cycle_period
                quarter = cycle_period / 4.0
                if cycle_elapsed < quarter:
                    self.signal_msg.data = self.amplitude * (cycle_elapsed / quarter)
                elif cycle_elapsed < 2 * quarter:
                    self.signal_msg.data = self.amplitude
                elif cycle_elapsed < 3 * quarter:
                    self.signal_msg.data = self.amplitude * (1 - (cycle_elapsed - 2 * quarter) / quarter)
                else:
                    self.signal_msg.data = 0.0
            elif wave_type == "ramp":
                self.signal_msg.data = self.amplitude * (segment_elapsed / self.segment_duration)
            elif wave_type == "sawtooth":
                cycle_period = 1.0 / self.frequency
                cycle_elapsed = segment_elapsed % cycle_period
                self.signal_msg.data = self.amplitude * (cycle_elapsed / cycle_period)
            else:
                self.get_logger().warn(f"Unknown wave type: {wave_type}. Using sine wave.")
                self.signal_msg.data = self.amplitude * np.sin(self.angular_frequency * segment_elapsed)

        # Publish the generated setpoint
        self.signal_publisher.publish(self.signal_msg)

    def parameter_callback(self, params):
        """
        Handles dynamic parameter updates.
        Updates parameters such as amplitude, frequency, timer period, and waveform type.
        """
        for param in params:
            if param.name == "amplitude":
                if param.value < 0.0:
                    self.get_logger().warn("Amplitude must be non-negative.")
                    return SetParametersResult(successful=False, reason="Amplitude cannot be negative.")
                self.amplitude = param.value
            elif param.name == "frequency":
                if param.value <= 0.0:
                    self.get_logger().warn("Frequency must be positive.")
                    return SetParametersResult(successful=False, reason="Frequency must be positive.")
                self.frequency = param.value
                self.segment_duration = 10 * (1 / self.frequency)  # Update duration if frequency changes
                self.angular_frequency = 2 * np.pi * self.frequency
            elif param.name == "timer_period":
                if param.value <= 0.0:
                    self.get_logger().warn("Timer period must be greater than zero.")
                    return SetParametersResult(successful=False, reason="Timer period must be greater than zero.")
                self.timer_period = param.value
                self.timer.cancel()
                self.timer = self.create_timer(self.timer_period, self.timer_callback)
            elif param.name == "hold_single":
                self.hold_single = param.value
                self.get_logger().info(f"Fixed signal mode {'activated' if self.hold_single else 'deactivated'}.")
            elif param.name == "fixed_wave":
                if param.value not in self.wave_types:
                    self.get_logger().warn(f"Fixed wave '{param.value}' not recognized. Defaulting to 'sine'.")
                    self.fixed_wave = "sine"
                else:
                    self.fixed_wave = param.value
                self.get_logger().info(f"Fixed wave updated to: {self.fixed_wave}")

        return SetParametersResult(successful=True)

def main(args=None):
    rclpy.init(args=args)
    node = SetpointPublisherNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()