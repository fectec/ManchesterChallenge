#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from rcl_interfaces.msg import SetParametersResult
import numpy as np
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy, QoSDurabilityPolicy

class SetpointPublisherNode(Node):
    def __init__(self):
        super().__init__("SetpointPublisher")

        # Declare parameters
        self.declare_parameter("timer_period", 0.1)         # Timer period
        self.declare_parameter("amplitude", 8.0)            # Signal amplitude
        self.declare_parameter("frequency", 0.05)           # Frequency in Hz

        # Declare new parameters for fixed mode
        self.declare_parameter("hold_single", True)         # If True, maintain a single waveform
        self.declare_parameter("fixed_wave", "step")        # Fixed waveform to use

        # Get parameters
        self.timer_period = self.get_parameter("timer_period").value
        self.amplitude = self.get_parameter("amplitude").value
        self.frequency = self.get_parameter("frequency").value
        self.hold_single = self.get_parameter("hold_single").value
        self.fixed_wave = self.get_parameter("fixed_wave").value

        # Calculate angular frequency
        self.angular_frequency = 2 * np.pi * self.frequency

        # Calculate segment duration (10 cycles of the sine wave)
        self.segment_duration = 10 * (1 / self.frequency)

        # Define a reliable QoS profile according to the template
        qos_profile = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE
        )

        # Create publisher and timer
        self.signal_publisher = self.create_publisher(Float32, "setpoint", qos_profile)
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

        # Output message and reference times
        self.signal_msg = Float32()
        self.start_time = self.get_clock().now()
        self.segment_start_time = self.get_clock().now()

        # Variables to alternate signals
        # List of available waveforms for cyclic mode
        self.wave_types = ["sine", "square", "step", "trapezoidal", "ramp", "sawtooth"]
        self.current_wave_index = 0

        # Callback for dynamic parameters
        self.add_on_set_parameters_callback(self.parameter_callback)

        self.get_logger().info("Setpoint Node Started 🚀")

    def timer_callback(self):
        current_time = self.get_clock().now()
        # Total elapsed time (not used directly in the setpoint)
        elapsed_total = (current_time - self.start_time).nanoseconds / 1e9
        
        if self.hold_single:
            # Fixed signal mode: ignore alternation and use the waveform specified in fixed_wave.
            # If the fixed waveform is not in the list, default to "sine".
            wave_type = self.fixed_wave if self.fixed_wave in self.wave_types else "sine"
            # For most periodic signals, we use the natural period: 1/frequency.
            cycle_period = 1.0 / self.frequency

            if wave_type == "sine":
                self.signal_msg.data = self.amplitude * np.sin(self.angular_frequency * elapsed_total)
            elif wave_type == "square":
                self.signal_msg.data = self.amplitude * np.sign(np.sin(self.angular_frequency * elapsed_total))
            elif wave_type == "sawtooth":
                t = elapsed_total % cycle_period
                self.signal_msg.data = self.amplitude * (t / cycle_period)
            elif wave_type == "trapezoidal":
                # Trapezoidal waveform in each cycle (period = 1/frequency)
                t = elapsed_total % cycle_period
                quarter = cycle_period / 4.0
                if t < quarter:  # Linear ramp up
                    self.signal_msg.data = self.amplitude * (t / quarter)
                elif t < 2 * quarter:  # High plateau
                    self.signal_msg.data = self.amplitude
                elif t < 3 * quarter:  # Linear ramp down
                    self.signal_msg.data = self.amplitude * (1 - (t - 2 * quarter) / quarter)
                else:  # Low plateau
                    self.signal_msg.data = 0.0
            elif wave_type == "step":
                # Step signal that changes in the middle of the cycle
                t = elapsed_total % cycle_period
                if t < cycle_period / 2:
                    self.signal_msg.data = 0.0
                else:
                    self.signal_msg.data = self.amplitude
            elif wave_type == "ramp":
                # Ramp signal that increases linearly during 10 cycles and then resets
                ramp_duration = self.segment_duration  # 10 cycles
                t = elapsed_total % ramp_duration
                self.signal_msg.data = self.amplitude * (t / ramp_duration)
            else:
                self.signal_msg.data = self.amplitude * np.sin(self.angular_frequency * elapsed_total)
        
        else:
            # Cyclic mode: alternate between the available waveforms in wave_types.
            # Elapsed time in the current segment
            segment_elapsed = (current_time - self.segment_start_time).nanoseconds / 1e9

            # Check if the segment duration (10 cycles) is complete
            if segment_elapsed >= self.segment_duration:
                # Switch to the next waveform (cyclically)
                self.current_wave_index = (self.current_wave_index + 1) % len(self.wave_types)
                self.segment_start_time = current_time
                self.get_logger().info(f"Switching to signal: {self.wave_types[self.current_wave_index]}")
                segment_elapsed = 0.0  # Reset segment time

            # Get the current waveform type
            wave_type = self.wave_types[self.current_wave_index]

            # Generate the setpoint based on the waveform type
            if wave_type == "sine":
                # Sine signal: use 10 cycles in the segment
                self.signal_msg.data = self.amplitude * np.sin(self.angular_frequency * segment_elapsed)
            elif wave_type == "square":
                # Square signal
                self.signal_msg.data = self.amplitude * np.sign(np.sin(self.angular_frequency * segment_elapsed))
            elif wave_type == "step":
                # Step signal: change in the middle of the segment
                if segment_elapsed < self.segment_duration / 2:
                    self.signal_msg.data = 0.0
                else:
                    self.signal_msg.data = self.amplitude
            elif wave_type == "trapezoidal":
                # For the trapezoidal signal, generate a trapezoid in each cycle (period = 1/frequency)
                cycle_period = 1.0 / self.frequency
                cycle_elapsed = segment_elapsed % cycle_period
                quarter = cycle_period / 4.0
                if cycle_elapsed < quarter:  # Linear ramp up
                    self.signal_msg.data = self.amplitude * (cycle_elapsed / quarter)
                elif cycle_elapsed < 2 * quarter:  # High plateau
                    self.signal_msg.data = self.amplitude
                elif cycle_elapsed < 3 * quarter:  # Linear ramp down
                    self.signal_msg.data = self.amplitude * (1 - (cycle_elapsed - 2 * quarter) / quarter)
                else:  # Low plateau
                    self.signal_msg.data = 0.0
            elif wave_type == "ramp":
                # Ramp signal: increases linearly from 0 to amplitude over the entire segment (10 cycles)
                self.signal_msg.data = self.amplitude * (segment_elapsed / self.segment_duration)
            elif wave_type == "sawtooth":
                # Sawtooth signal: ramp that resets every cycle (period = 1/frequency)
                cycle_period = 1.0 / self.frequency
                cycle_elapsed = segment_elapsed % cycle_period
                self.signal_msg.data = self.amplitude * (cycle_elapsed / cycle_period)
            else:
                self.get_logger().warn(f"Unknown signal type: {wave_type}. Using sine signal by default.")
                self.signal_msg.data = self.amplitude * np.sin(self.angular_frequency * segment_elapsed)

        # Publish the setpoint message
        self.signal_publisher.publish(self.signal_msg)

    def parameter_callback(self, params):
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
                # Update segment duration if frequency changes (10 cycles)
                self.segment_duration = 10 * (1 / self.frequency)
                # Update angular frequency if frequency changes
                self.angular_frequency = 2 * np.pi * self.frequency
            elif param.name == "timer_period":
                if param.value <= 0.0:
                    self.get_logger().warn("Timer period must be greater than zero.")
                    return SetParametersResult(successful=False, reason="Timer period must be greater than 0.")
                self.timer_period = param.value
                self.timer.cancel()
                self.timer = self.create_timer(self.timer_period, self.timer_callback)
            elif param.name == "hold_single":
                self.hold_single = param.value
                self.get_logger().info(f"Fixed signal mode {'activated' if self.hold_single else 'deactivated'}.")
            elif param.name == "fixed_wave":
                # When updating fixed_wave, check if it is one of the available types.
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