#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from rcl_interfaces.msg import SetParametersResult
import numpy as np

class SetpointPublisherNode(Node):

    def __init__(self):
        super().__init__("SetpointPublisher")

        # Declarar parámetros
        self.declare_parameter("timer_period", 0.1)         # Periodo del timer
        self.declare_parameter("amplitude", 1.0)              # Amplitud de la señal
        self.declare_parameter("frequency", 1.0)      # Frecuencia

        # Obtener parámetros
        self.timer_period = self.get_parameter("timer_period").value
        self.amplitude = self.get_parameter("amplitude").value
        self.frequency = self.get_parameter("frequency").value

        # Calcular la frecuencia angular
        self.angular_frequency = 2 * np.pi * self.frequency

        # Calcular la duración del segmento (10 ciclos del seno)
        self.segment_duration = 10 * ( 1 / self.frequency)

        # Crear publicador y timer
        self.signal_publisher = self.create_publisher(Float32, "setpoint", 10)
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

        # Mensaje de salida y tiempo de inicio
        self.signal_msg = Float32()
        self.start_time = self.get_clock().now()

        # Variables para alternar señales
        self.wave_types = ["sine", "square", "step", "trapezoidal", "ramp"]
        self.current_wave_index = 0
        self.segment_start_time = self.get_clock().now()

        # Callback para parámetros dinámicos
        self.add_on_set_parameters_callback(self.parameter_callback)

        self.get_logger().info("Setpoint Node Started 🚀")

    def timer_callback(self):
        current_time = self.get_clock().now()
        # Tiempo transcurrido total (no usado directamente en el setpoint)
        elapsed_total = (current_time - self.start_time).nanoseconds / 1e9
        # Tiempo transcurrido en el segmento actual
        segment_elapsed = (current_time - self.segment_start_time).nanoseconds / 1e9

        # Verificar si se completó la duración del segmento
        if segment_elapsed >= self.segment_duration:
            # Pasar al siguiente tipo de señal (cíclicamente)
            self.current_wave_index = (self.current_wave_index + 1) % len(self.wave_types)
            self.segment_start_time = current_time
            self.get_logger().info(f"Cambiando a la señal: {self.wave_types[self.current_wave_index]}")
            segment_elapsed = 0.0  # Reiniciar el tiempo del segmento

        # Obtener el tipo de señal actual
        wave_type = self.wave_types[self.current_wave_index]

        # Generar el setpoint según el tipo de señal
        if wave_type == "sine":
            self.signal_msg.data = self.amplitude * np.sin(self.angular_frequency * segment_elapsed)
        elif wave_type == "square":
            self.signal_msg.data = self.amplitude * np.sign(np.sin(self.angular_frequency * segment_elapsed))
        elif wave_type == "step":
            # Ejemplo: señal escalón que cambia a la mitad del segmento
            if segment_elapsed < self.segment_duration / 2:
                self.signal_msg.data = 0.0
            else:
                self.signal_msg.data = self.amplitude
        elif wave_type == "trapezoidal":
            # Dividir el segmento en 4 partes iguales
            quarter = self.segment_duration / 4
            if segment_elapsed < quarter:  # Subida lineal
                self.signal_msg.data = self.amplitude * (segment_elapsed / quarter)
            elif segment_elapsed < 2 * quarter:  # Meseta alta
                self.signal_msg.data = self.amplitude
            elif segment_elapsed < 3 * quarter:  # Bajada lineal
                self.signal_msg.data = self.amplitude * (1 - (segment_elapsed - 2 * quarter) / quarter)
            else:  # Meseta baja
                self.signal_msg.data = 0.0
        elif wave_type == "ramp":
            # Señal en rampa: aumenta linealmente de 0 a la amplitud en el segmento
            self.signal_msg.data = self.amplitude * (segment_elapsed / self.segment_duration)
        else:
            self.get_logger().warn(f"Tipo de señal desconocido: {wave_type}. Usando senal sinusoidal por defecto.")
            self.signal_msg.data = self.amplitude * np.sin(self.angular_frequency * segment_elapsed)

        # Publicar el mensaje de setpoint
        self.signal_publisher.publish(self.signal_msg)

    def parameter_callback(self, params):
        for param in params:
            if param.name == "amplitude":
                if param.value < 0.0:
                    self.get_logger().warn("La amplitud debe ser no negativa.")
                    return SetParametersResult(successful=False, reason="Amplitude cannot be negative.")
                self.amplitude = param.value
            elif param.name == "frequency":
                if param.value <= 0.0:
                    self.get_logger().warn("La frecuencia debe ser positiva.")
                    return SetParametersResult(successful=False, reason="Frequency must be positive.")
                self.frequency = param.value
                # Actualizar la duración del segmento si la frecuencia cambia
                self.segment_duration = 10 * (1/ self.frequency)
                # Actualizar la frecuencia angular si la frecuencia cambia
                self.angular_frequency = 2 * np.pi * self.frequency
            elif param.name == "timer_period":
                if param.value <= 0.0:
                    self.get_logger().warn("El periodo del timer debe ser mayor que cero.")
                    return SetParametersResult(successful=False, reason="Timer period must be greater than 0.")
                self.timer_period = param.value
                self.timer.cancel()
                self.timer = self.create_timer(self.timer_period, self.timer_callback)
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
