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

        # Declarar parámetros
        self.declare_parameter("timer_period", 0.1)         # Periodo del timer
        self.declare_parameter("amplitude", 8.0)              # Amplitud de la señal
        self.declare_parameter("frequency", 0.05)              # Frecuencia en Hz

        # Declarar nuevos parámetros para modo fijo
        self.declare_parameter("hold_single", True)        # Si True, se mantiene una única forma
        self.declare_parameter("fixed_wave", "step")          # Forma fija a utilizar

        # Obtener parámetros
        self.timer_period = self.get_parameter("timer_period").value
        self.amplitude = self.get_parameter("amplitude").value
        self.frequency = self.get_parameter("frequency").value
        self.hold_single = self.get_parameter("hold_single").value
        self.fixed_wave = self.get_parameter("fixed_wave").value

        # Calcular la frecuencia angular
        self.angular_frequency = 2 * np.pi * self.frequency

        # Calcular la duración del segmento (10 ciclos del seno)
        self.segment_duration = 10 * (1 / self.frequency)

        # Definir un QoS profile confiable según la plantilla
        qos_profile = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE
        )

        # Crear publicador y timer
        self.signal_publisher = self.create_publisher(Float32, "setpoint", qos_profile)
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

        # Mensaje de salida y tiempos de referencia
        self.signal_msg = Float32()
        self.start_time = self.get_clock().now()
        self.segment_start_time = self.get_clock().now()

        # Variables para alternar señales
        # Lista de señales disponibles para el modo cíclico
        self.wave_types = ["sine", "square", "step", "trapezoidal", "ramp", "sawtooth"]
        self.current_wave_index = 0

        # Callback para parámetros dinámicos
        self.add_on_set_parameters_callback(self.parameter_callback)

        self.get_logger().info("Setpoint Node Started 🚀")

    def timer_callback(self):
        current_time = self.get_clock().now()
        # Tiempo transcurrido total (no usado directamente en el setpoint)
        elapsed_total = (current_time - self.start_time).nanoseconds / 1e9
        
        if self.hold_single:
            # Modo de señal fija: se ignora la alternancia y se usa la forma elegida en fixed_wave.
            # Si la forma fija no está en la lista, se usa "sine" por defecto.
            wave_type = self.fixed_wave if self.fixed_wave in self.wave_types else "sine"
            # Para la mayoría de señales periódicas usamos el periodo natural: 1/frequency.
            cycle_period = 1.0 / self.frequency

            if wave_type == "sine":
                self.signal_msg.data = self.amplitude * np.sin(self.angular_frequency * elapsed_total)
            elif wave_type == "square":
                self.signal_msg.data = self.amplitude * np.sign(np.sin(self.angular_frequency * elapsed_total))
            elif wave_type == "sawtooth":
                t = elapsed_total % cycle_period
                self.signal_msg.data = self.amplitude * (t / cycle_period)
            elif wave_type == "trapezoidal":
                # Trapecio en cada ciclo (1/frequency)
                t = elapsed_total % cycle_period
                quarter = cycle_period / 4.0
                if t < quarter:  # Subida lineal
                    self.signal_msg.data = self.amplitude * (t / quarter)
                elif t < 2 * quarter:  # Meseta alta
                    self.signal_msg.data = self.amplitude
                elif t < 3 * quarter:  # Bajada lineal
                    self.signal_msg.data = self.amplitude * (1 - (t - 2 * quarter) / quarter)
                else:  # Meseta baja
                    self.signal_msg.data = 0.0
            elif wave_type == "step":
                # Escalón que cambia en la mitad del ciclo
                t = elapsed_total % cycle_period
                if t < cycle_period / 2:
                    self.signal_msg.data = 0.0
                else:
                    self.signal_msg.data = self.amplitude
            elif wave_type == "ramp":
                # Rampa que aumenta linealmente durante 10 ciclos y se reinicia
                ramp_duration = self.segment_duration  # 10 ciclos
                t = elapsed_total % ramp_duration
                self.signal_msg.data = self.amplitude * (t / ramp_duration)
            else:
                self.signal_msg.data = self.amplitude * np.sin(self.angular_frequency * elapsed_total)
        
        else:
            # Modo cíclico: se alterna entre las señales disponibles en wave_types.
            # Tiempo transcurrido en el segmento actual
            segment_elapsed = (current_time - self.segment_start_time).nanoseconds / 1e9

            # Verificar si se completó la duración del segmento (10 ciclos)
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
                # Señal sinusoidal: se usan 10 ciclos en el segmento
                self.signal_msg.data = self.amplitude * np.sin(self.angular_frequency * segment_elapsed)
            elif wave_type == "square":
                # Señal cuadrada
                self.signal_msg.data = self.amplitude * np.sign(np.sin(self.angular_frequency * segment_elapsed))
            elif wave_type == "step":
                # Señal escalón: cambio a la mitad del segmento
                if segment_elapsed < self.segment_duration / 2:
                    self.signal_msg.data = 0.0
                else:
                    self.signal_msg.data = self.amplitude
            elif wave_type == "trapezoidal":
                # Para la señal trapezoidal, se genera un trapecio en cada ciclo (periodo = 1/frequency)
                cycle_period = 1.0 / self.frequency
                cycle_elapsed = segment_elapsed % cycle_period
                quarter = cycle_period / 4.0
                if cycle_elapsed < quarter:  # Subida lineal
                    self.signal_msg.data = self.amplitude * (cycle_elapsed / quarter)
                elif cycle_elapsed < 2 * quarter:  # Meseta alta
                    self.signal_msg.data = self.amplitude
                elif cycle_elapsed < 3 * quarter:  # Bajada lineal
                    self.signal_msg.data = self.amplitude * (1 - (cycle_elapsed - 2 * quarter) / quarter)
                else:  # Meseta baja
                    self.signal_msg.data = 0.0
            elif wave_type == "ramp":
                # Señal rampa: aumenta linealmente de 0 a la amplitud durante el segmento completo (10 ciclos)
                self.signal_msg.data = self.amplitude * (segment_elapsed / self.segment_duration)
            elif wave_type == "sawtooth":
                # Señal de sierra: rampa que se reinicia cada ciclo (periodo = 1/frequency)
                cycle_period = 1.0 / self.frequency
                cycle_elapsed = segment_elapsed % cycle_period
                self.signal_msg.data = self.amplitude * (cycle_elapsed / cycle_period)
            else:
                self.get_logger().warn(f"Tipo de señal desconocido: {wave_type}. Usando señal sinusoidal por defecto.")
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
                # Actualizar la duración del segmento si la frecuencia cambia (10 ciclos)
                self.segment_duration = 10 * (1 / self.frequency)
                # Actualizar la frecuencia angular si la frecuencia cambia
                self.angular_frequency = 2 * np.pi * self.frequency
            elif param.name == "timer_period":
                if param.value <= 0.0:
                    self.get_logger().warn("El periodo del timer debe ser mayor que cero.")
                    return SetParametersResult(successful=False, reason="Timer period must be greater than 0.")
                self.timer_period = param.value
                self.timer.cancel()
                self.timer = self.create_timer(self.timer_period, self.timer_callback)
            elif param.name == "hold_single":
                self.hold_single = param.value
                self.get_logger().info(f"Modo señal fija {'activado' if self.hold_single else 'desactivado'}.")
            elif param.name == "fixed_wave":
                # Si se actualiza el fixed_wave, se verifica si es uno de los tipos disponibles.
                if param.value not in self.wave_types:
                    self.get_logger().warn(f"Fixed wave '{param.value}' no reconocido. Se usará 'sine' por defecto.")
                    self.fixed_wave = "sine"
                else:
                    self.fixed_wave = param.value
                self.get_logger().info(f"Señal fija actualizada a: {self.fixed_wave}")

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
